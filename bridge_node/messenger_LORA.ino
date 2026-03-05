// ============================================================
//  ESP-NOW <-> LoRa Bridge Node
//  LoRa module: REYAX RYLR998 (AT command, UART)
//
//  Wiring:
//    RYLR998 VCC  → 3.3V
//    RYLR998 GND  → GND
//    RYLR998 TXD  → GPIO 16  (ESP32 RX2)
//    RYLR998 RXD  → GPIO 17  (ESP32 TX2)
//    RYLR998 RST  → GPIO 4   (optional hard reset)
//    LED          → GPIO 2
//
//  RYLR998 default UART: 115200 8N1
//
//  Role:
//    • Beacons over ESP-NOW → nodes pick strongest bridge by RSSI
//    • ESP-NOW cluster → LoRa backbone (with LORACK on handoff)
//    • LoRa backbone → ESP-NOW cluster (re-broadcast inbound)
//    • TTL mesh forwarding + deduplication on both sides
// ============================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ── Debug logging ─────────────────────────────────────────────
// Uncomment to enable verbose serial logging and debug commands
//#define DEBUG
#ifdef DEBUG
  #define DLOG(...) Serial.printf(__VA_ARGS__)
  #define DLOGLN(s) Serial.println(s)
#else
  #define DLOG(...) do {} while(0)
  #define DLOGLN(s) do {} while(0)
#endif

// ── Display ──────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  32
#define OLED_RESET     -1
#define ROW0   0
#define ROW1   8
#define ROW2  16
#define ROW3  24
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── RYLR998 UART ─────────────────────────────────────────────
#define LORA_SERIAL   Serial2
#define LORA_RX_PIN   16
#define LORA_TX_PIN   17
#define LORA_RST_PIN   4     // pull LOW to hard-reset; -1 to disable
#define LORA_BAUD  115200

// RYLR998 network parameters — must match on every bridge
#define RYLR_NETWORK_ID   18      // AT+NETWORKID  (3–15, or 18 for public)
#define RYLR_FREQUENCY    868     // AT+BAND in MHz (868 EU / 915 US)
#define RYLR_SF            9      // AT+PARAMETER SF  (7–12)
#define RYLR_BW          125      // AT+PARAMETER BW  (125 / 250 / 500 kHz → 7/8/9)
#define RYLR_CR            1      // AT+PARAMETER CR  (1=4/5 … 4=4/8)
#define RYLR_PREAMBLE      8      // AT+PARAMETER preamble
// RYLR998 address of THIS bridge (0–65535; use bridge index)
#define RYLR_ADDR          1      // must be unique per bridge
// Broadcast address for RYLR998 — send to address 0 = all nodes in network
#define RYLR_BCAST_ADDR    0

// ── Bridge / mesh identity ───────────────────────────────────
#define MY_ADDR        0xB1    // mesh address of this bridge
#define BROADCAST_ADDR 0xFF
#define MESH_TTL       3
#define DEDUP_CACHE    24
#define MSG_ID_NONE    0xFFFF
#define MAX_MESSAGE_LENGTH 30

// ── Timings ──────────────────────────────────────────────────
#define BEACON_INTERVAL_MS    2000
#define LORA_RETRY_TIMEOUT_MS 3000   // wait 3s per attempt for LoRa ACK
#define LORA_MAX_RETRIES         1   // 1 retry → give-up + failAck at ~6s
// AT+SEND timeout: RYLR998 responds +OK immediately after accepting the command,
// but needs ~1.5s at SF9/125kHz to finish transmitting. Give 2s to be safe.
#define AT_SEND_TIMEOUT_MS    2000
// Guard time after +OK before sending another AT command (radio still on-air)
#define LORA_TX_COOLDOWN_MS   1800

// ── LED ──────────────────────────────────────────────────────
#define LED_PIN 2

// ============================================================
//  PACKET FORMAT  (shared with end-node firmware)
// ============================================================
struct MeshPacket {
  uint8_t  type;
  uint16_t msgId;
  uint8_t  src;
  uint8_t  dst;
  uint8_t  ttl;
  int8_t   rssi;
  char     payload[MAX_MESSAGE_LENGTH + 1];
};

#define PKT_DATA   0x01
#define PKT_ACK    0x02
#define PKT_BEACON 0x03
#define PKT_LORACK 0x04

// ── LoRa pending queue ────────────────────────────────────────
// One slot per in-flight message over LoRa backbone.
// With N nodes all sending simultaneously, we need N slots.
#define LORA_PENDING_SLOTS 4
struct LoRaPending {
  bool       active;
  MeshPacket pkt;
  uint8_t    origSenderMac[6];
  unsigned long sentAt;
  int        retries;
};
LoRaPending loraPendingSlots[LORA_PENDING_SLOTS];

// ============================================================
//  RYLR998 DRIVER
// ============================================================

// The RYLR998 sends/receives packets as:
//   AT+SEND=<addr>,<length>,<hex data>\r\n
//   +RCV=<addr>,<length>,<hex data>,<rssi>,<snr>\r\n
//
// We encode MeshPacket as hex so it survives the ASCII transport.
// Each byte → 2 hex chars; sizeof(MeshPacket) bytes → 2*sizeof bytes of hex.
// sizeof(MeshPacket) = 1+2+1+1+1+1+31 = 38 bytes → 76 hex chars.
// AT+SEND line overhead is small; well within RYLR998's 240-byte payload limit.

#define PKT_HEX_LEN  (sizeof(MeshPacket) * 2)  // hex-encoded length

// Line buffer for incoming AT responses
#define AT_LINE_BUF 160
char   atLineBuf[AT_LINE_BUF];
int    atLinePos = 0;

// RYLR998 receive ring buffer (filled from parseLoRaLine, drained in loop)
#define LORA_RX_BUF 4
struct LoRaRxEntry { MeshPacket pkt; int rssi; bool used; };
LoRaRxEntry loraRxBuf[LORA_RX_BUF];
int loraRxWrite = 0;
int loraRxRead  = 0;

// ── Hex encode / decode ───────────────────────────────────────
static const char hexChars[] = "0123456789ABCDEF";

void bytesToHex(const uint8_t* src, int len, char* dst) {
  for (int i = 0; i < len; i++) {
    dst[i * 2]     = hexChars[(src[i] >> 4) & 0x0F];
    dst[i * 2 + 1] = hexChars[ src[i]       & 0x0F];
  }
  dst[len * 2] = '\0';
}

bool hexToBytes(const char* src, int hexLen, uint8_t* dst) {
  if (hexLen % 2 != 0) return false;
  for (int i = 0; i < hexLen / 2; i++) {
    char hi = src[i * 2];
    char lo = src[i * 2 + 1];
    auto val = [](char c) -> int {
      if (c >= '0' && c <= '9') return c - '0';
      if (c >= 'A' && c <= 'F') return c - 'A' + 10;
      if (c >= 'a' && c <= 'f') return c - 'a' + 10;
      return -1;
    };
    int h = val(hi), l = val(lo);
    if (h < 0 || l < 0) return false;
    dst[i] = (uint8_t)((h << 4) | l);
  }
  return true;
}

// ── Send AT command and wait for "+OK" or error ───────────────
// Returns true if "+OK" received within timeout.
bool atCommand(const char* cmd, unsigned long timeoutMs = 500) {
  LORA_SERIAL.println(cmd);
  DLOG("[AT] >> %s\n", cmd);
  unsigned long start = millis();
  String resp = "";
  while (millis() - start < timeoutMs) {
    while (LORA_SERIAL.available()) {
      char c = LORA_SERIAL.read();
      resp += c;
      if (resp.endsWith("+OK\r\n") || resp.endsWith("+OK\n")) {
        DLOGLN("[AT] << +OK");
        return true;
      }
      if (resp.endsWith("+ERR")) {
        DLOG("[AT] << ERR: %s\n", resp.c_str());
        return false;
      }
    }
  }
  DLOG("[AT] timeout: %s\n", resp.c_str());
  return false;
}

// forward declaration — defined after stats state
void ledFlash(int ms = 80);
unsigned long loraTxBusyUntil = 0;

// ── Queued ACK ring buffer ────────────────────────────────────
// ACKs that arrive while the radio is busy are queued here.
#define ACK_QUEUE_SIZE 4
struct QueuedAck { bool active; MeshPacket pkt; };
QueuedAck ackQueue[ACK_QUEUE_SIZE];
int ackQueueWrite = 0;
int ackQueueRead  = 0;

void enqueueAck(const MeshPacket &pkt) {
  // Check if this msgId is already queued (avoid duplicates in queue)
  for (int i = 0; i < ACK_QUEUE_SIZE; i++)
    if (ackQueue[i].active && ackQueue[i].pkt.msgId == pkt.msgId) return;
  MeshPacket clean = pkt;
  memset(clean.payload, 0, sizeof(clean.payload));
  ackQueue[ackQueueWrite].pkt    = clean;
  ackQueue[ackQueueWrite].active = true;
  ackQueueWrite = (ackQueueWrite + 1) % ACK_QUEUE_SIZE;
  DLOG("[LoRa] ACK 0x%04X queued (slot %d)\n", pkt.msgId,
       (ackQueueWrite - 1 + ACK_QUEUE_SIZE) % ACK_QUEUE_SIZE);
}

// ── Send a MeshPacket over LoRa via AT+SEND ───────────────────
// Returns true if RYLR998 accepted the command (+OK).
bool loraSend(const MeshPacket &pkt, int destRylrAddr = RYLR_BCAST_ADDR) {
  if (millis() < loraTxBusyUntil) {
    // Radio still on-air — queue this packet to send after cooldown
    // (only ACK/LORACK types are queued; DATA retries handled by loraPendingSlots)
    if (pkt.type == PKT_ACK || pkt.type == PKT_LORACK) {
      enqueueAck(pkt);
    } else {
      DLOGLN("[LoRa] loraSend called while busy — skipping");
    }
    return false;
  }

  char hexBuf[PKT_HEX_LEN + 1];
  bytesToHex((const uint8_t*)&pkt, sizeof(MeshPacket), hexBuf);

  char cmd[AT_LINE_BUF];
  snprintf(cmd, sizeof(cmd), "AT+SEND=%d,%d,%s",
           destRylrAddr, (int)PKT_HEX_LEN, hexBuf);

  bool ok = atCommand(cmd, AT_SEND_TIMEOUT_MS);
  if (ok) {
    loraTxBusyUntil = millis() + LORA_TX_COOLDOWN_MS;
    ledFlash(120);
  }
  return ok;
}

// ── Parse a "+RCV=..." line from RYLR998 ─────────────────────
// Format: +RCV=<senderAddr>,<length>,<hexdata>,<rssi>,<snr>
void parseRcvLine(const char* line) {
  // Skip "+RCV="
  const char* p = line + 5;

  // senderAddr
  int senderAddr = atoi(p);
  while (*p && *p != ',') p++; if (*p) p++;

  // length (hex chars)
  int hexLen = atoi(p);
  while (*p && *p != ',') p++; if (*p) p++;

  // hex data
  const char* hexStart = p;
  while (*p && *p != ',') p++;
  int actualHexLen = p - hexStart;
  if (*p) p++;

  // rssi
  int rssi = atoi(p);

  // Validate
  if (actualHexLen != (int)PKT_HEX_LEN || hexLen != (int)PKT_HEX_LEN) {
    DLOG("[LoRa] Bad length: hexLen=%d actual=%d expected=%d\n",
         hexLen, actualHexLen, (int)PKT_HEX_LEN);
    return;
  }

  int slot = loraRxWrite;
  if (!hexToBytes(hexStart, actualHexLen, (uint8_t*)&loraRxBuf[slot].pkt)) {
    DLOGLN("[LoRa] Hex decode failed");
    return;
  }
  loraRxBuf[slot].rssi = rssi;
  loraRxBuf[slot].used = true;
  loraRxWrite = (loraRxWrite + 1) % LORA_RX_BUF;
  DLOG("[LoRa] RX from RYLR addr %d rssi=%d\n", senderAddr, rssi);
}

// ── Called from loop() to drain LORA_SERIAL byte by byte ─────
void tickLoraSerial() {
  while (LORA_SERIAL.available()) {
    char c = LORA_SERIAL.read();
    if (c == '\n') {
      atLineBuf[atLinePos] = '\0';
      // Only care about unsolicited +RCV lines here
      if (strncmp(atLineBuf, "+RCV=", 5) == 0) {
        parseRcvLine(atLineBuf);
      }
      atLinePos = 0;
    } else if (c != '\r') {
      if (atLinePos < AT_LINE_BUF - 1)
        atLineBuf[atLinePos++] = c;
    }
  }
}

// ── Init RYLR998 with AT commands ────────────────────────────
bool loraInit() {
  // Optional hard reset
  if (LORA_RST_PIN >= 0) {
    pinMode(LORA_RST_PIN, OUTPUT);
    digitalWrite(LORA_RST_PIN, LOW);
    delay(100);
    digitalWrite(LORA_RST_PIN, HIGH);
    delay(500); // boot time
  }

  LORA_SERIAL.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  delay(100);

  // Flush any boot message
  while (LORA_SERIAL.available()) LORA_SERIAL.read();

  // Test comms
  if (!atCommand("AT", 1000)) {
    DLOGLN("[LoRa] No response to AT");
    return false;
  }

  // Set address
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%d", RYLR_ADDR);
  if (!atCommand(cmd)) return false;

  // Set network ID
  snprintf(cmd, sizeof(cmd), "AT+NETWORKID=%d", RYLR_NETWORK_ID);
  if (!atCommand(cmd)) return false;

  // Set frequency band (MHz)
  snprintf(cmd, sizeof(cmd), "AT+BAND=%d000000", RYLR_FREQUENCY);
  if (!atCommand(cmd)) return false;

  // Set RF parameters: SF, BW code, CR, preamble
  // BW codes: 7=125kHz, 8=250kHz, 9=500kHz
  int bwCode = (RYLR_BW == 500) ? 9 : (RYLR_BW == 250) ? 8 : 7;
  snprintf(cmd, sizeof(cmd), "AT+PARAMETER=%d,%d,%d,%d",
           RYLR_SF, bwCode, RYLR_CR, RYLR_PREAMBLE);
  if (!atCommand(cmd)) return false;

  // Query firmware version for confirmation
  atCommand("AT+VER?", 500);

  DLOGLN("[LoRa] RYLR998 ready");
  return true;
}

// ============================================================
//  STATE
// ============================================================

uint16_t dedupCache[DEDUP_CACHE];
int      dedupHead = 0;

// Separate cache for ACK/LORACK dedup — prevents ESP-NOW echo loops
// without interfering with DATA dedup
uint16_t ackDedupCache[DEDUP_CACHE];
int      ackDedupHead = 0;

bool isDuplicate(uint16_t id) {
  for (int i = 0; i < DEDUP_CACHE; i++)
    if (dedupCache[i] == id) return true;
  return false;
}
void markSeen(uint16_t id) {
  dedupCache[dedupHead] = id;
  dedupHead = (dedupHead + 1) % DEDUP_CACHE;
}
void unmarkSeen(uint16_t id) {
  for (int i = 0; i < DEDUP_CACHE; i++)
    if (dedupCache[i] == id) { dedupCache[i] = MSG_ID_NONE; return; }
}

bool isAckDuplicate(uint16_t id) {
  for (int i = 0; i < DEDUP_CACHE; i++)
    if (ackDedupCache[i] == id) return true;
  return false;
}
void markAckSeen(uint16_t id) {
  ackDedupCache[ackDedupHead] = id;
  ackDedupHead = (ackDedupHead + 1) % DEDUP_CACHE;
}

uint8_t  msgSeq = 0;
uint16_t makeId(uint8_t src, uint8_t seq) { return ((uint16_t)src << 8) | seq; }

// ── LoRa pending queue ────────────────────────────────────────
// One slot per in-flight message over LoRa backbone.
// With N nodes all sending simultaneously, we need N slots.
// Find a pending slot by msgId, or return nullptr
LoRaPending* findPending(uint16_t msgId) {
  for (int i = 0; i < LORA_PENDING_SLOTS; i++)
    if (loraPendingSlots[i].active && loraPendingSlots[i].pkt.msgId == msgId)
      return &loraPendingSlots[i];
  return nullptr;
}

// Allocate a free pending slot, or evict the oldest if full
LoRaPending* allocPending() {
  for (int i = 0; i < LORA_PENDING_SLOTS; i++)
    if (!loraPendingSlots[i].active) return &loraPendingSlots[i];
  // All slots full — evict oldest (smallest sentAt)
  int oldest = 0;
  for (int i = 1; i < LORA_PENDING_SLOTS; i++)
    if (loraPendingSlots[i].sentAt < loraPendingSlots[oldest].sentAt) oldest = i;
  DLOG("[LoRa] Pending queue full, evicting msgId=0x%04X\n",
       loraPendingSlots[oldest].pkt.msgId);
  return &loraPendingSlots[oldest];
}

// ── ESP-NOW receive ring buffer ───────────────────────────────
#define RX_BUF_SIZE 8
struct RxEntry { MeshPacket pkt; uint8_t mac[6]; int8_t rssi; bool used; };
RxEntry      rxBuf[RX_BUF_SIZE];
volatile int rxWrite = 0;
static   int rxRead  = 0;

static const uint8_t ESPNOW_BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ── Known ESP-NOW peers ───────────────────────────────────────
#define MAX_ESPNOW_PEERS 16
struct KnownPeer { bool valid; uint8_t mac[6]; uint8_t addr; };
KnownPeer knownPeers[MAX_ESPNOW_PEERS];

void registerPeer(const uint8_t* mac, uint8_t meshAddr) {
  for (int i = 0; i < MAX_ESPNOW_PEERS; i++)
    if (knownPeers[i].valid && memcmp(knownPeers[i].mac, mac, 6) == 0) return;
  for (int i = 0; i < MAX_ESPNOW_PEERS; i++) {
    if (!knownPeers[i].valid) {
      memcpy(knownPeers[i].mac, mac, 6);
      knownPeers[i].addr  = meshAddr;
      knownPeers[i].valid = true;
      esp_now_peer_info_t p = {};
      memcpy(p.peer_addr, mac, 6);
      p.channel = 0; p.encrypt = false;
      esp_now_add_peer(&p);
      DLOG("[PEER] Registered @%02X\n", meshAddr);
      return;
    }
  }
}

bool getMac(uint8_t meshAddr, uint8_t* macOut) {
  for (int i = 0; i < MAX_ESPNOW_PEERS; i++)
    if (knownPeers[i].valid && knownPeers[i].addr == meshAddr)
      { memcpy(macOut, knownPeers[i].mac, 6); return true; }
  return false;
}

// ── LED auto-off ──────────────────────────────────────────────
unsigned long ledOffAt = 0;
void ledFlash(int ms) { digitalWrite(LED_PIN, HIGH); ledOffAt = millis() + ms; }

// ============================================================
//  STATS + DISPLAY STATE
// ============================================================

struct BridgeStats {
  uint32_t espRx       = 0;  // ESP-NOW packets received from cluster
  uint32_t espTx       = 0;  // ESP-NOW packets forwarded into cluster
  uint32_t loraRxCount = 0;  // LoRa packets received
  uint32_t loraTxCount = 0;  // LoRa packets transmitted
  uint32_t loraRetries = 0;  // LoRa retry attempts
  uint32_t dupsDropped = 0;  // Deduplicated packets dropped
  int      lastLoraRssi = 0; // RSSI of last LoRa packet received
  int      lastEspRssi  = 0; // RSSI of last ESP-NOW packet received
  uint8_t  lastSrc      = 0; // mesh addr of last ESP-NOW sender
  uint8_t  lastLoraSrc  = 0; // mesh addr of last LoRa sender
  char     lastPayload[MAX_MESSAGE_LENGTH + 1]; // last data payload seen
} stats;

// Display pages cycle on button press or auto every 5 s
// Page 0: Identity + uptime
// Page 1: Traffic counters
// Page 2: Last LoRa packet
// Page 3: Known peers list (scrollable)
#define DISPLAY_PAGES     4
#define DISPLAY_AUTO_MS   5000   // auto-cycle interval

int           displayPage    = 0;
unsigned long lastPageChange = 0;
bool          displayDirty   = true;  // set true whenever stats change

// ── Optional page button (press to cycle pages manually) ─────
// Wire a button between this pin and GND; set to -1 to disable.
#define PAGE_BTN_PIN  0   // GPIO0 = BOOT button on most ESP32 devboards

// peer scroll for page 3
int peerScrollIdx = 0;

// ============================================================
//  DISPLAY DRAW FUNCTIONS
// ============================================================

void drawPage0() {
  // ROW0: "BRIDGE @B1"
  // ROW1: "Up HH:MM:SS"
  // ROW2: "ESP peers: N"
  // ROW3: "Net:18 SF9 868M"
  unsigned long t = millis() / 1000;
  display.clearDisplay();

  display.setCursor(0, ROW0);
  display.printf("BRIDGE @%02X", MY_ADDR);
  display.setCursor(84, ROW0);
  display.printf("p%d/%d", displayPage + 1, DISPLAY_PAGES);

  display.setCursor(0, ROW1);
  display.printf("Up %02d:%02d:%02d",
                 (int)(t/3600)%24, (int)(t/60)%60, (int)(t%60));

  int peerCount = 0;
  for (int i = 0; i < MAX_ESPNOW_PEERS; i++)
    if (knownPeers[i].valid) peerCount++;
  int activePending = 0;
  for (int i = 0; i < LORA_PENDING_SLOTS; i++)
    if (loraPendingSlots[i].active) activePending++;
  display.setCursor(0, ROW2);
  display.printf("Peers:%-2d  LoRa:%s", peerCount,
                 activePending ? "TX" : "idle");

  display.drawFastHLine(0, ROW3 - 1, SCREEN_WIDTH, SSD1306_WHITE);
  display.setCursor(0, ROW3);
  display.printf("Net%-2d SF%d BW%d %dM",
                 RYLR_NETWORK_ID, RYLR_SF, RYLR_BW, RYLR_FREQUENCY);

  display.display();
}

void drawPage1() {
  // Traffic counters
  display.clearDisplay();

  display.setCursor(0, ROW0);
  display.printf("TRAFFIC         p%d/%d", displayPage + 1, DISPLAY_PAGES);

  display.setCursor(0, ROW1);
  display.printf("ESP rx:%-5lu tx:%-5lu", stats.espRx, stats.espTx);

  display.setCursor(0, ROW2);
  display.printf("LRA rx:%-5lu tx:%-5lu", stats.loraRxCount, stats.loraTxCount);

  display.drawFastHLine(0, ROW3 - 1, SCREEN_WIDTH, SSD1306_WHITE);
  display.setCursor(0, ROW3);
  display.printf("dup:%-4lu retry:%-4lu", stats.dupsDropped, stats.loraRetries);

  display.display();
}

void drawPage2() {
  // Last packet info
  display.clearDisplay();

  display.setCursor(0, ROW0);
  display.printf("LAST PKT        p%d/%d", displayPage + 1, DISPLAY_PAGES);

  display.setCursor(0, ROW1);
  display.printf("ESP @%02X  rssi:%4d", stats.lastSrc, stats.lastEspRssi);

  display.setCursor(0, ROW2);
  display.printf("LRA @%02X  rssi:%4d", stats.lastLoraSrc, stats.lastLoraRssi);

  display.drawFastHLine(0, ROW3 - 1, SCREEN_WIDTH, SSD1306_WHITE);
  // Show last payload truncated to 21 chars
  char trunc[22];
  strncpy(trunc, stats.lastPayload, 21);
  trunc[21] = '\0';
  display.setCursor(0, ROW3);
  display.print(trunc[0] ? trunc : "(no data yet)");

  display.display();
}

void drawPage3() {
  // Known peers list — 2 peers per screen, peerScrollIdx to scroll
  display.clearDisplay();

  display.setCursor(0, ROW0);
  int peerCount = 0;
  for (int i = 0; i < MAX_ESPNOW_PEERS; i++)
    if (knownPeers[i].valid) peerCount++;
  display.printf("PEERS %-2d        p%d/%d", peerCount, displayPage + 1, DISPLAY_PAGES);

  // collect valid peers
  int validIdxs[MAX_ESPNOW_PEERS];
  int vn = 0;
  for (int i = 0; i < MAX_ESPNOW_PEERS; i++)
    if (knownPeers[i].valid) validIdxs[vn++] = i;

  if (vn == 0) {
    display.setCursor(0, ROW1); display.print("No peers yet");
  } else {
    // clamp scroll
    if (peerScrollIdx >= vn) peerScrollIdx = 0;
    for (int row = 0; row < 2; row++) {
      int idx = (peerScrollIdx + row) % vn;
      int pi  = validIdxs[idx];
      display.setCursor(0, ROW1 + row * 8);
      display.printf("@%02X %02X:%02X:%02X:%02X:%02X:%02X",
        knownPeers[pi].addr,
        knownPeers[pi].mac[0], knownPeers[pi].mac[1],
        knownPeers[pi].mac[2], knownPeers[pi].mac[3],
        knownPeers[pi].mac[4], knownPeers[pi].mac[5]);
    }
  }

  display.drawFastHLine(0, ROW3 - 1, SCREEN_WIDTH, SSD1306_WHITE);
  display.setCursor(0, ROW3);
  if (vn > 2) display.printf("BTN: next  (%d more)", vn - 2);
  else        display.print("BTN: next page");

  display.display();
}

void drawDisplay() {
  switch (displayPage) {
    case 0: drawPage0(); break;
    case 1: drawPage1(); break;
    case 2: drawPage2(); break;
    case 3: drawPage3(); break;
  }
  displayDirty = false;
}

// ============================================================
//  ESP-NOW SEND HELPERS
// ============================================================

void espnowBroadcast(const MeshPacket &pkt) {
  esp_now_send(ESPNOW_BROADCAST, (const uint8_t*)&pkt, sizeof(pkt));
}

void espnowUnicast(const uint8_t* mac, const MeshPacket &pkt) {
  esp_now_send(mac, (const uint8_t*)&pkt, sizeof(pkt));
}

void espnowForward(MeshPacket pkt) {
  if (pkt.ttl == 0) return;
  pkt.ttl--;
  espnowBroadcast(pkt);
}

void sendLoraAck(const MeshPacket &orig, const uint8_t* senderMac) {
  MeshPacket ack = {};
  ack.type       = PKT_LORACK;
  ack.msgId      = orig.msgId;
  ack.src        = MY_ADDR;
  ack.dst        = orig.src;
  ack.ttl        = 1;
  ack.rssi       = 0;
  ack.payload[0] = '\0';
  espnowUnicast(senderMac, ack);
}

void sendEspnowAck(const MeshPacket &orig, const uint8_t* senderMac) {
  MeshPacket ack = {};
  ack.type       = PKT_ACK;
  ack.msgId      = orig.msgId;
  ack.src        = MY_ADDR;
  ack.dst        = orig.src;
  ack.ttl        = MESH_TTL;
  ack.rssi       = 0;
  ack.payload[0] = '\0';
  espnowUnicast(senderMac, ack);
}

// ── Beacon ───────────────────────────────────────────────────
void sendBeacon() {
  MeshPacket b = {};
  b.type       = PKT_BEACON;
  b.msgId      = makeId(MY_ADDR, msgSeq++);
  b.src        = MY_ADDR;
  b.dst        = BROADCAST_ADDR;
  b.ttl        = 1;
  b.rssi       = 0;
  b.payload[0] = '\0';
  espnowBroadcast(b);
}

// ── LoRa retry tick ───────────────────────────────────────────
void tickLoRaRetry() {
  if (millis() < loraTxBusyUntil) return;
  for (int i = 0; i < LORA_PENDING_SLOTS; i++) {
    LoRaPending &p = loraPendingSlots[i];
    if (!p.active) continue;
    if (millis() - p.sentAt < LORA_RETRY_TIMEOUT_MS) continue;
    if (p.retries < LORA_MAX_RETRIES) {
      p.retries++;
      bool sent = loraSend(p.pkt);
      if (sent) {
        p.sentAt = millis();
        stats.loraRetries++;
        displayDirty = true;
        DLOG("[LoRa] Retry %d msgId=0x%04X\n", p.retries, p.pkt.msgId);
      }
    } else {
      DLOG("[LoRa] Give up msgId=0x%04X — sending fail ACK\n", p.pkt.msgId);
      MeshPacket failAck = {};
      failAck.type  = PKT_ACK;   failAck.msgId = p.pkt.msgId;
      failAck.src   = MY_ADDR;   failAck.dst   = p.pkt.src;
      failAck.ttl   = 1;         failAck.rssi  = -128;
      espnowUnicast(p.origSenderMac, failAck);
      unmarkSeen(p.pkt.msgId);
      p.active = false;
      displayDirty = true;
    }
    break; // one slot per tick
  }
}

// ============================================================
//  PROCESS INCOMING ESP-NOW PACKET
// ============================================================
void processEspNow(const MeshPacket &pkt, const uint8_t* senderMac, int8_t rssi) {
  registerPeer(senderMac, pkt.src);

  if (pkt.type == PKT_BEACON) return;

  // ── ACK / LORACK: deduplicated separately to break ESP-NOW feedback loops ─
  if (pkt.type == PKT_ACK || pkt.type == PKT_LORACK) {
    DLOG("[ESP] ACK/LORACK type=%02X msgId=0x%04X src=@%02X dst=@%02X ttl=%d\n",
                  pkt.type, pkt.msgId, pkt.src, pkt.dst, pkt.ttl);
    // Dedup ACKs separately — prevents ESP-NOW echo loops
    if (isAckDuplicate(pkt.msgId)) {
      DLOG("[ESP] ACK dup dropped 0x%04X\n", pkt.msgId);
      return;
    }
    markAckSeen(pkt.msgId);
    LoRaPending *slot = findPending(pkt.msgId);
    if (slot) { slot->active = false; DLOG("[ESP] ACK resolved 0x%04X\n", pkt.msgId); }
    uint8_t dm[6];
    if (getMac(pkt.dst, dm)) {
      DLOG("[ESP] ACK dst @%02X is LOCAL → unicast\n", pkt.dst);
      MeshPacket clean = pkt; memset(clean.payload, 0, sizeof(clean.payload));
      espnowUnicast(dm, clean);
    } else {
      DLOG("[ESP] ACK dst @%02X not local → loraSend ttl=%d\n", pkt.dst, pkt.ttl);
      if (pkt.ttl > 0) {
        MeshPacket fwd = pkt; memset(fwd.payload, 0, sizeof(fwd.payload)); fwd.ttl--;
        loraSend(fwd);
      } else {
        DLOGLN("[ESP] ACK TTL=0 dropped!");
      }
    }
    return;
  }

  // ── Dedup DATA only ───────────────────────────────────────────
  if (isDuplicate(pkt.msgId)) {
    DLOG("[ESP] Dup 0x%04X\n", pkt.msgId);
    stats.dupsDropped++;
    displayDirty = true;
    return;
  }
  // Note: markSeen() is called AFTER loraSend() succeeds for remote packets,
  // so that a node retry is not dropped if the LoRa send failed.
  // For local/forMe packets we mark immediately below.

  if (pkt.type != PKT_DATA) return;

  stats.espRx++;
  stats.lastSrc     = pkt.src;
  stats.lastEspRssi = rssi;
  if (pkt.payload[0]) strncpy(stats.lastPayload, pkt.payload, MAX_MESSAGE_LENGTH);
  displayDirty = true;

  bool forMe   = (pkt.dst == MY_ADDR);
  bool isBcast = (pkt.dst == BROADCAST_ADDR);
  uint8_t destMac[6];
  bool forLocal = !isBcast && !forMe && getMac(pkt.dst, destMac);

  DLOG("[ESP] DATA src=@%02X dst=@%02X forMe=%d bcast=%d local=%d\n",
                pkt.src, pkt.dst, forMe, isBcast, forLocal);

  if (forMe) {
    markSeen(pkt.msgId);
    sendEspnowAck(pkt, senderMac);
    return;
  }

  if (forLocal) {
    markSeen(pkt.msgId);
    espnowForward(pkt);
    stats.espTx++; displayDirty = true;
    return;
  }

  // ── Remote destination → LoRa backbone ───────────────────────
  MeshPacket loraPkt = pkt;
  loraPkt.ttl = MESH_TTL;

  bool sent = loraSend(loraPkt);
  displayDirty = true;

  if (sent) {
    // Only mark seen after confirmed LoRa handoff —
    // if loraSend fails, the node retry must not be dropped as duplicate
    markSeen(pkt.msgId);
    stats.loraTxCount++;
    ledFlash(150);
    if (!isBcast) sendLoraAck(pkt, senderMac);
    if (!isBcast) {
      LoRaPending *slot = allocPending();
      slot->active  = true;
      slot->pkt     = loraPkt;
      memcpy(slot->origSenderMac, senderMac, 6);
      slot->sentAt  = millis();
      slot->retries = 0;
    }
  } else {
    DLOGLN("[ESP] loraSend failed — node may retry");
    // Do NOT markSeen — let the node's next retry come through fresh
  }
}

// ============================================================
//  PROCESS INCOMING LoRa PACKET
// ============================================================
void processLoRa(const MeshPacket &pkt, int rssi) {
  DLOG("[LoRa] type=%02X msgId=0x%04X src=@%02X dst=@%02X ttl=%d rssi=%d\n",
                pkt.type, pkt.msgId, pkt.src, pkt.dst, pkt.ttl, rssi);

  // ── ACK / LORACK: deduplicated to prevent ESP-NOW flood ──────
  if (pkt.type == PKT_ACK || pkt.type == PKT_LORACK) {
    DLOG("[LoRa] ACK/LORACK type=%02X msgId=0x%04X src=@%02X dst=@%02X ttl=%d\n",
                  pkt.type, pkt.msgId, pkt.src, pkt.dst, pkt.ttl);
    if (isAckDuplicate(pkt.msgId)) {
      DLOG("[LoRa] ACK dup dropped 0x%04X\n", pkt.msgId);
      return;
    }
    markAckSeen(pkt.msgId);
    // ACKs are routing traffic, not data — count them separately via loraTx/espTx
    LoRaPending *slot = findPending(pkt.msgId);
    if (slot) slot->active = false;
    uint8_t dm[6];
    if (getMac(pkt.dst, dm)) {
      DLOG("[LoRa] ACK dst @%02X is LOCAL → unicast\n", pkt.dst);
      MeshPacket clean = pkt;
      memset(clean.payload, 0, sizeof(clean.payload));
      espnowUnicast(dm, clean);
    } else {
      DLOG("[LoRa] ACK dst @%02X not local → loraSend ttl=%d\n", pkt.dst, pkt.ttl);
      if (pkt.ttl > 0) {
        MeshPacket fwd = pkt;
        memset(fwd.payload, 0, sizeof(fwd.payload));
        fwd.ttl--;
        loraSend(fwd);
      } else {
        DLOGLN("[LoRa] ACK TTL=0 dropped!");
      }
    }
    return;
  }

  // ── Dedup DATA only ───────────────────────────────────────────
  if (isDuplicate(pkt.msgId)) {
    DLOG("[LoRa] Dup 0x%04X\n", pkt.msgId);
    stats.dupsDropped++;
    displayDirty = true;
    return;
  }
  markSeen(pkt.msgId);

  if (pkt.type != PKT_DATA) return;

  stats.loraRxCount++;
  stats.lastLoraSrc  = pkt.src;
  stats.lastLoraRssi = rssi;
  if (pkt.payload[0]) strncpy(stats.lastPayload, pkt.payload, MAX_MESSAGE_LENGTH);
  displayDirty = true;

  bool forMe   = (pkt.dst == MY_ADDR);
  bool isBcast = (pkt.dst == BROADCAST_ADDR);
  uint8_t destMac[6];
  bool forLocal = !isBcast && !forMe && getMac(pkt.dst, destMac);

  if (forMe) {
    MeshPacket ack = {};
    ack.type = PKT_ACK; ack.msgId = pkt.msgId;
    ack.src  = MY_ADDR; ack.dst   = pkt.src;
    ack.ttl  = MESH_TTL; ack.rssi = 0;
    loraSend(ack);
    return;
  }

  if (forLocal) {
    MeshPacket local = pkt;
    local.ttl = MESH_TTL;
    espnowBroadcast(local);
    stats.espTx++;
    MeshPacket ack = {};
    ack.type = PKT_ACK; ack.msgId = pkt.msgId;
    ack.src  = MY_ADDR; ack.dst   = pkt.src;
    ack.ttl  = MESH_TTL; ack.rssi = 0;
    loraSend(ack);
    ledFlash(80);
    displayDirty = true;
    return;
  }

  // Unknown destination — flood LoRa (TTL) and local cluster
  if (pkt.ttl > 0) {
    MeshPacket fwd = pkt; fwd.ttl--;
    if (loraSend(fwd)) { stats.loraTxCount++; displayDirty = true; }
  }
  MeshPacket local = pkt; local.ttl = MESH_TTL;
  espnowBroadcast(local);
  stats.espTx++;
  displayDirty = true;
}

// ============================================================
//  ESP-NOW RECEIVE CALLBACK
// ============================================================
void onEspNowReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(MeshPacket)) return;
  int slot = rxWrite;
  rxBuf[slot].used = false;
  memcpy(&rxBuf[slot].pkt, data, sizeof(MeshPacket));
  memcpy(rxBuf[slot].mac,  info->src_addr, 6);
  rxBuf[slot].rssi = (int8_t)info->rx_ctrl->rssi;
  rxBuf[slot].used = true;
  rxWrite = (rxWrite + 1) % RX_BUF_SIZE;
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  DLOGLN("\n[BRIDGE] Booting RYLR998 bridge...");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  if (PAGE_BTN_PIN >= 0) pinMode(PAGE_BTN_PIN, INPUT_PULLUP);

  // Display
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    DLOGLN("[BRIDGE] OLED not found — continuing without display");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, ROW0); display.print("BRIDGE BOOTING...");
    display.setCursor(0, ROW1); display.printf("@%02X  RYLR998", MY_ADDR);
    display.display();
  }

  memset(&stats, 0, sizeof(stats));

  for (int i = 0; i < DEDUP_CACHE; i++) dedupCache[i] = MSG_ID_NONE;
  for (int i = 0; i < DEDUP_CACHE; i++) ackDedupCache[i] = MSG_ID_NONE;
  memset(loraPendingSlots, 0, sizeof(loraPendingSlots));
  memset(ackQueue, 0, sizeof(ackQueue));
  memset(knownPeers, 0, sizeof(knownPeers));
  memset(rxBuf,      0, sizeof(rxBuf));
  memset(loraRxBuf,  0, sizeof(loraRxBuf));

  // ── WiFi / ESP-NOW ────────────────────────────────────────────
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(80);
  WiFi.disconnect();
  esp_wifi_set_protocol(WIFI_IF_STA,
    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
    WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[BRIDGE] ESP-NOW FAILED"); while (true);
  }
  esp_now_peer_info_t bp = {};
  memcpy(bp.peer_addr, ESPNOW_BROADCAST, 6);
  bp.channel = 0; bp.encrypt = false;
  esp_now_add_peer(&bp);
  esp_now_register_recv_cb(onEspNowReceive);
  #ifdef DEBUG
  Serial.print("[BRIDGE] MAC: "); Serial.println(WiFi.macAddress());
  DLOG("[BRIDGE] Mesh addr: @%02X\n", MY_ADDR);
  #endif

  // ── RYLR998 ───────────────────────────────────────────────────
  if (!loraInit()) {
    Serial.println("[BRIDGE] RYLR998 init FAILED"); while (true);
  }

  DLOGLN("[BRIDGE] Ready.");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // LED auto-off
  if (ledOffAt > 0 && now > ledOffAt) { digitalWrite(LED_PIN, LOW); ledOffAt = 0; }

  // ── Drain queued ACKs after radio cooldown ────────────────────
  while (ackQueueRead != ackQueueWrite && millis() >= loraTxBusyUntil) {
    if (ackQueue[ackQueueRead].active) {
      DLOG("[LoRa] Sending queued ACK 0x%04X\n",
                    ackQueue[ackQueueRead].pkt.msgId);
      loraSend(ackQueue[ackQueueRead].pkt);
      ackQueue[ackQueueRead].active = false;
    }
    ackQueueRead = (ackQueueRead + 1) % ACK_QUEUE_SIZE;
  }

  // ── Page button (BOOT button / GPIO0) ─────────────────────────
  if (PAGE_BTN_PIN >= 0) {
    static bool lastBtn = HIGH;
    bool btn = digitalRead(PAGE_BTN_PIN);
    if (btn == LOW && lastBtn == HIGH) {           // falling edge
      // On page 3: scroll peers; otherwise advance page
      if (displayPage == 3) {
        int vn = 0;
        for (int i = 0; i < MAX_ESPNOW_PEERS; i++)
          if (knownPeers[i].valid) vn++;
        if (vn > 0) peerScrollIdx = (peerScrollIdx + 2) % vn;
        else { displayPage = (displayPage + 1) % DISPLAY_PAGES; peerScrollIdx = 0; }
      } else {
        displayPage = (displayPage + 1) % DISPLAY_PAGES;
      }
      lastPageChange = now;
      displayDirty   = true;
    }
    lastBtn = btn;
  }

  // ── Auto-cycle pages ──────────────────────────────────────────
  if (now - lastPageChange >= DISPLAY_AUTO_MS) {
    displayPage    = (displayPage + 1) % DISPLAY_PAGES;
    lastPageChange = now;
    displayDirty   = true;
    if (displayPage == 3) peerScrollIdx = 0; // reset peer scroll on page entry
  }

  // ── Redraw display if anything changed ────────────────────────
  if (displayDirty) drawDisplay();

  // Drain RYLR998 serial → parse +RCV lines into loraRxBuf
  tickLoraSerial();

  // Beacon — always send on schedule; it's ESP-NOW only, never blocks
  static unsigned long lastBeacon = 0;
  if (now - lastBeacon >= BEACON_INTERVAL_MS) {
    lastBeacon = now;
    sendBeacon();
  }

  // LoRa retry
  tickLoRaRetry();

  // Process ESP-NOW packets
  while (rxRead != rxWrite) {
    if (rxBuf[rxRead].used) {
      MeshPacket pkt = rxBuf[rxRead].pkt;
      uint8_t mac[6]; memcpy(mac, rxBuf[rxRead].mac, 6);
      int8_t rssi    = rxBuf[rxRead].rssi;
      rxBuf[rxRead].used = false;
      processEspNow(pkt, mac, rssi);
    }
    rxRead = (rxRead + 1) % RX_BUF_SIZE;
  }

  // Process LoRa packets
  while (loraRxRead != loraRxWrite) {
    if (loraRxBuf[loraRxRead].used) {
      MeshPacket pkt = loraRxBuf[loraRxRead].pkt;
      int rssi       = loraRxBuf[loraRxRead].rssi;
      loraRxBuf[loraRxRead].used = false;
      processLoRa(pkt, rssi);
    }
    loraRxRead = (loraRxRead + 1) % LORA_RX_BUF;
  }
#ifdef DEBUG
  // Serial debug
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "peers") {
      for (int i = 0; i < MAX_ESPNOW_PEERS; i++) {
        if (!knownPeers[i].valid) continue;
        Serial.printf("  @%02X  %02X:%02X:%02X:%02X:%02X:%02X\n",
          knownPeers[i].addr,
          knownPeers[i].mac[0], knownPeers[i].mac[1], knownPeers[i].mac[2],
          knownPeers[i].mac[3], knownPeers[i].mac[4], knownPeers[i].mac[5]);
      }
    } else if (cmd == "status") {
      int active = 0;
      for (int i = 0; i < LORA_PENDING_SLOTS; i++) {
        if (!loraPendingSlots[i].active) continue;
        active++;
        Serial.printf("  slot%d msgId=0x%04X retries=%d\n", i,
                      loraPendingSlots[i].pkt.msgId, loraPendingSlots[i].retries);
      }
      if (!active) Serial.println("LoRa pending: none");
    } else if (cmd == "beacon") {
      sendBeacon(); Serial.println("Beacon sent");
    } else if (cmd == "at") {
      // Pass raw AT command: type "at AT+VER?" in serial monitor
      Serial.println("Type AT command directly:");
    } else if (cmd.startsWith("AT") || cmd.startsWith("at+") || cmd.startsWith("AT+")) {
      // Forward raw AT command to RYLR998
      atCommand(cmd.c_str(), 1000);
    }
  }
#endif
}
