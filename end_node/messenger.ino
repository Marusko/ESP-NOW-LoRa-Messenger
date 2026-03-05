// ============================================================
//  ESP-NOW Mesh Messenger  –  End Node
//  Display : SSD1306 128x32 OLED
//  Network : ESP-NOW cluster + LoRa bridge beacon scan
// ============================================================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ============================================================
//  SECTION 1 – CONSTANTS  (must come first so structs can use them)
// ============================================================

#define MY_ADDR        0x01
#define BROADCAST_ADDR 0xFF
#define MESH_TTL       3
#define MAX_RETRIES    5    // with loraAcked: 5×3s=15s > bridge give-up at ~6s
#define DEDUP_CACHE    24
#define MSG_ID_NONE    0xFFFF
#define MAX_MESSAGE_LENGTH 30
#define MAX_MESSAGES   6
#define MAX_BRIDGES    4
#define RX_BUF_SIZE    4

// Two-tier ACK timeouts
#define ACK_TIMEOUT_SHORT_MS  800    // waiting for ESP-NOW ACK or LORACK
#define ACK_TIMEOUT_LONG_MS   3000   // waiting for real ACK after LORACK

// Packet type bytes
#define PKT_DATA    0x01
#define PKT_ACK     0x02
#define PKT_BEACON  0x03
#define PKT_LORACK  0x04  // Bridge → node: "msg accepted into LoRa backbone"

// ── UI layout (6×8 px font → 4 rows × 21 cols on 128×32 screen) ──
#define ROW0   0
#define ROW1   8
#define ROW2  16
#define ROW3  24
#define COLS  21

// ── Pin assignments ──────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  32
#define OLED_RESET     -1
#define LED_PIN         2

#define BTN_UP      12
#define BTN_DOWN    14
#define BTN_LEFT    27
#define BTN_RIGHT   26
#define BTN_CENTER  25
#define BTN_SET     33
#define BTN_RST     32

// ── Button timing ────────────────────────────────────────────
const unsigned long debounceDelay   =  30;
const unsigned long longPressTime   = 600;
const unsigned long repeatStartTime = 600;
const unsigned long repeatInterval  = 150;

// ============================================================
//  SECTION 2 – TYPES / STRUCTS
// ============================================================

// ── Shared over-the-air packet (identical to bridge firmware) ─
struct MeshPacket {
  uint8_t  type;
  uint16_t msgId;
  uint8_t  src;
  uint8_t  dst;
  uint8_t  ttl;
  int8_t   rssi;
  char     payload[MAX_MESSAGE_LENGTH + 1];
};

// ── ACK display state ────────────────────────────────────────
enum AckStatus { ACK_NONE, ACK_WAIT, ACK_OK, ACK_FAIL };

// ── Pending outbound message (ACK / retry tracking) ──────────
struct PendingMsg {
  bool       active;
  MeshPacket pkt;
  unsigned long sentAt;
  int        retries;
  bool       loraAcked;  // true after LORACK → switch to long timeout
};

// ── Button state ─────────────────────────────────────────────
enum ButtonEvent { NONE, PRESS, LONG_PRESS, REPEAT, RELEASE };
struct Button {
  int  pin;
  bool stableState, lastReading;
  unsigned long lastDebounceTime, pressStartTime, lastRepeatTime;
  bool longPressTriggered;
};

// ── Bridge beacon entry ──────────────────────────────────────
struct BridgeNode {
  bool    valid;
  uint8_t mac[6];
  int8_t  rssi;
  unsigned long lastSeen;
};

// ── ESP-NOW receive ring-buffer entry ────────────────────────
struct RxEntry {
  MeshPacket pkt;
  uint8_t    mac[6];
  int8_t     rssi;
  bool       used;
};

// ── Menu identifiers ─────────────────────────────────────────
enum Menu { MENU_EDITOR, MENU_HISTORY, MENU_IDLE, MENU_SETTINGS, MENU_DETAIL };

// ============================================================
//  SECTION 3 – GLOBAL STATE
// ============================================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── Buttons ──────────────────────────────────────────────────
const int buttonPins[] = { BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT,
                            BTN_CENTER, BTN_SET, BTN_RST };
const int buttonCount  = 7;
Button    buttons[buttonCount];
bool enableLongPress[buttonCount] = { false,false,false,false,true,true,true };
bool enableRepeat[buttonCount]    = { true, true, true, true,false,false,true };

// ── LED ──────────────────────────────────────────────────────
unsigned long ledFlashUntil = 0;

// ── Networking ───────────────────────────────────────────────
static const uint8_t ESPNOW_BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint16_t dedupCache[DEDUP_CACHE];
int      dedupHead = 0;

uint16_t ackDedupCache[DEDUP_CACHE];
int      ackDedupHead = 0;

uint8_t  msgSeq = 0;

PendingMsg pending = { false, {}, 0, 0, false };

BridgeNode bridges[MAX_BRIDGES];
int        bestBridgeIdx = -1;

// ── ESP-NOW receive ring buffer (written from WiFi-task callback) ─
RxEntry      rxBuf[RX_BUF_SIZE];
volatile int rxWrite = 0;
static   int rxRead  = 0;

// ── ACK display ──────────────────────────────────────────────
AckStatus     lastAckStatus   = ACK_NONE;
unsigned long ackStatusExpiry = 0;

// ── Message history ──────────────────────────────────────────
String    messages[MAX_MESSAGES];
AckStatus msgAck[MAX_MESSAGES];
uint16_t  msgIds[MAX_MESSAGES];   // msgId of each stored message (MSG_ID_NONE for RX)
int  messageCount       = 0;
int  scrollIndex        = 0;
int  detailScrollOffset = 0;
int  unreadCount        = 0;

// ── Text editor ──────────────────────────────────────────────
uint8_t destAddr   = 0x01;
String  destination = "@01";
int     settingsCursor = 0;

char messageBody[MAX_MESSAGE_LENGTH + 1];
int  cursorPos   = 0;
int  scrollOffset = 0;

const char charSet[]  = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789,.?!";
const int  charSetSize = sizeof(charSet) - 1;

// ── Menu ─────────────────────────────────────────────────────
Menu          currentMenu  = MENU_IDLE;
unsigned long lastActivity = 0;
const unsigned long idleTimeout = 10000;

// ============================================================
//  SECTION 4 – FORWARD DECLARATIONS
//  (needed because network functions call UI and vice-versa)
// ============================================================
void redrawCurrent();
void drawEditor();
void drawHistory();

// ============================================================
//  SECTION 5 – UTILITY / DEDUP
// ============================================================

uint16_t makeId(uint8_t src, uint8_t seq) {
  return ((uint16_t)src << 8) | seq;
}

bool isDuplicate(uint16_t id) {
  for (int i = 0; i < DEDUP_CACHE; i++)
    if (dedupCache[i] == id) return true;
  return false;
}

void markSeen(uint16_t id) {
  dedupCache[dedupHead] = id;
  dedupHead = (dedupHead + 1) % DEDUP_CACHE;
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

// ============================================================
//  SECTION 6 – MESSAGE HISTORY HELPERS
// ============================================================

void addMessage(const String &msg, AckStatus ack = ACK_NONE,
                uint16_t id = MSG_ID_NONE, bool isTx = false) {
  for (int i = MAX_MESSAGES - 1; i > 0; i--) {
    messages[i] = messages[i - 1];
    msgAck[i]   = msgAck[i - 1];
    msgIds[i]   = msgIds[i - 1];
  }
  messages[0] = msg;
  msgAck[0]   = ack;
  msgIds[0]   = id;
  if (messageCount < MAX_MESSAGES) messageCount++;
  // Only flash LED and count unread for incoming messages, not our own TX
  if (!isTx) {
    unreadCount++;
    digitalWrite(LED_PIN, HIGH);
    ledFlashUntil = millis() + 300;
  }
}

// Find the history slot for a given msgId and update its ACK status
void updateMsgAck(uint16_t id, AckStatus ack) {
  for (int i = 0; i < messageCount; i++) {
    if (msgIds[i] == id) {
      msgAck[i] = ack;
      return;
    }
  }
}

// ============================================================
//  SECTION 7 – DESTINATION HELPER
// ============================================================

void updateDestination() {
  char buf[4];
  snprintf(buf, sizeof(buf), "@%02X", destAddr);
  destination = String(buf);
}

// ============================================================
//  SECTION 8 – ESP-NOW SEND HELPERS
// ============================================================

void espnowSend(const MeshPacket &pkt) {
  esp_now_send(ESPNOW_BROADCAST, (const uint8_t*)&pkt, sizeof(pkt));
}

void meshForward(MeshPacket pkt) {
  if (pkt.ttl == 0) return;
  pkt.ttl--;
  espnowSend(pkt);
}

void sendAck(const MeshPacket &orig) {
  MeshPacket ack;
  ack.type       = PKT_ACK;
  ack.msgId      = orig.msgId;
  ack.src        = MY_ADDR;
  ack.dst        = orig.src;
  ack.ttl        = MESH_TTL;
  ack.rssi       = 0;
  ack.payload[0] = '\0';
  espnowSend(ack);
}

uint16_t meshSendDirect(uint8_t dst, const char* text) {
  MeshPacket pkt;
  pkt.type  = PKT_DATA;
  pkt.msgId = makeId(MY_ADDR, msgSeq++);
  pkt.src   = MY_ADDR;
  pkt.dst   = dst;
  pkt.ttl   = MESH_TTL;
  pkt.rssi  = 0;
  strncpy(pkt.payload, text, MAX_MESSAGE_LENGTH);
  pkt.payload[MAX_MESSAGE_LENGTH] = '\0';

  markSeen(pkt.msgId);
  espnowSend(pkt);

  pending.active    = true;
  pending.pkt       = pkt;
  pending.sentAt    = millis();
  pending.retries   = 0;
  pending.loraAcked = false;

  lastAckStatus   = ACK_WAIT;
  ackStatusExpiry = 0;
  return pkt.msgId;
}

void meshSendBroadcast(const char* text) {
  MeshPacket pkt;
  pkt.type  = PKT_DATA;
  pkt.msgId = makeId(MY_ADDR, msgSeq++);
  pkt.src   = MY_ADDR;
  pkt.dst   = BROADCAST_ADDR;
  pkt.ttl   = MESH_TTL;
  pkt.rssi  = 0;
  strncpy(pkt.payload, text, MAX_MESSAGE_LENGTH);
  pkt.payload[MAX_MESSAGE_LENGTH] = '\0';

  markSeen(pkt.msgId);
  espnowSend(pkt);

  char label[48];
  snprintf(label, sizeof(label), "BC @FF %s", text);
  addMessage(String(label), ACK_NONE, pkt.msgId, true);
}

// ============================================================
//  SECTION 9 – BRIDGE BEACON TRACKING
// ============================================================

void updateBestBridge() {
  bestBridgeIdx = -1;
  int8_t best = -127;
  for (int i = 0; i < MAX_BRIDGES; i++) {
    if (bridges[i].valid && bridges[i].rssi > best) {
      best = bridges[i].rssi;
      bestBridgeIdx = i;
    }
  }
}

void registerBridge(const uint8_t* mac, int8_t rssi) {
  unsigned long now = millis();
  for (int i = 0; i < MAX_BRIDGES; i++) {
    if (bridges[i].valid && memcmp(bridges[i].mac, mac, 6) == 0) {
      bridges[i].rssi     = rssi;
      bridges[i].lastSeen = now;
      updateBestBridge();
      return;
    }
  }
  for (int i = 0; i < MAX_BRIDGES; i++) {
    if (!bridges[i].valid) {
      memcpy(bridges[i].mac, mac, 6);
      bridges[i].rssi     = rssi;
      bridges[i].lastSeen = now;
      bridges[i].valid    = true;
      updateBestBridge();
      return;
    }
  }
  // Evict weakest
  int w = 0;
  for (int i = 1; i < MAX_BRIDGES; i++)
    if (bridges[i].rssi < bridges[w].rssi) w = i;
  memcpy(bridges[w].mac, mac, 6);
  bridges[w].rssi     = rssi;
  bridges[w].lastSeen = now;
  updateBestBridge();
}

void pruneBridges() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_BRIDGES; i++) {
    // 60 s — generous enough to survive bridge busy with LoRa retries
    if (bridges[i].valid && now - bridges[i].lastSeen > 60000) {
      bridges[i].valid = false;
    }
  }
  updateBestBridge();
}

// ============================================================
//  SECTION 10 – ESP-NOW RECEIVE CALLBACK + PACKET PROCESSING
// ============================================================

// Callback runs in WiFi task — only touch the ring buffer here.
void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(MeshPacket)) return;
  int slot = rxWrite;
  rxBuf[slot].used = false;
  memcpy(&rxBuf[slot].pkt, data, sizeof(MeshPacket));
  memcpy(rxBuf[slot].mac, info->src_addr, 6);
  rxBuf[slot].rssi = (int8_t)info->rx_ctrl->rssi;
  rxBuf[slot].used = true;
  rxWrite = (rxWrite + 1) % RX_BUF_SIZE;
}

void processPacket(const MeshPacket &pkt, const uint8_t* senderMac, int8_t rssi) {

  // ── Beacon from a bridge node ──────────────────────────────
  if (pkt.type == PKT_BEACON) {
    registerBridge(senderMac, rssi);
    return;
  }

  // ── ACK / LORACK: never deduplicated — always process ──────
  // (They share the msgId of the original data packet, which is
  //  already in the dedup cache, so dedup would drop them.)

  if (pkt.type == PKT_LORACK) {
    if (pending.active && pkt.msgId == pending.pkt.msgId && !pending.loraAcked) {
      pending.loraAcked = true;
      pending.sentAt    = millis();
      lastAckStatus     = ACK_WAIT;
      if (currentMenu == MENU_EDITOR || currentMenu == MENU_HISTORY) redrawCurrent();
    }
    return;
  }

  if (pkt.type == PKT_ACK) {
    if (isAckDuplicate(pkt.msgId)) return;
    markAckSeen(pkt.msgId);
    if (pending.active && pkt.msgId == pending.pkt.msgId) {
      pending.active  = false;
      // rssi == -128 is a sentinel sent by the bridge meaning "LoRa delivery failed"
      bool deliveryFail = (pkt.rssi == -128);
      lastAckStatus   = deliveryFail ? ACK_FAIL : ACK_OK;
      ackStatusExpiry = millis() + (deliveryFail ? 5000 : 3000);
      updateMsgAck(pkt.msgId, lastAckStatus);
      if (currentMenu == MENU_EDITOR || currentMenu == MENU_HISTORY) redrawCurrent();
    }
    meshForward(pkt);
    return;
  }

  // ── Deduplication — DATA packets only ──────────────────────
  if (isDuplicate(pkt.msgId)) return;
  markSeen(pkt.msgId);
  if (pkt.type == PKT_DATA) {
    bool forMe = (pkt.dst == MY_ADDR);
    bool bcast = (pkt.dst == BROADCAST_ADDR);

    if (forMe) {
      char label[48];
      // "RX @XX body" — @XX is who sent it to us
      snprintf(label, sizeof(label), "RX @%02X %s", pkt.src, pkt.payload);
      addMessage(String(label), ACK_NONE, pkt.msgId, false);
      sendAck(pkt);
      if (currentMenu == MENU_EDITOR || currentMenu == MENU_IDLE) redrawCurrent();
    } else if (bcast) {
      char label[48];
      // "BC @XX body" — @XX is the broadcaster
      snprintf(label, sizeof(label), "BC @%02X %s", pkt.src, pkt.payload);
      addMessage(String(label), ACK_NONE, pkt.msgId, false);
      if (currentMenu == MENU_EDITOR || currentMenu == MENU_IDLE) redrawCurrent();
    }

    meshForward(pkt);
  }
}

// ── ACK timeout / retry ──────────────────────────────────────
void tickAck() {
  if (!pending.active) return;
  unsigned long now     = millis();
  unsigned long timeout = pending.loraAcked ? ACK_TIMEOUT_LONG_MS
                                            : ACK_TIMEOUT_SHORT_MS;
  if (now - pending.sentAt < timeout) return;

  if (pending.retries < MAX_RETRIES) {
    pending.retries++;
    pending.sentAt = now;
    // Keep loraAcked flag — if we ever got a LORACK, all retries
    // use the long timeout. Resetting to false caused premature [!].
    espnowSend(pending.pkt);
  } else {
    pending.active  = false;
    lastAckStatus   = ACK_FAIL;
    ackStatusExpiry = millis() + 4000;
    updateMsgAck(pending.pkt.msgId, ACK_FAIL);
    if (currentMenu == MENU_EDITOR || currentMenu == MENU_HISTORY) redrawCurrent();
  }
}

// ============================================================
//  SECTION 11 – BUTTON UPDATE
// ============================================================

ButtonEvent updateButton(int i) {
  Button &b = buttons[i];
  bool reading = digitalRead(b.pin);
  unsigned long now = millis();
  if (reading != b.lastReading) { b.lastDebounceTime = now; b.lastReading = reading; }
  if (now - b.lastDebounceTime > debounceDelay) {
    if (reading != b.stableState) {
      b.stableState = reading;
      if (reading == LOW) {
        b.pressStartTime = now; b.lastRepeatTime = now;
        b.longPressTriggered = false;
        return PRESS;
      } else {
        b.longPressTriggered = false; b.lastRepeatTime = 0;
        return RELEASE;
      }
    }
    if (b.stableState == LOW) {
      unsigned long held = now - b.pressStartTime;
      if (enableLongPress[i] && !b.longPressTriggered && held >= longPressTime)
        { b.longPressTriggered = true; return LONG_PRESS; }
      if (enableRepeat[i] && held >= repeatStartTime
          && now - b.lastRepeatTime >= repeatInterval)
        { b.lastRepeatTime = now; return REPEAT; }
    }
  }
  return NONE;
}

// ============================================================
//  SECTION 12 – SELF-COMMAND HANDLER
// ============================================================

// Returns a response string for a known command, or "" if unknown.
// Add new commands here. All matching is case-insensitive.
String handleSelfCommand(const char* cmd) {
  // Normalise to uppercase for comparison
  String s = String(cmd);
  s.trim();
  s.toUpperCase();

  if (s == "BATT") {
    // Placeholder — replace analogRead with your actual ADC pin / divider
    // e.g. float v = analogRead(34) * (3.3f / 4095.0f) * 2.0f;
    return "BATT: -- (no ADC)";
  }
  if (s == "RSSI") {
    if (bestBridgeIdx >= 0)
      return "GW RSSI: " + String(bridges[bestBridgeIdx].rssi) + " dBm";
    return "GW RSSI: no bridge";
  }
  if (s == "ADDR") {
    char buf[16];
    snprintf(buf, sizeof(buf), "My addr: @%02X", MY_ADDR);
    return String(buf);
  }
  if (s == "MSGS") {
    return "Msgs: " + String(messageCount) + "/" + String(MAX_MESSAGES);
  }
  if (s == "PING") {
    return "PONG";
  }
  if (s == "HELP") {
    return "BATT RSSI ADDR MSGS PING";
  }

  return "Unknown: " + String(cmd);
}

// ============================================================
//  SECTION 13 – TEXT EDITOR SEND
// ============================================================

void sendCurrentText() {
  if (messageBody[0] == '\0') return;
  int len = strlen(messageBody);
  while (len > 0 && messageBody[len - 1] == ' ') messageBody[--len] = '\0';
  if (len == 0) return;

  // ── Self-command: destination is this node ──────────────────
  // Addressed to MY_ADDR → execute locally, never sent over air
  if (destAddr == MY_ADDR) {
    String result = handleSelfCommand(messageBody);
    char label[48];
    snprintf(label, sizeof(label), "CMD @%02X %s", MY_ADDR, messageBody);
    addMessage(String(label), ACK_NONE, MSG_ID_NONE, true);  // echo command
    addMessage("    " + result, ACK_NONE, MSG_ID_NONE, false); // show result
    messageBody[0] = '\0';
    cursorPos    = 0;
    scrollOffset = 0;
    if (currentMenu == MENU_EDITOR) drawEditor();
    return;
  }

  bool isBroadcast = (destAddr == BROADCAST_ADDR);

  if (isBroadcast) {
    meshSendBroadcast(messageBody);
  } else {
    char label[48];
    snprintf(label, sizeof(label), "TX @%02X %s", destAddr, messageBody);
    uint16_t id = meshSendDirect(destAddr, messageBody);
    addMessage(String(label), ACK_WAIT, id, true);
  }

  messageBody[0] = '\0';
  cursorPos    = 0;
  scrollOffset = 0;
}

// ============================================================
//  SECTION 13 – UI HELPERS
// ============================================================

void drawHighlightChar(int x, int y, char c) {
  display.fillRect(x, y, 6, 8, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(x, y);
  display.print(c);
  display.setTextColor(SSD1306_WHITE);
}

void drawHintBar(const char* left, const char* mid, const char* right) {
  char buf[22];
  snprintf(buf, sizeof(buf), "%-7s%-7s%-7s", left, mid, right);
  buf[21] = '\0';
  display.setCursor(0, ROW3);
  display.print(buf);
}

void drawAckIndicator() {
  if (lastAckStatus == ACK_WAIT) {
    const char* s = (pending.active && pending.loraAcked) ? "LR." : "...";
    display.setCursor(110, ROW0); display.print(s);
  } else if (lastAckStatus == ACK_OK) {
    display.setCursor(110, ROW0); display.print("[+]");
  } else if (lastAckStatus == ACK_FAIL) {
    display.setCursor(110, ROW0); display.print("[!]");
  }
}

// ============================================================
//  SECTION 14 – DRAW FUNCTIONS
// ============================================================

void drawEditor() {
  display.clearDisplay();

  int destCols    = destination.length() + 1;   // "@XX:" = 4
  int line1chars  = COLS - destCols;             // 17
  int line2chars  = COLS;                        // 21
  int visibleChars = line1chars + line2chars;

  if (cursorPos < scrollOffset) scrollOffset = cursorPos;
  if (cursorPos >= scrollOffset + visibleChars)
    scrollOffset = cursorPos - visibleChars + 1;

  int bodyLen = strlen(messageBody);

  display.setCursor(0, ROW0);
  display.print(destination); display.print(':');
  for (int i = scrollOffset; i < scrollOffset + line1chars; i++)
    display.print(i < bodyLen ? (char)messageBody[i] : ' ');

  display.setCursor(0, ROW1);
  for (int i = scrollOffset + line1chars;
       i < scrollOffset + line1chars + line2chars; i++)
    display.print(i < bodyLen ? (char)messageBody[i] : ' ');

  // Cursor highlight
  int visPos = cursorPos - scrollOffset;
  int cRow, cCol;
  if (visPos < line1chars) { cRow = 0; cCol = destCols + visPos; }
  else                     { cRow = 1; cCol = visPos - line1chars; }
  drawHighlightChar(cCol * 6, cRow * 8,
                    (cursorPos < bodyLen) ? messageBody[cursorPos] : '_');

  // ROW2: length + broadcast label
  display.setCursor(0, ROW2);
  display.printf("%d/%d", bodyLen, MAX_MESSAGE_LENGTH);
  if (destAddr == BROADCAST_ADDR) { display.setCursor(42, ROW2); display.print("[BCAST]"); }
  drawAckIndicator();

  display.drawFastHLine(0, ROW3 - 1, SCREEN_WIDTH, SSD1306_WHITE);
  drawHintBar("<>:move", "^v:char", "C:send");
  display.display();
}

void drawHistory() {
  display.clearDisplay();

  display.setCursor(0, ROW0);
  display.printf("INBOX %d/%d", scrollIndex + 1, max(messageCount, 1));
  if (unreadCount > 0) { display.setCursor(84, ROW0); display.printf("+%d new", unreadCount); }

  if (scrollIndex > 0)                { display.setCursor(120, ROW0); display.print('^'); }
  if (scrollIndex < messageCount - 1)  { display.setCursor(120, ROW1); display.print('v'); }

  for (int slot = 0; slot < 2; slot++) {
    int idx = scrollIndex + slot;
    display.setCursor(0, ROW1 + slot * 8);
    if (idx < messageCount) {
      AckStatus as = msgAck[idx];
      if      (as == ACK_WAIT) display.print("..");
      else if (as == ACK_OK)   display.print("+>");
      else if (as == ACK_FAIL) display.print("!>");
      else                     display.print("  ");
      String m = messages[idx];
      int maxC = COLS - 3;
      if ((int)m.length() > maxC) m = m.substring(0, maxC - 1) + ">";
      display.print(m);
    }
  }

  display.drawFastHLine(0, ROW3 - 1, SCREEN_WIDTH, SSD1306_WHITE);
  drawHintBar("^v:nav", "C:read", "SET:cfg");
  display.display();
}

void drawDetail() {
  display.clearDisplay();

  const int pageSize = COLS * 2;
  String msg     = (scrollIndex < messageCount) ? messages[scrollIndex] : "";
  int msgLen     = msg.length();
  int totalPages = max(1, (msgLen + pageSize - 1) / pageSize);
  int curPage    = detailScrollOffset / pageSize + 1;

  display.setCursor(0, ROW0);
  display.printf("MSG%d/%d p%d/%d", scrollIndex+1, messageCount, curPage, totalPages);

  AckStatus as = (scrollIndex < messageCount) ? msgAck[scrollIndex] : ACK_NONE;
  const char* ackStr = (as==ACK_OK)?"[+]":(as==ACK_FAIL)?"[!]":(as==ACK_WAIT)?"[.]":"";
  display.setCursor(SCREEN_WIDTH - (int)strlen(ackStr) * 6, ROW0);
  display.print(ackStr);

  for (int row = 0; row < 2; row++) {
    int start = detailScrollOffset + row * COLS;
    if (start >= msgLen) break;
    display.setCursor(0, ROW1 + row * 8);
    display.print(msg.substring(start, min(start + COLS, msgLen)));
  }

  display.drawFastHLine(0, ROW3 - 1, SCREEN_WIDTH, SSD1306_WHITE);
  drawHintBar("<>:page", "C:back", "");
  display.display();
}

void drawIdle() {
  display.clearDisplay();

  unsigned long t = millis() / 1000;
  display.setCursor(0, ROW0);
  display.printf("Up %02d:%02d:%02d",
                 (int)(t/3600)%24, (int)(t/60)%60, (int)(t%60));

  if (bestBridgeIdx >= 0) {
    display.setCursor(90, ROW0);
    display.printf("GW%+d", bridges[bestBridgeIdx].rssi);
  } else {
    display.setCursor(96, ROW0); display.print("noGW");
  }

  display.setCursor(0, ROW1);
  if (unreadCount > 0) display.printf("** %d NEW **", unreadCount);
  else                 display.print("No new msgs");

  display.setCursor(0, ROW2);
  display.printf("Me:@%02X To:%s", MY_ADDR, destination.c_str());

  display.drawFastHLine(0, ROW3 - 1, SCREEN_WIDTH, SSD1306_WHITE);
  display.setCursor(0, ROW3); display.print("Any key to wake");
  display.display();
}

void drawSettings() {
  display.clearDisplay();

  display.setCursor(0, ROW0); display.print("SETTINGS");
  if (bestBridgeIdx >= 0) {
    display.setCursor(72, ROW0);
    display.printf("GW:%+ddBm", bridges[bestBridgeIdx].rssi);
  }

  display.setCursor(0, ROW1); display.print("Dest: @");
  char hiChar = "0123456789ABCDEF"[destAddr >> 4];
  char loChar = "0123456789ABCDEF"[destAddr & 0x0F];
  int  hiX = 7 * 6, loX = 8 * 6;
  if (settingsCursor == 0) {
    drawHighlightChar(hiX, ROW1, hiChar);
    display.setCursor(loX, ROW1); display.print(loChar);
  } else {
    display.setCursor(hiX, ROW1); display.print(hiChar);
    drawHighlightChar(loX, ROW1, loChar);
  }

  display.setCursor(0, ROW2);
  if (destAddr == BROADCAST_ADDR) display.print("= 0xFF  [BROADCAST]");
  else if (destAddr == MY_ADDR) display.printf("= 0x%02X  [SELF]", MY_ADDR);
  else display.printf("= 0x%02X  (%3d dec)", destAddr, destAddr);

  display.drawFastHLine(0, ROW3 - 1, SCREEN_WIDTH, SSD1306_WHITE);
  drawHintBar("^v:val", "<>:digit", "C:ok");
  display.display();
}

// ── redrawCurrent (declared forward above, defined here) ─────
void redrawCurrent() {
  switch (currentMenu) {
    case MENU_EDITOR:   drawEditor();   break;
    case MENU_HISTORY:  drawHistory();  break;
    case MENU_DETAIL:   drawDetail();   break;
    case MENU_SETTINGS: drawSettings(); break;
    default:            drawIdle();     break;
  }
}

// ============================================================
//  SECTION 15 – MENU NAVIGATION
// ============================================================

bool handleNavigation(int btnIndex, ButtonEvent event) {
  if (btnIndex != 5 || event != PRESS) return false;
  switch (currentMenu) {
    case MENU_EDITOR:   currentMenu = MENU_HISTORY;  unreadCount = 0; break;
    case MENU_HISTORY:  currentMenu = MENU_SETTINGS; break;
    case MENU_SETTINGS: currentMenu = MENU_EDITOR;   break;
    case MENU_DETAIL:   currentMenu = MENU_HISTORY;  break;
    default:            currentMenu = MENU_EDITOR;   break;
  }
  scrollIndex = 0;
  return true;
}

// ============================================================
//  SECTION 16 – SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for (int i = 0; i < buttonCount; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    buttons[i] = { buttonPins[i], HIGH, HIGH, 0, 0, 0, false };
  }

  // Init caches
  for (int i = 0; i < DEDUP_CACHE; i++) dedupCache[i] = MSG_ID_NONE;
  memset(bridges, 0, sizeof(bridges));
  memset(rxBuf,   0, sizeof(rxBuf));
  memset(msgAck,  0, sizeof(msgAck));
  for (int i = 0; i < MAX_MESSAGES; i++) msgIds[i] = MSG_ID_NONE;

  // Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(80);
  WiFi.disconnect();
  esp_wifi_set_protocol(WIFI_IF_STA,
    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
    WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
  if (esp_now_init() != ESP_OK) {
    display.setCursor(0,0); display.print("ESP-NOW FAIL");
    display.display(); while (true);
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ESPNOW_BROADCAST, 6);
  peer.channel = 0; peer.encrypt = false;
  esp_now_add_peer(&peer);
  esp_now_register_recv_cb(onReceive);

  messageBody[0] = '\0';
  updateDestination();
  drawIdle();
}

// ============================================================
//  SECTION 17 – LOOP
// ============================================================

void loop() {
  unsigned long now = millis();

  if (now > ledFlashUntil) digitalWrite(LED_PIN, LOW);

  // ACK status expiry
  if (lastAckStatus != ACK_NONE && ackStatusExpiry > 0 && now > ackStatusExpiry) {
    lastAckStatus = ACK_NONE; ackStatusExpiry = 0;
  }

  tickAck();

  // Bridge expiry every 5 s
  static unsigned long lastPrune = 0;
  if (now - lastPrune > 5000) { lastPrune = now; pruneBridges(); }

  // Drain ESP-NOW receive ring buffer
  while (rxRead != rxWrite) {
    if (rxBuf[rxRead].used) {
      // Copy out of buffer into local vars before processing
      MeshPacket pkt = rxBuf[rxRead].pkt;
      uint8_t    mac[6]; memcpy(mac, rxBuf[rxRead].mac, 6);
      int8_t     rssi = rxBuf[rxRead].rssi;
      rxBuf[rxRead].used = false;
      processPacket(pkt, mac, rssi);
    }
    rxRead = (rxRead + 1) % RX_BUF_SIZE;
  }

  // Idle timeout
  if (currentMenu != MENU_IDLE && now - lastActivity > idleTimeout) {
    currentMenu = MENU_IDLE; drawIdle();
  }

  // Buttons
  for (int i = 0; i < buttonCount; i++) {
    ButtonEvent event = updateButton(i);
    if (event == NONE) continue;

    lastActivity = now;

    if (currentMenu == MENU_IDLE) {
      currentMenu = MENU_EDITOR; drawEditor(); continue;
    }

    if (handleNavigation(i, event)) { redrawCurrent(); continue; }

    switch (currentMenu) {

      // ── EDITOR ──────────────────────────────────────────────
      case MENU_EDITOR: {
        int bodyLen = strlen(messageBody);
        if (i==2 && (event==PRESS||event==REPEAT)) { if(cursorPos>0) cursorPos--; }
        if (i==3 && (event==PRESS||event==REPEAT)) {
          if (cursorPos<bodyLen) { cursorPos++; }
          else if (bodyLen<MAX_MESSAGE_LENGTH) {
            messageBody[cursorPos]='A'; messageBody[cursorPos+1]='\0'; cursorPos++;
          }
        }
        if (i==0 && (event==PRESS||event==REPEAT)) {
          if (cursorPos==bodyLen && bodyLen<MAX_MESSAGE_LENGTH)
            { messageBody[cursorPos]=charSet[0]; messageBody[cursorPos+1]='\0'; }
          if (cursorPos<(int)strlen(messageBody)) {
            char &c=messageBody[cursorPos]; int idx=0;
            for(int j=0;j<charSetSize;j++) if(charSet[j]==c){idx=j;break;}
            c=charSet[(idx+1)%charSetSize];
          }
        }
        if (i==1 && (event==PRESS||event==REPEAT)) {
          if (cursorPos==bodyLen && bodyLen<MAX_MESSAGE_LENGTH)
            { messageBody[cursorPos]=charSet[0]; messageBody[cursorPos+1]='\0'; }
          if (cursorPos<(int)strlen(messageBody)) {
            char &c=messageBody[cursorPos]; int idx=0;
            for(int j=0;j<charSetSize;j++) if(charSet[j]==c){idx=j;break;}
            c=charSet[(idx+charSetSize-1)%charSetSize];
          }
        }
        if (i==4 && event==PRESS)
          { if(cursorPos<(int)strlen(messageBody)) cursorPos++; }
        if (i==4 && event==LONG_PRESS) { sendCurrentText(); }
        if (i==6 && (event==PRESS||event==REPEAT)) {
          int len=strlen(messageBody);
          if (len>0 && cursorPos<len) {
            for(int j=cursorPos;j<len;j++) messageBody[j]=messageBody[j+1];
            if(cursorPos>=(int)strlen(messageBody)&&cursorPos>0) cursorPos--;
          }
        }
        drawEditor(); break;
      }

      // ── HISTORY ─────────────────────────────────────────────
      case MENU_HISTORY:
        if (i==0&&(event==PRESS||event==REPEAT)&&scrollIndex>0)             scrollIndex--;
        if (i==1&&(event==PRESS||event==REPEAT)&&scrollIndex<messageCount-1) scrollIndex++;
        if (i==4&&event==PRESS)
          { detailScrollOffset=0; currentMenu=MENU_DETAIL; drawDetail(); }
        else drawHistory();
        break;

      // ── DETAIL ──────────────────────────────────────────────
      case MENU_DETAIL: {
        String msg=(scrollIndex<messageCount)?messages[scrollIndex]:"";
        int pageSize=COLS*2;
        if (i==2&&(event==PRESS||event==REPEAT))
          detailScrollOffset=max(0,detailScrollOffset-pageSize);
        if (i==3&&(event==PRESS||event==REPEAT))
          detailScrollOffset=min(max(0,(int)msg.length()-1),
                                 detailScrollOffset+pageSize);
        if ((i==4||i==6)&&event==PRESS)
          { currentMenu=MENU_HISTORY; drawHistory(); break; }
        drawDetail(); break;
      }

      // ── SETTINGS ────────────────────────────────────────────
      case MENU_SETTINGS: {
        if (i==0&&(event==PRESS||event==REPEAT)) {
          if(settingsCursor==0) destAddr=(destAddr&0x0F)|(((destAddr>>4)+1)&0x0F)<<4;
          else                  destAddr=(destAddr&0xF0)|((destAddr+1)&0x0F);
          updateDestination();
        }
        if (i==1&&(event==PRESS||event==REPEAT)) {
          if(settingsCursor==0) destAddr=(destAddr&0x0F)|(((destAddr>>4)-1)&0x0F)<<4;
          else                  destAddr=(destAddr&0xF0)|((destAddr-1)&0x0F);
          updateDestination();
        }
        if ((i==2||i==3)&&event==PRESS) settingsCursor=1-settingsCursor;
        if (i==4&&event==PRESS) { currentMenu=MENU_EDITOR; drawEditor(); break; }
        drawSettings(); break;
      }

      default: break;
    }
  }

  // Idle clock tick
  static unsigned long lastIdleDraw = 0;
  if (currentMenu==MENU_IDLE && now-lastIdleDraw>1000)
    { lastIdleDraw=now; drawIdle(); }

  // Serial debug passthrough
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) addMessage("DBG:" + line);
    if (currentMenu==MENU_EDITOR||currentMenu==MENU_IDLE) redrawCurrent();
  }
}
