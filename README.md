# ESP-NOW-LoRa Messenger

> [!IMPORTANT]
> **Disclaimer:** This project was developed with the assistance of an AI coding assistant (Claude by Anthropic). The architecture, hardware design, protocol decisions, and testing were done by the project author. AI was used as a development tool throughout the firmware debugging and iteration process.

> [!NOTE]
> **No warranty:** This project is provided as-is, for educational and hobbyist purposes only. It may not work correctly in all configurations, environments, or hardware combinations. The author takes no responsibility for any damage to hardware, loss of data, or any other issues arising from the use of this project. Use at your own risk.

An off-grid mesh messenger for ESP32, combining **ESP-NOW** for local wireless clusters and **LoRa** (REYAX RYLR998) for long-range backbone links between clusters. Nodes have a 7-button keyboard or Five Directional Joystick and SSD1306 OLED display. Messages are delivered with end-to-end acknowledgement across the full mesh.

---

## Features

- Text messaging between any nodes in the mesh
- ESP32 Long Range (LR) mode + 20 dBm TX power for extended ESP-NOW range (~1 km LOS)
- Two-tier ACK protocol — PKT_LORACK confirms LoRa handoff, PKT_ACK confirms final delivery
- Automatic bridge selection based on RSSI beacon scanning
- TTL-based mesh forwarding with per-type deduplication
- Retry and graceful failure (`[+]` / `[!]` delivery indicators)
- Message history with scroll and detail view
- OLED stats display on bridge nodes (traffic counters, RSSI, peer list)
- Supports multiple simultaneous in-flight messages across bridges

---

## Repository Structure

```
/end_node/messenger.ino          — End node firmware (keyboard + display)
/bridge_node/messenger_LORA.ino  — Bridge node firmware (ESP-NOW ↔ LoRa)
README.md
LICENSE
```

---

## Hardware

### End Node

| Component | Notes |
|---|---|
| ESP32 dev board | Any standard 30-pin board |
| SSD1306 128×32 OLED | I2C, address 0x3C |
| 7× tactile buttons or Five Directional Joystick | See pin assignments below |
| LED | GPIO 2, optional |

#### End Node Pin Assignments

| OLED Pin | GPIO |
|---|---|
| SDA (OLED) | 21 (default I2C) |
| SCL (OLED) | 22 (default I2C) |
| VCC (OLED) | 3.3V |
| GND (OLED) | GND |

| Function | GPIO |
|---|---|
| BTN_UP | 12 |
| BTN_DOWN | 14 |
| BTN_LEFT | 27 |
| BTN_RIGHT | 26 |
| BTN_CENTER | 25 |
| BTN_SET | 33 |
| BTN_RST | 32 |
| LED | 2 |

All buttons are active-LOW with `INPUT_PULLUP`. Wire each button between its GPIO and GND or in case of linked joystick wire COM to GND.

---

### Bridge Node

| Component | Notes |
|---|---|
| ESP32 dev board | Any standard 30-pin board |
| REYAX RYLR998 | LoRa module, UART AT command interface |
| SSD1306 128×32 OLED | I2C, address 0x3C |
| LED | GPIO 2, optional |
| Tactile button | GPIO 0 (BOOT button), cycles OLED display pages |

#### Bridge Node Wiring

| RYLR998 Pin | ESP32 GPIO | Notes |
|---|---|---|
| VCC | 3.3V | Do not use 5V |
| GND | GND | |
| TXD | GPIO 16 (RX2) | |
| RXD | GPIO 17 (TX2) | |
| RST | GPIO 4 | Optional hard reset |

| Component | ESP32 GPIO |
|---|---|
| SDA (OLED) | 21 |
| SCL (OLED) | 22 |
| VCC (OLED) | 3.3V |
| GND (OLED) | GND |
| LED | 2 |
| Page button | 0 (BOOT) |

---

## Network Topology

```
  [ Node 1 ]──ESP-NOW──[ Bridge A ]══LoRa══[ Bridge B ]──ESP-NOW──[ Node 2 ]
  [ Node 3 ]──ESP-NOW──╯                              ╰──ESP-NOW──[ Node 4 ]
```

- **End nodes** communicate with the nearest bridge over ESP-NOW (2.4 GHz, up to ~1 km line of sight with ESP32 Long Range mode enabled)
- **Bridges** relay messages over LoRa (sub-GHz, km-range depending on antenna and environment)
- Nodes automatically select the bridge with the strongest beacon RSSI
- Up to 4 bridges tracked per node; bridge list pruned after 60s without beacon

---

## Configuration

### End Node (`messenger.ino`)

```cpp
#define MY_ADDR   0x01   // Unique mesh address for this node (0x01–0xFE)
```

Each node must have a unique `MY_ADDR`. `0xFF` is reserved for broadcast.

> [!NOTE]
> **Range:** Both end nodes and bridge nodes enable `WIFI_PROTOCOL_LR` and set TX power to 20 dBm at startup. ESP32 Long Range mode only improves range when **both sides** have it enabled, which is the case here. Real-world range depends heavily on antenna, obstacles, and RF environment.

> [!WARNING]
> **EU users:** TX power is set to `80` (80 × 0.25 = 20 dBm), which complies with the EU 2.4 GHz EIRP limit. If you are outside the EU and want maximum power, change `esp_wifi_set_max_tx_power(80)` to `esp_wifi_set_max_tx_power(84)` (21 dBm) in both firmwares.

### Bridge Node (`messenger_LORA.ino`)

```cpp
#define MY_ADDR         0xB1   // Unique mesh address for this bridge (0xB1, 0xB2, ...)
#define RYLR_ADDR          1   // RYLR998 radio address, unique per bridge (1, 2, ...)
#define RYLR_NETWORK_ID   18   // Must match on all bridges
#define RYLR_FREQUENCY   868   // 868 for EU, 915 for US (MHz)
#define RYLR_SF            9   // Spreading factor 7–12
#define RYLR_BW          125   // Bandwidth: 125 / 250 / 500 kHz
#define RYLR_CR            1   // Coding rate: 1=4/5, 2=4/6, 3=4/7, 4=4/8
#define RYLR_PREAMBLE      8   // Preamble length
```

All bridges on the same backbone **must share** the same `RYLR_NETWORK_ID`, `RYLR_FREQUENCY`, `RYLR_SF`, `RYLR_BW`, `RYLR_CR`, and `RYLR_PREAMBLE`. Each bridge must have a unique `MY_ADDR` and `RYLR_ADDR`.

> [!NOTE]
> `RYLR_ADDR = 0` is reserved as the broadcast address by the RYLR998. Start bridge addresses at 1.

---

## Libraries

Install via Arduino Library Manager:

- **Adafruit SSD1306** by Adafruit
- **Adafruit GFX Library** by Adafruit

ESP-NOW and WiFi are part of the ESP32 Arduino core (no separate install needed).

---

## UI — End Node

### Menus

| Menu | Description |
|---|---|
| **Idle** | Shows time since start, bridge RSSI, unread count |
| **Editor** | Compose and send a message |
| **History** | Scrollable list of sent/received messages with ACK status |
| **Detail** | Full message view with scroll for long messages |
| **Settings** | Set destination address |

### Joystick Button Layout

```
        [  UP  ]
[ LEFT ][CENTER][ RIGHT ]
        [ DOWN ]
[ SET ]          [ RST ]
```

### Editor

| Button | Short Press | Repeat | Long Press |
|---|---|---|---|
| UP | Next char | Next char | — |
| DOWN | Previous char | Previous char | — |
| LEFT | Move cursor left | Move cursor left | — |
| RIGHT | Move cursor right | Move cursor right | — |
| CENTER | Confirm char | — | Send message |
| SET | Cycle through menus | — | — |
| RST | Remove char | Remove chars | — |

### History

| Button | Short Press | Repeat | Long Press |
|---|---|---|---|
| UP | Newer message | Newer message | — |
| DOWN | Older message | Older message | — |
| LEFT | — | — | — |
| RIGHT | — | — | — |
| CENTER | Open message details | — | — |
| SET | Cycle through menus | — | — |
| RST | — | — | — |

### Detail

| Button | Short Press | Repeat | Long Press |
|---|---|---|---|
| UP | — | — | — |
| DOWN | — | — | — |
| LEFT | Previous message page | Previous message page | — |
| RIGHT | Next message page | Next message page | — |
| CENTER | Back to history | — | — |
| SET | Cycle through menus | — | — |
| RST | Back to history | — | — |

### Settings

| Button | Short Press | Repeat | Long Press |
|---|---|---|---|
| UP | Next base 16 number | Next base 16 number | — |
| DOWN | Previous base 16 number | Previous base 16 number | — |
| LEFT | Move between destination positions | — | — |
| RIGHT | Move between destination positions | — | — |
| CENTER | Confirm and open editor | — | — |
| SET | Cycle through menus | — | — |
| RST | — | — | — |

### Delivery Indicators

| Symbol | Meaning |
|---|---|
| `...` | Waiting for ACK |
| `LR.` | Handed off to LoRa backbone (LORACK received) |
| `[+]` | Delivered successfully |
| `[!]` | Delivery failed (bridge gave up or timeout) |

---

## Protocol

### Packet Format (38 bytes)

```
┌──────┬───────┬─────┬─────┬─────┬──────┬─────────┐
│ type │ msgId │ src │ dst │ ttl │ rssi │ payload │
│  1B  │  2B   │ 1B  │ 1B  │ 1B  │  1B  │   31B   │
└──────┴───────┴─────┴─────┴─────┴──────┴─────────┘
```

| Type | Value | Description |
|---|---|---|
| `PKT_DATA` | 0x01 | User message |
| `PKT_ACK` | 0x02 | End-to-end delivery confirmation |
| `PKT_BEACON` | 0x03 | Bridge advertisement (ESP-NOW broadcast) |
| `PKT_LORACK` | 0x04 | LoRa handoff confirmation (bridge → node) |

Packets are transmitted over LoRa as hex-encoded ASCII via RYLR998 AT commands (`AT+SEND`). Each 38-byte packet becomes 76 hex characters, well within the RYLR998's 240-byte payload limit.

### Two-Tier ACK Flow

```
Node1 ──DATA──► Bridge1 ──AT+SEND──► Bridge2 ──ESP-NOW──► Node2
      ◄─LORACK─╯                                          │
      ◄────────────────────────────────────── PKT_ACK ◄───╯
```

1. Node sends DATA to bridge over ESP-NOW
2. Bridge relays over LoRa (`AT+SEND → +OK`)
3. Bridge immediately sends **PKT_LORACK** back to node — "message is on the backbone"
4. Node switches to long 3 s retry timer
5. Destination bridge receives DATA, delivers to destination node over ESP-NOW
6. Destination node sends **PKT_ACK** back
7. ACK is routed back over LoRa to origin bridge, then ESP-NOW to origin node
8. Node shows `[+]`

If the destination bridge never receives the packet (LoRa unreachable), the origin bridge retries once then sends a **fail ACK** (`rssi = -128` sentinel) so the node shows `[!]` promptly instead of waiting indefinitely.

### Deduplication

Two independent 24-slot ring caches:
- **DATA cache** — prevents the same message from being forwarded multiple times
- **ACK cache** — prevents ESP-NOW echo loops when the bridge hears its own unicast transmissions

`msgId` is constructed as `(src << 8) | seq`, guaranteeing uniqueness per source node.

---

## Bridge OLED Display Pages

The bridge cycles through 4 pages automatically every 5 seconds. Press the BOOT button (GPIO 0) to advance manually.

| Page | Content |
|---|---|
| 1 — Identity | Bridge address, uptime, peer count, LoRa state, radio config |
| 2 — Traffic | ESP-NOW rx/tx, LoRa rx/tx, duplicates dropped, retries |
| 3 — Last Packet | Last ESP-NOW sender + RSSI, last LoRa sender + RSSI, last payload |
| 4 — Peers | Known ESP-NOW nodes with MAC addresses (scrollable) |

> [!NOTE]
> Traffic counters count **DATA packets only**. ACK routing traffic is excluded.

---

## Serial Debug Commands (Bridge)

Serial debug commands are only available when `DEBUG` is enabled. To enable, uncomment the following line near the top of `messenger_LORA.ino`:

```cpp
#define DEBUG
```

Then open Serial Monitor at 115200 baud and type any of the following:

| Command | Output |
|---|---|
| `peers` | List all known ESP-NOW peers with mesh address and MAC |
| `status` | Show all active LoRa pending slots (msgId, retry count) |
| `beacon` | Manually send a beacon |
| `AT+...` | Forward raw AT command to RYLR998 |

Enabling `DEBUG` also turns on verbose packet logging for all ESP-NOW and LoRa traffic, useful for tracing message flow and diagnosing delivery failures. In production, leave `DEBUG` commented out — all logging macros compile to nothing with zero overhead.

---

## Timing Reference

### End Node

| Parameter | Value | Notes |
|---|---|---|
| ACK short timeout | 800 ms | Per attempt before LORACK received; 5 × 0.8 s = 4 s total if LoRa never responds |
| ACK long timeout | 3000 ms | Per attempt after LORACK; 5 × 3 s = 15 s total, longer than bridge give-up at ~6 s |
| Max retries | 5 | Applies to both timeout phases independently |
| Idle screen timeout | 10 s | Returns to idle menu after inactivity |
| Button debounce | 30 ms | |
| Long press threshold | 600 ms | |
| Key repeat start | 600 ms | Delay before repeat begins |
| Key repeat interval | 150 ms | Rate once repeating |
| Bridge prune timeout | 60 s | Node drops bridge entry if no beacon received |

### Bridge Node

| Parameter | Value | Notes |
|---|---|---|
| Beacon interval | 2000 ms | ESP-NOW broadcast to local cluster |
| LoRa retry timeout | 3000 ms | Wait per attempt before retrying AT+SEND |
| LoRa max retries | 1 | 1 retry → give-up + fail ACK sent at ~6 s |
| AT+SEND timeout | 2000 ms | Wait for `+OK` response from RYLR998 |
| TX cooldown | 1800 ms | Guard time after `+OK` — radio is still transmitting at SF9/125 kHz (~1.5 s airtime) |

---

## Known Limitations

- Maximum message length: 30 characters
- Maximum nodes per ESP-NOW cluster: 16 (ESP-NOW peer table limit)
- Maximum simultaneous in-flight LoRa messages per bridge: 4
- ACK queue size: 4 (ACKs arriving while radio is busy)
- Character set: uppercase A–Z, digits 0–9, space and `, . ? !`
- No encryption (ESP-NOW and RYLR998 both support it but it is not currently enabled)

---

## Parts & Where to Buy

Links to the components used in this project. Prices and availability may vary by region.

### End Node
| Component | Link |
|---|---|
| ESP32 dev board | [SVK](https://www.drotik-elektro.sk/arduino-platforma/1581-vyvojova-doska-esp32-2-4-ghz-dual-mode-wi-fi-bluetooth-modul-anteny.html) [AliExpress](https://www.aliexpress.com/item/1005006422498371.html) |
| SSD1306 128×32 OLED (I2C) | [SVK](https://www.drotik-elektro.sk/arduino-platforma/1479-iic-i2c-oled-displej-pre-iot-arduino-raspbery-0.91-biely-128-x-32-3-3-v-5v.html) [AliExpress](https://www.aliexpress.com/item/1005008640132638.html) |
| Five Directional Joystick | [AliExpress](https://www.aliexpress.com/item/1005008023446979.html) |

### Bridge Node
| Component | Link |
|---|---|
| ESP32 dev board | [SVK](https://www.drotik-elektro.sk/arduino-platforma/1581-vyvojova-doska-esp32-2-4-ghz-dual-mode-wi-fi-bluetooth-modul-anteny.html) [AliExpress](https://www.aliexpress.com/item/1005006422498371.html) |
| SSD1306 128×32 OLED (I2C) | [SVK](https://www.drotik-elektro.sk/arduino-platforma/1479-iic-i2c-oled-displej-pre-iot-arduino-raspbery-0.91-biely-128-x-32-3-3-v-5v.html) [AliExpress](https://www.aliexpress.com/item/1005008640132638.html) |
| REYAX RYLR998 LoRa module | [Mirifica](https://www.mirifica.de/en/reyax-rylr998_101491_2820/) |

---

## License

MIT License — see `LICENSE` file.
