# esphome-desfire-ev3

An ESPHome external component that reads and authenticates **MIFARE DESFire EV3** (and EV1/EV2) NFC cards using a **PN532** reader over I2C — with no dependency on ESPHome's built-in `pn532` component.

---

## Features

- **Non-blocking state machine** — never stalls the ESPHome main loop; WiFi, MQTT, OTA, and other components stay responsive at all times
- **AES-128 mutual authentication** with a DESFire application key
- **Encrypted file read** — decrypts an AES-CBC payload from a card file
- **Self-contained PN532 driver** — implements the PN532 I2C frame protocol directly (no `pn532_i2c:` needed)
- **Automatic retry with I2C bus recovery** — transient RF/I2C failures are retried transparently (up to 2 retries) with hardware bus recovery
- **Fast card detection** — full auth + read completes in ~80–150 ms; card removal detected within ~300 ms
- **Text and binary sensors** — UID, decrypted result, and auth status published to Home Assistant
- Portable **software AES-128** — no external crypto libraries required
- **ESP32 compatible**

---

## Hardware

| Component | Notes |
|-----------|-------|
| **ESP32** (recommended) | Any ESPHome-supported board. Dual-core gives best results. |
| **PN532 NFC module** | Connected via I2C (default address `0x24`). |

### Wiring

Wire the PN532 to the ESP's I2C bus and set its mode switches to **I2C**.

**Recommended ESP32 pins:**

| Signal | GPIO | Notes |
|--------|------|-------|
| SDA | **GPIO 21** | Default I2C data line |
| SCL | **GPIO 22** | Default I2C clock line |

> **Pull-ups:** Use **4.7 kΩ external pull-ups** on both SDA and SCL to 3.3 V. Most PN532 breakout boards (Adafruit, Elechouse) already have pull-ups soldered on — if yours does, don't add external ones.

**Pins to avoid:** GPIO 0, 1, 2, 3, 5, 6–11, 12, 15, 34–39. See the [ESP32 GPIO reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html) for details.

```
ESP32                  PN532
─────                  ─────
GPIO 21 (SDA) ───────► SDA
GPIO 22 (SCL) ───────► SCL
3.3V          ───────► VCC
GND           ───────► GND
```

---

## Installation

### Option 1: Git reference (recommended)

Add to your YAML — ESPHome will clone the component automatically:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/fabian-kapko/esphome-desfire-ev3
      ref: main
```

### Option 2: Local copy

Copy the `components/desfire_reader/` directory into your ESPHome config folder:

```
your-esphome-config/
├── your-device.yaml
└── components/
    └── desfire_reader/
        ├── __init__.py
        ├── desfire_reader.h
        └── desfire_reader.cpp
```

---

## Configuration

### Minimal example

```yaml
i2c:
  sda: GPIO21
  scl: GPIO22
  scan: true
  frequency: 400kHz

external_components:
  - source:
      type: git
      url: https://github.com/fabian-kapko/esphome-desfire-ev3
      ref: main

desfire_reader:
  id: my_reader
  address: 0x24
  update_interval: 50ms

  app_id: "A1:B2:C3"
  app_key: "00:11:22:33:44:55:66:77:88:99:AA:BB:CC:DD:EE:FF"
  data_key: "AA:BB:CC:DD:EE:FF:00:11:22:33:44:55:66:77:88:99"

  uid:
    name: "Card UID"
    id: card_uid

  result:
    name: "Card Result"
    id: card_data

  auth_ok:
    name: "Card Auth OK"
    id: card_auth
    on_press:
      then:
        - logger.log:
            format: "Valid card: %s"
            args: ["id(card_data).state.c_str()"]
```

### Full example (door lock with relay)

```yaml
i2c:
  sda: GPIO21
  scl: GPIO22
  scan: true
  frequency: 400kHz

external_components:
  - source:
      type: git
      url: https://github.com/fabian-kapko/esphome-desfire-ev3
      ref: main

switch:
  - platform: gpio
    pin: GPIO27
    id: door_relay
    name: "Door Lock"
    restore_mode: ALWAYS_OFF

desfire_reader:
  id: nfc_reader
  address: 0x24
  update_interval: 50ms

  app_id: "A1:B2:C3"
  app_key: "00:11:22:33:44:55:66:77:88:99:AA:BB:CC:DD:EE:FF"
  data_key: "AA:BB:CC:DD:EE:FF:00:11:22:33:44:55:66:77:88:99"

  uid:
    name: "Card UID"

  result:
    name: "Card Result"
    id: card_data

  auth_ok:
    name: "Card Auth OK"
    on_press:
      then:
        - logger.log:
            format: "Access granted: %s"
            args: ["id(card_data).state.c_str()"]
        - switch.turn_on: door_relay
        - delay: 3s
        - switch.turn_off: door_relay

sensor:
  - platform: wifi_signal
    name: "RSSI"
    update_interval: 60s
```

### Configuration variables

| Key | Required | Default | Description |
|-----|----------|---------|-------------|
| `address` | No | `0x24` | I2C address of the PN532 |
| `update_interval` | No | `500ms` | How often to start a new card detection cycle |
| `app_id` | **Yes** | — | 3-byte DESFire Application ID in `XX:XX:XX` hex format |
| `app_key` | **Yes** | — | 16-byte AES-128 key for application authentication |
| `data_key` | **Yes** | — | 16-byte AES-128 key to decrypt file `0x01` |
| `result` | No | — | Text sensor — decrypted plaintext from the card file |
| `auth_ok` | No | — | Binary sensor — `true` while an authenticated card is present |
| `uid` | No | — | Text sensor — card UID in `XX:XX:XX:XX:XX:XX:XX` format |

---

## How it works

```
ESP ──I2C──► PN532 ──RF──► DESFire card
```

### Architecture

The component uses a **non-blocking state machine** that runs inside ESPHome's `loop()` (~16 ms cycle time). Each loop iteration performs at most **one** I2C operation, then returns control to the main loop. This ensures WiFi, MQTT, OTA, and all other components remain fully responsive.

`update()` (called at `update_interval`) only sets a flag to trigger a new card detection cycle. The actual protocol steps are driven by `loop()`.

### Protocol flow

1. **Card detection** — sends `InListPassiveTarget` (ISO 14443-A, 106 kbps)
2. **SelectApplication** — selects the configured App ID on the card
3. **AES authentication** — performs the DESFire AES mutual-auth handshake (challenge/response with `RndA` / `RndB`)
4. **ReadData** — reads from file `0x01`
5. **AES-128-CBC decrypt** — decrypts the file payload with `data_key` and a zero IV
6. **Publish** — pushes UID, result string, and auth state to ESPHome sensors

When the card is removed (2 consecutive missed polls), sensors are cleared back to their default state.

### Error recovery

| Failure type | Behaviour |
|---|---|
| **Timeout** (ACK or response) | Automatic I2C bus recovery → PN532 wakeup → retry (up to 2 retries) |
| **Wrong application** (SelectApp fails) | Immediate fail — card doesn't have the expected app |
| **Wrong key** (auth fails) | Immediate fail — crypto mismatch, not retryable |
| **I2C hardware timeout** | Bus recovery (9 SCL clocks + STOP condition) → re-initialize → retry |
| **Max retries exceeded** | Exponential backoff cooldown (100 ms → 200 ms → 400 ms → … → 5 s max) |

### Timing

| Operation | Typical time |
|---|---|
| Full auth + read (happy path) | ~80–150 ms |
| Card removal detection | ~300 ms |
| Retry after transient failure | ~50 ms |
| Single `loop()` iteration | < 2 ms |

### PN532 I2C framing

The component speaks directly to the PN532 using its native frame format (preamble · start-code · LEN · LCS · TFI · data · DCS · postamble) without relying on ESPHome's built-in PN532 driver. This means:

- No `pn532_i2c:` entry is needed in your YAML
- No conflict if you happen to have other PN532-based components
- The component is fully self-contained

---

## Card preparation

The card must have an application with:

- **App ID** matching `app_id`
- **Key 0** matching `app_key` (AES-128)
- **File 0x01** — containing data encrypted with `data_key` under AES-128-CBC with a zero IV

You can provision cards with [desfire-ev3-pywriter](https://github.com/fabian-kapko/desfire-ev3-pywriter).

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `PN532 not responding` at boot | Bad wiring or wrong I2C address | Check SDA/SCL wiring, confirm PN532 mode switches are set to I2C, verify address with `i2c: scan: true` |
| `SelectApp FAILED` | Wrong `app_id` or card doesn't have the app | Verify `app_id` matches the application on the card |
| `AES auth FAILED — wrong app key?` | `app_key` doesn't match key 0 on the card | Re-provision the card with the correct key |
| `Auth1 read timeout` (occasional) | RF signal instability, card at edge of field | Hold card closer to reader; component will auto-retry up to 2 times |
| `I2C hardware timeout detected` | I2C bus locked up after a failed transaction | Component auto-recovers the bus; if persistent, check wiring and pull-ups |
| Frequent API disconnects | I2C operations blocking too long | Should not happen with current version; check for other blocking components |

---

## License

MIT — see [LICENSE](LICENSE).