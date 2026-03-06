# esphome-desfire-ev3
An ESPHome external component that reads and authenticates **MIFARE DESFire EV3** (and EV1/EV2) NFC cards using a **PN532** reader over I2C — with no dependency on ESPHome's built-in `pn532` component.

---

## Features

- **AES-128 mutual authentication** with a DESFire application key
- **Encrypted file read** — decrypts a 16-byte AES-CBC payload from a card file
- **Self-contained PN532 driver** — implements the PN532 I2C frame protocol directly
- **Text and binary sensors** —  decrypted result, and auth status to Home Assistant
- Portable **software AES-128** (no external crypto libraries required)

---

## Hardware

| Component | Notes |
|-----------|-------|
| ESP8266 / ESP32 | Any ESPHome-supported board |
| PN532 NFC module | Connected via I2C (default address `0x24`) |

Wire the PN532 to the ESP's I2C bus (SDA/SCL) and set its mode switches to **I2C**.

---

## Installation

Copy (or symlink) the `components/desfire_reader/` directory into your ESPHome config folder:

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

```yaml
i2c:
  sda: GPIO4
  scl: GPIO5
  scan: true

desfire_reader:
  id: my_desfire
  address: 0x24
  update_interval: 1000ms
  app_id: "A1:B2:C3"
  app_key:  "00:11:22:33:44:55:66:77:88:99:AA:BB:CC:DD:EE:FF"
  data_key: "AA:BB:CC:DD:EE:FF:00:11:22:33:44:55:66:77:88:99"  # AES key to decrypt file data
  result:
    name: "Card Data"
    id: card_data
  auth_ok:
    name: "Card Auth OK"
    id: card_auth
    on_state:
      then:
        - if:
            condition:
              binary_sensor.is_on: card_auth
            then:
              - logger.log:
                  format: "Valid card: %s"
                  args: ["id(card_data).state.c_str()"]

```

### Configuration variables

| Key | Required | Description |
|-----|----------|-------------|
| `address` | No | I2C address of the PN532, default `0x24` |
| `update_interval` | No | How often to poll for a card, default `500ms` |
| `app_id` | **Yes** | 3-byte DESFire Application ID, `XX:XX:XX` hex |
| `app_key` | **Yes** | 16-byte AES-128 key for application authentication |
| `data_key` | **Yes** | 16-byte AES-128 key to decrypt file `0x01` |
| `result` | No | Text sensor — decrypted plaintext from the card file |
| `auth_ok` | No | Binary sensor — `true` while an authenticated card is present |
| `uid` | No | Text sensor — card UID (populated on successful read) |

---

## How it works

```
ESP ──I2C──► PN532 ──RF──► DESFire card
```

On each `update_interval`:

1. **Card detection** — sends `InListPassiveTarget` (ISO 14443-A, 106 kbps)
2. **SelectApplication** — selects the configured App ID on the card
3. **AES authentication** — performs the DESFire AES mutual-auth handshake (challenge/response with `RndA` / `RndB`)
4. **ReadData** — reads 16 bytes from file `0x01`
5. **AES-128-CBC decrypt** — decrypts the file payload with `data_key` and a zero IV
6. **Publish** — pushes the result string and auth state to ESPHome sensors

When the card is removed, sensors are cleared back to their default state.

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
- **File 0x01** — at least 16 bytes, containing data encrypted with `data_key` under AES-128-CBC with a zero IV

You can provision cards with tools such as [LibNFC](https://nfc-tools.github.io/projects/libnfc/), [MIFARE DESFire tools](https://github.com/nfc-tools/mifare-desfire), or a dedicated Android app.

---

## License

MIT — see [LICENSE](LICENSE).
