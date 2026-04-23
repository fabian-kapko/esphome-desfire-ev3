# esphome-desfire-ev3

An ESPHome external component for **MIFARE DESFire EV3** (and EV1/EV2) NFC card authentication using a **PN532** reader over I2C.

Implements the **EV2 secure channel** (`AuthenticateEV2First`, cmd `0x71`) with per-tap session key derivation, AES-CBC encryption, and CMAC integrity verification. Self-contained PN532 I2C driver — no dependency on ESPHome's built-in `pn532` component.

## Features

- **EV2 mutual authentication** — both card and reader prove key knowledge before any data flows
- **Per-session encryption** — fresh AES-128 keys derived from random nonces on every tap
- **CMAC integrity** — response MAC verified over ciphertext, tamper detection on every read
- **Three comm modes** — Full (encrypted + CMAC), MAC (cleartext + CMAC), Plain (legacy)
- **Self-contained PN532 driver** — speaks the PN532 I2C frame protocol directly
- **I2C bus recovery** — automatic SDA/SCL bit-bang recovery on bus lockups
- **Portable AES-128** — software implementation, no external crypto libraries

## Hardware

| Component | Notes |
|-----------|-------|
| ESP32 | Any ESPHome-supported board with I2C (must be an arduino framework, if you have specific boards, most will work with just generic ESP32 variant without specifing other details) |
| PN532 NFC module | I2C mode (set DIP switches to I2C), default address `0x24` |

Wire the PN532 to your ESP32's I2C bus (SDA/SCL). The component auto-configures the PN532 at startup.

## Installation

### External component (recommended)

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/fabian-kapko/esphome-desfire-ev3
      ref: main
```

### Manual

Copy the `components/desfire_reader/` directory into your ESPHome config:

```
config/
├── your-device.yaml
└── components/
    └── desfire_reader/
        ├── __init__.py
        ├── desfire_reader.h
        └── desfire_reader.cpp
```

## Configuration

### Full mode (recommended)

```yaml
i2c:
  sda: GPIO25
  scl: GPIO26
  frequency: 100kHz
  scan: true
  id: bus_a

external_components:
  - source:
      type: git
      url: https://github.com/fabian-kapko/esphome-desfire-ev3
      ref: main

desfire_reader:
  id: my_reader
  address: 0x24
  app_id: "AA:BB:CC"
  app_key: "01:23:45:67:89:AB:CE:F0:12:34:56:78:9A:BC:EF"
  comm_mode: full

  uid:
    name: "Card UID"
    id: card_uid

  nonauthorised_card_uid:
    name: "Nonauthorised Card UID"

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

### Plain mode (legacy, backward compatible)

For cards provisioned with a separate `data_key` — static ciphertext stored on card, decrypted by the reader:

```yaml
desfire_reader:
  id: my_reader
  address: 0x24
  app_id: "AA:BB:CC"
  app_key: "01:23:45:67:89:AB:CE:F0:12:34:56:78:9A:BC:EF"
  data_key: "12:34:56:78:9A:BC:EF:01:23:45:67:89:AB:CE:F0"
  comm_mode: plain

  uid:
    name: "Card UID"
  result:
    name: "Card Result"
  auth_ok:
    name: "Card Auth OK"
```

### Configuration reference

| Key | Required | Default | Description |
|-----|----------|---------|-------------|
| `address` | No | `0x24` | PN532 I2C address |
| `update_interval` | No | `50ms` | Card polling interval |
| `app_id` | **Yes** | — | 3-byte DESFire Application ID (`XX:XX:XX`) |
| `app_key` | **Yes** | — | 16-byte AES-128 application key (`XX:XX:...:XX`) |
| `data_key` | No | — | 16-byte AES-128 decryption key (plain mode only) |
| `comm_mode` | No | `plain` | `full`, `mac`, or `plain` |
| `result` | No | — | Text sensor: decrypted file payload |
| `auth_ok` | No | — | Binary sensor: `true` while authenticated card is present |
| `uid` | No | — | Text sensor: card UID (after successful auth) |
| `nonauthorised_card_uid` | No | — | Text sensor: UID of cards that fail authentication |

## Communication modes

| Mode | Card stores | RF transfer | Reader needs |
|------|------------|-------------|-------------|
| **full** | Plaintext | AES-CBC encrypted + CMAC (session key) | `app_key` only |
| **mac** | Plaintext | Cleartext + CMAC integrity | `app_key` only |
| **plain** | Ciphertext (pre-encrypted with `data_key`) | Static ciphertext | `app_key` + `data_key` |

**Full mode is recommended.** The card stores plaintext; the EV2 session handles all RF encryption and integrity. No `data_key` needed — one fewer secret to manage.

## How it works

```
ESP32 ──I2C──► PN532 ──13.56 MHz──► DESFire EV3
```

On each poll:

1. **Detect** — `InListPassiveTarget` (ISO 14443-A, 106 kbps)
2. **SelectApplication** — selects the configured AID
3. **AuthenticateEV2First** (`0x71`) — mutual 3-pass AES-128 handshake, derives fresh session keys from RndA ⊕ RndB
4. **ReadData** — reads file `0x01` through the EV2 secure channel
5. **Decrypt + verify** — AES-CBC decrypt with session key, CMAC verified over ciphertext (Full mode)
6. **Publish** — pushes UID, payload, and auth state to Home Assistant sensors

When the card is removed, all sensors clear automatically.

### PN532 I2C driver

The component implements the PN532 I2C frame protocol directly (preamble · start-code · LEN · LCS · TFI · data · DCS · postamble). This means:

- No `pn532_i2c:` entry needed in YAML
- No conflict with other PN532-based components
- Automatic I2C bus recovery if SDA gets stuck low

## Security

| Layer | Protection |
|-------|-----------|
| Authentication | AuthenticateEV2First (`0x71`) — mutual AES-128 with TI binding |
| Session keys | Derived per-tap via AES-CMAC over RndA/RndB |
| RF encryption | AES-CBC with per-session IV (Full mode) |
| Integrity | CMAC verified over ciphertext on every response |
| Replay protection | Command counter increments per command-response pair |
| RNG | Hardware (`esp_fill_random` on ESP32) |
| Key hygiene | All intermediate crypto zeroed via volatile writes |

### What it defeats

- **UID cloning** (Flipper, Proxmark, phone apps) — fails at mutual AES auth
- **RF eavesdropping** (SDR) — different encrypted traffic every tap (Full mode)
- **Replay attacks** — per-session keys + command counter
- **Data injection** — CMAC detects tampered responses

### Remaining risks

- **Relay attacks** — DESFire EV3 proximity check not yet supported (requires PICC-level config)
- **Static key sharing** — all cards use the same `app_key`; mitigate with ESP32 flash encryption + secure boot
- **No originality check** — hardware emulators with the correct key could impersonate a card

## Card provisioning

Use the companion toolkit: **[desfire-ev3-pywriter](https://github.com/fabian-kapko/desfire-ev3-pywriter)**

### Provision for Full mode

```bash
python3 provision.py \
  --picc-key 0000000000000000 \
  --app-id AA:BB:CC \
  --app-key 0123456789ABCEF0123456789ABCEF \
  --data "EMP001"
```

### Provision for Plain mode (legacy)

```bash
python3 provision.py \
  --picc-key 0000000000000000 \
  --app-id AA:BB:CC \
  --app-key 0123456789ABCEF0123456789ABCEF \
  --data-key 123456789ABCEF0123456789ABCEF0  \
  --data "EMP001"
```

### Factory reset

```bash
python3 factory_reset.py --picc-key 0000000000000000 --desfsh ./desfsh
```

## License

MIT — see [LICENSE](LICENSE).
