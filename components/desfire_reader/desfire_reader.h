#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <string.h>

namespace esphome {
namespace desfire_reader {

static const char *const TAG = "desfire";

static const uint8_t DESFIRE_SW1         = 0x91;
static const uint8_t DESFIRE_OK          = 0x00;
static const uint8_t DESFIRE_MORE_FRAMES = 0xAF;
static const uint8_t PN532_CMD_IN_DATA_EXCHANGE = 0x40;
static const uint8_t PN532_CMD_IN_LIST_PASSIVE  = 0x4A;

static const uint8_t PN532_BUF_SIZE = 64;

// Cooldowns (ms)
static const uint16_t COOLDOWN_SUCCESS_MS   = 500;
static const uint16_t COOLDOWN_FAIL_BASE_MS = 200;
static const uint32_t COOLDOWN_FAIL_MAX_MS  = 30000;

// Forward declarations of standalone AES helpers (defined in .cpp)
void aes_key_exp_(const uint8_t *key, uint8_t *rk);
void aes_enc_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out);
void aes_dec_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out);

// ── State machine states ──
enum class NfcState : uint8_t {
  IDLE,              // waiting for cooldown
  DETECT_SEND,       // send InListPassiveTarget
  DETECT_READ,       // read detection response
  SELECT_SEND,       // send SelectApplication
  SELECT_READ,       // read SelectApplication response
  AUTH1_SEND,        // send auth phase 1
  AUTH1_READ,        // read auth phase 1 response
  AUTH2_SEND,        // send auth phase 2
  AUTH2_READ,        // read auth phase 2 response
  READ_SEND,         // send ReadData
  READ_READ,         // read ReadData response
  PUBLISH,           // decrypt and publish results
};

class DesfireReaderComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_app_id(uint8_t a, uint8_t b, uint8_t c) {
    app_id_[0] = a; app_id_[1] = b; app_id_[2] = c;
  }
  void set_app_key(const uint8_t *key) { memcpy(app_key_, key, 16); }
  void set_data_key(const uint8_t *key) { memcpy(data_key_, key, 16); }

  // Overloads for vector (Python codegen sends std::vector)
  void set_app_key(const std::vector<uint8_t> &key) {
    if (key.size() < 16) return;
    for (int i = 0; i < 16; i++) app_key_[i] = key[i];
  }
  void set_data_key(const std::vector<uint8_t> &key) {
    if (key.size() < 16) return;
    for (int i = 0; i < 16; i++) data_key_[i] = key[i];
  }

  // Legacy no-op stubs
  void set_aes_key(const std::vector<uint8_t> &key) { set_data_key(key); }
  void set_des_key(const std::vector<uint8_t> &) {}

  void set_result_sensor(text_sensor::TextSensor *s)    { result_sensor_ = s; }
  void set_auth_sensor(binary_sensor::BinarySensor *s)  { auth_sensor_ = s; }
  void set_uid_sensor(text_sensor::TextSensor *s)       { uid_sensor_ = s; }

 protected:
  // ── PN532 I2C frame layer (non-blocking) ──
  bool write_command_(const uint8_t *cmd, uint8_t cmd_len);
  bool try_read_ack_();    // non-blocking: returns true if ACK received, false if not ready
  bool try_read_response_(uint8_t command, uint8_t *resp, uint8_t resp_cap,
                          uint8_t &resp_len);  // non-blocking single attempt

  // ── DESFire operations (send/receive split) ──
  bool send_desfire_apdu_(const uint8_t *apdu, uint8_t apdu_len);
  bool read_desfire_apdu_(uint8_t *response, uint8_t resp_cap,
                          uint8_t &resp_len, uint8_t &sw1, uint8_t &sw2);

  bool aes_cbc_decrypt_(const uint8_t *in, uint8_t len,
                        const uint8_t *iv, uint8_t *out);
  void random_bytes_(uint8_t *buf, uint8_t len);

  // ── Security helpers ──
  static void secure_zero_(volatile uint8_t *buf, uint8_t len);

  // ── Publish helpers (only when value changes) ──
  void format_uid_(const uint8_t *uid_bytes, uint8_t uid_len, char *out);
  void publish_uid_(const char *uid_str);
  void publish_auth_(bool state);
  void publish_result_(const char *str);

  // ── State machine helpers ──
  void handle_fail_();
  void reset_to_idle_(uint32_t cooldown_ms);

  // ── Config ──
  uint8_t app_id_[3]{0xA1, 0xB2, 0xC3};
  uint8_t app_key_[16]{};
  uint8_t data_key_[16]{};
  uint8_t app_rk_[176]{};
  uint8_t data_rk_[176]{};

  // ── Sensors ──
  text_sensor::TextSensor     *result_sensor_{nullptr};
  text_sensor::TextSensor     *uid_sensor_{nullptr};
  binary_sensor::BinarySensor *auth_sensor_{nullptr};

  // ── Publish-only-when-changed cache ──
  char     last_uid_[24]{};
  char     last_result_[50]{};
  bool     last_auth_{false};

  // ── State machine ──
  NfcState state_{NfcState::IDLE};
  uint32_t cooldown_until_{0};
  uint32_t state_entered_at_{0};     // millis() when we entered current state
  uint8_t  poll_attempts_{0};        // retry counter for read states
  bool     ack_received_{false};     // whether ACK has been received for current command

  // ── Card tracking ──
  uint8_t  prev_uid_[7]{};
  uint8_t  prev_uid_len_{0};
  bool     card_present_{false};
  uint8_t  no_card_count_{0};
  uint8_t  consecutive_fails_{0};

  // ── Auth state carried between states ──
  char     current_uid_str_[24]{};
  uint8_t  current_uid_bytes_[7]{};
  uint8_t  current_uid_len_{0};

  // Auth handshake buffers (persisted across update() calls)
  uint8_t  enc_rnd_b_[16]{};
  uint8_t  rnd_a_[16]{};
  uint8_t  token_[32]{};

  // Raw file data buffer
  uint8_t  raw_file_[48]{};
  uint8_t  raw_file_len_{0};

  // ── Timeouts ──
  static const uint16_t READ_TIMEOUT_MS  = 300;   // max time to wait for a response
  static const uint8_t  MAX_READ_POLLS   = 80;    // max poll attempts per read
};

}  // namespace desfire_reader
}  // namespace esphome