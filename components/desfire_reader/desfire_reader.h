#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <string.h>

#ifdef USE_ESP32
#include <esp_random.h>
#endif

namespace esphome {
namespace desfire_reader {

static const char *const TAG = "desfire";

static const uint8_t DESFIRE_SW1         = 0x91;
static const uint8_t DESFIRE_OK          = 0x00;
static const uint8_t DESFIRE_MORE_FRAMES = 0xAF;
static const uint8_t PN532_CMD_IN_DATA_EXCHANGE = 0x40;
static const uint8_t PN532_CMD_IN_LIST_PASSIVE  = 0x4A;
static const uint8_t PN532_CMD_IN_RELEASE       = 0x52;

static const uint8_t PN532_BUF_SIZE = 64;

// Cooldowns (ms)
static const uint16_t COOLDOWN_SUCCESS_MS   = 150;
static const uint16_t COOLDOWN_RETRY_MS     = 30;    // retry after transient fail
static const uint16_t COOLDOWN_FAIL_BASE_MS = 100;
static const uint32_t COOLDOWN_FAIL_MAX_MS  = 5000;
static const uint8_t  NO_CARD_THRESHOLD     = 2;
static const uint8_t  MAX_RETRIES           = 2;     // retry auth this many times before giving up

// Forward declarations of standalone AES helpers (defined in .cpp)
void aes_key_exp_(const uint8_t *key, uint8_t *rk);
void aes_enc_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out);
void aes_dec_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out);

// ── State machine states ──
enum class NfcState : uint8_t {
  IDLE,
  DETECT_WAIT_ACK,
  DETECT_WAIT_RESP,
  SELECT_WAIT_ACK,
  SELECT_WAIT_RESP,
  AUTH1_WAIT_ACK,
  AUTH1_WAIT_RESP,
  AUTH2_WAIT_ACK,
  AUTH2_WAIT_RESP,
  READ_WAIT_ACK,
  READ_WAIT_RESP,
  PUBLISH,
  RELEASE_WAIT_ACK,     // release target after fail, then re-detect
  RELEASE_WAIT_RESP,
};

class DesfireReaderComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_app_id(uint8_t a, uint8_t b, uint8_t c) {
    app_id_[0] = a; app_id_[1] = b; app_id_[2] = c;
  }
  void set_app_key(const uint8_t *key) { memcpy(app_key_, key, 16); }
  void set_data_key(const uint8_t *key) { memcpy(data_key_, key, 16); }

  void set_app_key(const std::vector<uint8_t> &key) {
    if (key.size() < 16) return;
    for (int i = 0; i < 16; i++) app_key_[i] = key[i];
  }
  void set_data_key(const std::vector<uint8_t> &key) {
    if (key.size() < 16) return;
    for (int i = 0; i < 16; i++) data_key_[i] = key[i];
  }

  void set_aes_key(const std::vector<uint8_t> &key) { set_data_key(key); }
  void set_des_key(const std::vector<uint8_t> &) {}

  void set_result_sensor(text_sensor::TextSensor *s)    { result_sensor_ = s; }
  void set_auth_sensor(binary_sensor::BinarySensor *s)  { auth_sensor_ = s; }
  void set_uid_sensor(text_sensor::TextSensor *s)       { uid_sensor_ = s; }

 protected:
  bool write_command_(const uint8_t *cmd, uint8_t cmd_len);
  bool try_read_ack_();
  bool try_read_response_(uint8_t command, uint8_t *resp, uint8_t resp_cap,
                          uint8_t &resp_len);

  bool send_desfire_apdu_(const uint8_t *apdu, uint8_t apdu_len);
  bool read_desfire_apdu_(uint8_t *response, uint8_t resp_cap,
                          uint8_t &resp_len, uint8_t &sw1, uint8_t &sw2);

  bool aes_cbc_decrypt_(const uint8_t *in, uint8_t len,
                        const uint8_t *iv, uint8_t *out);
  void random_bytes_(uint8_t *buf, uint8_t len);

  static void secure_zero_(volatile uint8_t *buf, uint8_t len);

  void format_uid_(const uint8_t *uid_bytes, uint8_t uid_len, char *out);
  void publish_uid_(const char *uid_str);
  void publish_auth_(bool state);
  void publish_result_(const char *str);

  void handle_fail_();          // hard fail — give up on this card
  void handle_retry_();         // transient fail — release target and retry
  void reset_to_idle_(uint32_t cooldown_ms);
  void clear_card_state_();
  void start_release_();        // send InRelease to reset PN532 target state
  void start_select_app_();     // begin SelectApp (reused for retries)

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

  // ── Publish cache ──
  char     last_uid_[24]{};
  char     last_result_[50]{};
  bool     last_auth_{false};

  // ── State machine ──
  NfcState state_{NfcState::IDLE};
  uint32_t cooldown_until_{0};
  uint32_t state_entered_at_{0};
  bool     update_requested_{false};

  // ── Card tracking ──
  uint8_t  prev_uid_[7]{};
  uint8_t  prev_uid_len_{0};
  bool     card_present_{false};
  uint8_t  no_card_count_{0};
  uint8_t  consecutive_fails_{0};
  uint8_t  retry_count_{0};          // retries for current card

  // ── Carried between states ──
  char     current_uid_str_[24]{};
  uint8_t  current_uid_bytes_[7]{};
  uint8_t  current_uid_len_{0};

  uint8_t  enc_rnd_b_[16]{};
  uint8_t  rnd_a_[16]{};
  uint8_t  token_[32]{};

  uint8_t  raw_file_[48]{};
  uint8_t  raw_file_len_{0};

  // ── Timeouts ──
  static const uint16_t ACK_TIMEOUT_MS    = 50;
  static const uint16_t RESP_TIMEOUT_MS   = 150;
  static const uint16_t DETECT_TIMEOUT_MS = 200;
};

}  // namespace desfire_reader
}  // namespace esphome