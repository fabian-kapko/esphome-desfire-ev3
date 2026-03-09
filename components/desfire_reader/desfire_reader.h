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
static const uint16_t COOLDOWN_SUCCESS_MS = 500;
static const uint16_t COOLDOWN_FAIL_MS    = 200;

// Forward declarations of standalone AES helpers (defined in .cpp)
void aes_key_exp_(const uint8_t *key, uint8_t *rk);
void aes_enc_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out);
void aes_dec_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out);

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
    for (int i = 0; i < 16; i++) app_key_[i] = key[i];
  }
  void set_data_key(const std::vector<uint8_t> &key) {
    for (int i = 0; i < 16; i++) data_key_[i] = key[i];
  }

  // Legacy no-op stubs
  void set_aes_key(const std::vector<uint8_t> &key) { set_data_key(key); }
  void set_des_key(const std::vector<uint8_t> &) {}

  void set_result_sensor(text_sensor::TextSensor *s)    { result_sensor_ = s; }
  void set_auth_sensor(binary_sensor::BinarySensor *s)  { auth_sensor_ = s; }
  void set_uid_sensor(text_sensor::TextSensor *s)       { uid_sensor_ = s; }

 protected:
  // ── PN532 I2C frame layer ──
  bool write_command_(const uint8_t *cmd, uint8_t cmd_len);
  // max_polls × 3 ms = worst-case blocking time for this call.
  // Use ~20 for detect (~60 ms), ~80 for DESFire APDU (~240 ms).
  bool read_response_(uint8_t command, uint8_t *resp, uint8_t resp_cap,
                      uint8_t &resp_len, uint8_t max_polls = 80);

  // ── DESFire operations (all fixed-size stack buffers) ──
  bool desfire_apdu_(const uint8_t *apdu, uint8_t apdu_len,
                     uint8_t *response, uint8_t resp_cap,
                     uint8_t &resp_len, uint8_t &sw1, uint8_t &sw2);
  bool df_select_app_();
  bool df_auth_aes_();
  bool df_read_file_(uint8_t file_id, uint8_t length,
                     uint8_t *out, uint8_t &out_len);

  bool aes_cbc_decrypt_(const uint8_t *in, uint8_t len, uint8_t *out);
  void random_bytes_(uint8_t *buf, uint8_t len);

  // ── Publish helpers (only when value changes) ──
  void format_uid_(const uint8_t *uid_bytes, uint8_t uid_len, char *out);
  void publish_uid_(const char *uid_str);
  void publish_auth_(bool state);
  void publish_result_(const char *str);

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
  char     last_result_[18]{};
  bool     last_auth_{false};

  // ── Cooldown + UID dedup ──
  uint32_t cooldown_until_{0};
  uint8_t  prev_uid_[7]{};
  uint8_t  prev_uid_len_{0};
  bool     card_present_{false};  // true while card data is published
  uint8_t  no_card_count_{0};    // consecutive no-card detects (debounce)
};

}  // namespace desfire_reader
}  // namespace esphome