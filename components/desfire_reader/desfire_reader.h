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

// Max PN532 I2C response frame:
//   1 status + 1 preamble + 2 start + 1 LEN + 1 LCS
//   + 1 TFI + 1 CMD + up to 52 payload + 1 DCS + 1 postamble = ~62
// 64 is tight but sufficient for DESFire single-frame exchanges.
static const uint8_t PN532_BUF_SIZE = 64;

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
  void set_app_key(const uint8_t *key) {
    memcpy(app_key_, key, 16);
  }
  void set_data_key(const uint8_t *key) {
    memcpy(data_key_, key, 16);
  }

  // Overloads for vector (Python codegen sends std::vector)
  void set_app_key(const std::vector<uint8_t> &key) {
    for (int i = 0; i < 16; i++) app_key_[i] = key[i];
  }
  void set_data_key(const std::vector<uint8_t> &key) {
    for (int i = 0; i < 16; i++) data_key_[i] = key[i];
  }

  // Legacy no-op stubs (backward compat)
  void set_aes_key(const std::vector<uint8_t> &key) { set_data_key(key); }
  void set_des_key(const std::vector<uint8_t> &) {}

  void set_result_sensor(text_sensor::TextSensor *s)    { result_sensor_ = s; }
  void set_auth_sensor(binary_sensor::BinarySensor *s)  { auth_sensor_ = s; }
  void set_uid_sensor(text_sensor::TextSensor *s)       { uid_sensor_ = s; }

 protected:
  // ── PN532 I2C frame layer ──
  // cmd/cmd_len: raw PN532 command bytes (without TFI framing).
  // resp/resp_cap: caller-owned buffer for payload (after TFI+CMD stripped).
  // resp_len: actual payload length written.
  bool write_command_(const uint8_t *cmd, uint8_t cmd_len);
  bool read_response_(uint8_t command, uint8_t *resp, uint8_t resp_cap,
                      uint8_t &resp_len);

  // ── DESFire operations ──
  // All use fixed-size stack buffers internally.
  bool desfire_apdu_(const uint8_t *apdu, uint8_t apdu_len,
                     uint8_t *response, uint8_t resp_cap,
                     uint8_t &resp_len, uint8_t &sw1, uint8_t &sw2);
  bool df_select_app_();
  bool df_auth_aes_(const uint8_t *key);
  bool df_read_file_(uint8_t file_id, uint8_t length,
                     uint8_t *out, uint8_t &out_len);

  bool aes_cbc_decrypt_(const uint8_t *in, size_t len, uint8_t *out);
  void random_bytes_(uint8_t *buf, size_t len);

  // ── Helpers ──
  void publish_card_removed_();
  void publish_uid_(const uint8_t *uid_bytes, uint8_t uid_len);
  void publish_auth_(bool state);
  void publish_result_(const char *str);

  // ── Config ──
  uint8_t app_id_[3]{0xA1, 0xB2, 0xC3};
  uint8_t app_key_[16]{};    // AES-128 key for app authentication
  uint8_t data_key_[16]{};   // AES-128 key to decrypt file contents
  uint8_t app_rk_[176]{};    // precomputed round keys for app_key_
  uint8_t data_rk_[176]{};   // precomputed round keys for data_key_

  // ── Sensors ──
  text_sensor::TextSensor     *result_sensor_{nullptr};
  text_sensor::TextSensor     *uid_sensor_{nullptr};
  binary_sensor::BinarySensor *auth_sensor_{nullptr};

  // ── Publish-only-when-changed state ──
  char     last_uid_[24]{};     // "XX:XX:XX:XX:XX:XX:XX\0" max 7-byte UID
  char     last_result_[18]{};  // 16 printable chars + NUL
  bool     last_auth_{false};

  // ── Card presence state ──
  bool     last_card_state_{false};
  uint8_t  no_card_count_{0};
};

}  // namespace desfire_reader
}  // namespace esphome