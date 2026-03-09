#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <vector>

namespace esphome {
namespace desfire_reader {

static const char *const TAG = "desfire";

static const uint8_t DESFIRE_SW1 = 0x91;
static const uint8_t DESFIRE_OK = 0x00;
static const uint8_t DESFIRE_MORE_FRAMES = 0xAF;

static const uint8_t PN532_CMD_SAM_CONFIGURATION = 0x14;
static const uint8_t PN532_CMD_IN_DATA_EXCHANGE = 0x40;
static const uint8_t PN532_CMD_IN_LIST_PASSIVE = 0x4A;

static const size_t PN532_MAX_FRAME = 96;
static const size_t PN532_MAX_PAYLOAD = PN532_MAX_FRAME - 8;
static const size_t DESFIRE_MAX_APDU = 40;
static const size_t DESFIRE_MAX_RESP = 32;
static const size_t UID_STR_LEN = 3 * 10;
static const size_t RESULT_STR_LEN = 17;

void aes_key_exp_(const uint8_t *key, uint8_t *rk);
void aes_enc_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out);
void aes_dec_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out);

class DesfireReaderComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_app_id(uint8_t a, uint8_t b, uint8_t c) {
    app_id_[0] = a;
    app_id_[1] = b;
    app_id_[2] = c;
  }
  void set_app_key(const std::vector<uint8_t> &key) {
    for (int i = 0; i < 16 && i < (int) key.size(); i++)
      app_key_[i] = key[i];
  }
  void set_data_key(const std::vector<uint8_t> &key) {
    for (int i = 0; i < 16 && i < (int) key.size(); i++)
      data_key_[i] = key[i];
  }
  void set_reread_cooldown_ms(uint32_t cooldown_ms) { reread_cooldown_ms_ = cooldown_ms; }

  void set_aes_key(const std::vector<uint8_t> &key) { set_data_key(key); }
  void set_des_key(const std::vector<uint8_t> &) {}

  void set_result_sensor(text_sensor::TextSensor *s) { result_sensor_ = s; }
  void set_auth_sensor(binary_sensor::BinarySensor *s) { auth_sensor_ = s; }
  void set_uid_sensor(text_sensor::TextSensor *s) { uid_sensor_ = s; }

 protected:
  bool desfire_apdu_(const uint8_t *apdu, size_t apdu_len, uint8_t *response, size_t response_max,
                     size_t &response_len, uint8_t &sw1, uint8_t &sw2);
  bool df_select_picc_();
  bool df_select_app_();
  bool df_auth_aes_(const uint8_t *key);
  bool df_read_file_(uint8_t file_id, size_t length, uint8_t *out, size_t &out_len);

  bool aes_cbc_decrypt_(const uint8_t *in, size_t len, uint8_t *out);
  void random_bytes_(uint8_t *buf, size_t len);
  void clear_card_state_(bool clear_sensors);

  bool write_command_(const uint8_t *cmd, size_t cmd_len);
  bool read_response_(uint8_t command, uint8_t *resp, size_t resp_max, size_t &resp_len,
                      bool count_timeout_as_error = true);
  bool read_ack_();
  bool pn532_wakeup_();
  bool pn532_init_();
  bool maybe_recover_pn532_();
  void reset_bus_error_state_();
  void on_bus_error_();
  bool uid_equals_last_(const char *uid) const;
  void copy_last_uid_(const char *uid);
  void format_uid_(const uint8_t *uid, uint8_t uid_len, char *out, size_t out_len);
  bool publish_if_changed_(text_sensor::TextSensor *sensor, const char *value,
                           char *last_value, size_t last_size);
  bool publish_if_changed_(binary_sensor::BinarySensor *sensor, bool value,
                           bool &last_value, bool &last_valid);
  void mark_attempt_(const char *uid, uint32_t now, bool success);
  bool should_skip_uid_(const char *uid, uint32_t now) const;

  uint8_t app_id_[3]{0xA1, 0xB2, 0xC3};
  uint8_t app_key_[16]{};
  uint8_t data_key_[16]{};
  uint8_t app_rk_[176]{};
  uint8_t data_rk_[176]{};

  text_sensor::TextSensor *result_sensor_{nullptr};
  text_sensor::TextSensor *uid_sensor_{nullptr};
  binary_sensor::BinarySensor *auth_sensor_{nullptr};

  char last_uid_[UID_STR_LEN]{};
  char last_uid_published_[UID_STR_LEN]{};
  char last_result_published_[RESULT_STR_LEN]{};
  bool last_auth_published_{false};
  bool last_auth_valid_{false};

  uint32_t last_read_ms_{0};
  uint32_t reread_cooldown_ms_{1500};
  uint32_t failed_retry_ms_{250};
  uint8_t no_card_count_{0};
  uint8_t consecutive_bus_errors_{0};
  uint32_t last_recover_ms_{0};
  bool last_attempt_success_{false};
  bool pn532_ready_{false};

  uint8_t tx_frame_[PN532_MAX_FRAME]{};
  uint8_t rx_frame_[PN532_MAX_FRAME]{};
  uint8_t scratch_resp_[DESFIRE_MAX_RESP]{};
  uint8_t scratch_data_[DESFIRE_MAX_RESP]{};
};

}  // namespace desfire_reader
}  // namespace esphome