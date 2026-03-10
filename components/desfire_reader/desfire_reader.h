#pragma once

#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/pn532_i2c/pn532_i2c.h"

#include "mbedtls/aes.h"

namespace esphome {
namespace desfire_reader {

class DesfireReader : public Component {

 public:

  void setup() override;
  void loop() override;

  void set_pn532(esphome::pn532_i2c::PN532I2C *pn532) { this->pn532_ = pn532; }

  void set_app_id(uint32_t id) { this->app_id_ = id; }

  void set_app_key(const std::vector<uint8_t> &key);

  void set_file_id(uint8_t id) { this->file_id_ = id; }
  void set_file_offset(uint32_t o) { this->file_offset_ = o; }
  void set_max_payload_len(uint16_t l) { this->max_payload_len_ = l; }

  void set_result_sensor(text_sensor::TextSensor *s) { result_sensor_ = s; }
  void set_uid_sensor(text_sensor::TextSensor *s) { uid_sensor_ = s; }
  void set_auth_sensor(binary_sensor::BinarySensor *s) { auth_sensor_ = s; }

 protected:

  esphome::pn532_i2c::PN532I2C *pn532_{nullptr};

  text_sensor::TextSensor *result_sensor_{nullptr};
  text_sensor::TextSensor *uid_sensor_{nullptr};
  binary_sensor::BinarySensor *auth_sensor_{nullptr};

  uint32_t app_id_{0};

  uint8_t file_id_{1};
  uint32_t file_offset_{0};
  uint16_t max_payload_len_{32};

  uint8_t app_key_[16];

  uint8_t session_key_[16];
  uint8_t iv_[16];

  bool authenticate_();
  bool select_application_();
  bool read_file_secure_(std::vector<uint8_t> &out);

  bool validate_payload_(const std::vector<uint8_t> &data, std::string &out);

  uint32_t crc32_(const uint8_t *data, size_t len);

  void aes_encrypt_(uint8_t *data, uint8_t *key);
  void aes_decrypt_(uint8_t *data, uint8_t *key);
};

}  
}