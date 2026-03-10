#include "desfire_reader.h"
#include "esp_system.h"

namespace esphome {
namespace desfire_reader {

static const char *TAG = "desfire_reader";

void DesfireReader::setup() {
  memset(iv_, 0, 16);
}

void DesfireReader::set_app_key(const std::vector<uint8_t> &key) {
  memcpy(app_key_, key.data(), 16);
}

void DesfireReader::loop() {

  uint8_t uid[10];
  uint8_t uid_len;

  if (!pn532_->read_passive_target_id(PN532_MIFARE_ISO14443A, uid, &uid_len))
    return;

  std::string uid_str;
  for (int i = 0; i < uid_len; i++) {
    char buf[4];
    sprintf(buf, "%02X", uid[i]);
    uid_str += buf;
  }

  if (uid_sensor_) uid_sensor_->publish_state(uid_str);

  if (!select_application_()) {
    if (auth_sensor_) auth_sensor_->publish_state(false);
    return;
  }

  if (!authenticate_()) {
    if (auth_sensor_) auth_sensor_->publish_state(false);
    return;
  }

  std::vector<uint8_t> file_data;

  if (!read_file_secure_(file_data)) {
    if (auth_sensor_) auth_sensor_->publish_state(false);
    return;
  }

  std::string parsed;

  if (!validate_payload_(file_data, parsed)) {
    if (auth_sensor_) auth_sensor_->publish_state(false);
    return;
  }

  if (result_sensor_) result_sensor_->publish_state(parsed);

  if (auth_sensor_) auth_sensor_->publish_state(true);
}

bool DesfireReader::select_application_() {

  uint8_t cmd[4];

  cmd[0] = 0x5A;
  cmd[1] = (app_id_) & 0xFF;
  cmd[2] = (app_id_ >> 8) & 0xFF;
  cmd[3] = (app_id_ >> 16) & 0xFF;

  return pn532_->in_data_exchange(cmd, 4, nullptr, nullptr);
}

bool DesfireReader::authenticate_() {

  uint8_t rndA[16];
  esp_fill_random(rndA, 16);

  uint8_t cmd[2] = {0xAA, 0x00};

  uint8_t resp[32];
  uint8_t len;

  if (!pn532_->in_data_exchange(cmd, 2, resp, &len))
    return false;

  aes_decrypt_(resp, app_key_);

  uint8_t rndB[16];
  memcpy(rndB, resp, 16);

  uint8_t rndB_rot[16];
  for (int i=0;i<16;i++)
    rndB_rot[i] = rndB[(i+1)%16];

  uint8_t buf[32];
  memcpy(buf, rndA, 16);
  memcpy(buf+16, rndB_rot, 16);

  aes_encrypt_(buf, app_key_);

  if (!pn532_->in_data_exchange(buf, 32, resp, &len))
    return false;

  aes_decrypt_(resp, app_key_);

  for (int i=0;i<15;i++) {
    if (resp[i] != rndA[i+1])
      return false;
  }

  memcpy(session_key_, rndA, 4);
  memcpy(session_key_+4, rndB, 4);
  memcpy(session_key_+8, rndA+12, 4);
  memcpy(session_key_+12, rndB+12, 4);

  memset(iv_,0,16);

  return true;
}

bool DesfireReader::read_file_secure_(std::vector<uint8_t> &out) {

  uint8_t cmd[8];

  cmd[0] = 0xBD;
  cmd[1] = file_id_;

  cmd[2] = file_offset_ & 0xFF;
  cmd[3] = (file_offset_ >> 8) & 0xFF;
  cmd[4] = (file_offset_ >> 16) & 0xFF;

  cmd[5] = max_payload_len_ & 0xFF;
  cmd[6] = (max_payload_len_ >> 8) & 0xFF;
  cmd[7] = (max_payload_len_ >> 16) & 0xFF;

  uint8_t resp[128];
  uint8_t len;

  if (!pn532_->in_data_exchange(cmd,8,resp,&len))
    return false;

  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_dec(&aes, session_key_, 128);

  uint8_t iv[16];
  memcpy(iv,iv_,16);

  uint8_t outbuf[128];

  mbedtls_aes_crypt_cbc(&aes,MBEDTLS_AES_DECRYPT,len,iv,resp,outbuf);

  out.assign(outbuf,outbuf+len);

  return true;
}

bool DesfireReader::validate_payload_(const std::vector<uint8_t> &data, std::string &out) {

  if (data.size() < 6)
    return false;

  uint8_t version = data[0];

  if (version != 1)
    return false;

  uint8_t len = data[1];

  if (len + 6 > data.size())
    return false;

  uint32_t crc_card =
    data[2+len] |
    (data[3+len] << 8) |
    (data[4+len] << 16) |
    (data[5+len] << 24);

  uint32_t crc_calc = crc32_(data.data(),2+len);

  if (crc_card != crc_calc)
    return false;

  out = std::string((char*)&data[2], len);

  return true;
}

uint32_t DesfireReader::crc32_(const uint8_t *data, size_t len) {

  uint32_t crc = 0xFFFFFFFF;

  for (size_t i=0;i<len;i++) {
    crc ^= data[i];
    for (int j=0;j<8;j++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xEDB88320;
      else
        crc >>= 1;
    }
  }

  return ~crc;
}

void DesfireReader::aes_encrypt_(uint8_t *data, uint8_t *key) {

  mbedtls_aes_context aes;

  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes,key,128);

  uint8_t iv[16];
  memset(iv,0,16);

  mbedtls_aes_crypt_cbc(&aes,MBEDTLS_AES_ENCRYPT,32,iv,data,data);
}

void DesfireReader::aes_decrypt_(uint8_t *data, uint8_t *key) {

  mbedtls_aes_context aes;

  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_dec(&aes,key,128);

  uint8_t iv[16];
  memset(iv,0,16);

  mbedtls_aes_crypt_cbc(&aes,MBEDTLS_AES_DECRYPT,16,iv,data,data);
}

}
}