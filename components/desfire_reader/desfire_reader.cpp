
#include "desfire_reader.h"
#include "esphome/core/hal.h"
#include <stdio.h>
#include <string.h>

namespace esphome {
namespace desfire_reader {

static bool safe_str_copy_(char *dst, size_t dst_len, const char *src) {
  if (dst_len == 0)
    return false;
  size_t n = strlen(src);
  if (n >= dst_len)
    n = dst_len - 1;
  bool changed = (strncmp(dst, src, dst_len) != 0);
  memcpy(dst, src, n);
  dst[n] = '\0';
  return changed;
}

bool DesfireReaderComponent::publish_if_changed_(text_sensor::TextSensor *sensor, const char *value,
                                                 char *last_value, size_t last_size) {
  if (sensor == nullptr)
    return false;
  if (strncmp(last_value, value, last_size) == 0)
    return false;
  safe_str_copy_(last_value, last_size, value);
  sensor->publish_state(value);
  return true;
}

bool DesfireReaderComponent::publish_if_changed_(binary_sensor::BinarySensor *sensor, bool value,
                                                 bool &last_value, bool &last_valid) {
  if (sensor == nullptr)
    return false;
  if (last_valid && last_value == value)
    return false;
  last_value = value;
  last_valid = true;
  sensor->publish_state(value);
  return true;
}

void DesfireReaderComponent::format_uid_(const uint8_t *uid, uint8_t uid_len, char *out, size_t out_len) {
  if (out_len == 0)
    return;
  out[0] = '\0';
  size_t pos = 0;
  for (uint8_t i = 0; i < uid_len; i++) {
    if (i > 0) {
      if (pos + 1 >= out_len)
        break;
      out[pos++] = ':';
    }
    if (pos + 2 >= out_len)
      break;
    snprintf(&out[pos], out_len - pos, "%02X", uid[i]);
    pos += 2;
  }
  if (pos >= out_len)
    pos = out_len - 1;
  out[pos] = '\0';
}

bool DesfireReaderComponent::uid_equals_last_(const char *uid) const {
  return strncmp(last_uid_, uid, sizeof(last_uid_)) == 0;
}

void DesfireReaderComponent::copy_last_uid_(const char *uid) { safe_str_copy_(last_uid_, sizeof(last_uid_), uid); }
void DesfireReaderComponent::reset_bus_error_state_() { consecutive_bus_errors_ = 0; }
void DesfireReaderComponent::on_bus_error_() {
  if (consecutive_bus_errors_ < 255)
    consecutive_bus_errors_++;
}

bool DesfireReaderComponent::pn532_wakeup_() {
  static const uint8_t wakeup[] = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  return this->write(wakeup, sizeof(wakeup)) == i2c::ERROR_OK;
}

bool DesfireReaderComponent::read_ack_() {
  for (uint8_t retry = 0; retry < 20; retry++) {
    uint8_t ack[7] = {};
    if (this->read(ack, sizeof(ack)) == i2c::ERROR_OK) {
      if (ack[0] != 0x01) {
        delay(2);
        continue;
      }
      if (ack[1] == 0x00 && ack[2] == 0x00 && ack[3] == 0xFF && ack[4] == 0x00 && ack[5] == 0xFF && ack[6] == 0x00)
        return true;
      on_bus_error_();
      return false;
    }
    delay(2);
  }
  on_bus_error_();
  return false;
}

bool DesfireReaderComponent::write_command_(const uint8_t *cmd, size_t cmd_len) {
  if (cmd_len == 0 || cmd_len > PN532_MAX_PAYLOAD)
    return false;

  const uint8_t len = static_cast<uint8_t>(cmd_len + 1);
  const uint8_t lcs = static_cast<uint8_t>(0x100u - len);
  uint8_t dcs = 0xD4;
  for (size_t i = 0; i < cmd_len; i++)
    dcs += cmd[i];
  dcs = static_cast<uint8_t>(0x100u - dcs);

  const size_t frame_len = cmd_len + 8;
  if (frame_len > sizeof(tx_frame_))
    return false;

  tx_frame_[0] = 0x00;
  tx_frame_[1] = 0x00;
  tx_frame_[2] = 0xFF;
  tx_frame_[3] = len;
  tx_frame_[4] = lcs;
  tx_frame_[5] = 0xD4;
  memcpy(&tx_frame_[6], cmd, cmd_len);
  tx_frame_[6 + cmd_len] = dcs;
  tx_frame_[7 + cmd_len] = 0x00;

  if (this->write(tx_frame_, frame_len) != i2c::ERROR_OK) {
    on_bus_error_();
    return false;
  }
  delay(2);
  return this->read_ack_();
}

bool DesfireReaderComponent::read_response_(uint8_t command, uint8_t *resp, size_t resp_max, size_t &resp_len,
                                            bool count_timeout_as_error) {
  resp_len = 0;
  for (uint8_t retry = 0; retry < 40; retry++) {
    memset(rx_frame_, 0, sizeof(rx_frame_));
    if (this->read(rx_frame_, sizeof(rx_frame_)) != i2c::ERROR_OK) {
      delay(2);
      continue;
    }
    if (rx_frame_[0] != 0x01) {
      delay(2);
      continue;
    }
    if (rx_frame_[1] != 0x00 || rx_frame_[2] != 0x00 || rx_frame_[3] != 0xFF) {
      on_bus_error_();
      return false;
    }

    const uint8_t len = rx_frame_[4];
    const uint8_t lcs = rx_frame_[5];
    if ((uint8_t) (len + lcs) != 0x00 || len < 2) {
      on_bus_error_();
      return false;
    }

    const size_t needed = static_cast<size_t>(len) + 8;
    if (needed > sizeof(rx_frame_)) {
      on_bus_error_();
      return false;
    }

    const uint8_t tfi = rx_frame_[6];
    const uint8_t cmd = rx_frame_[7];
    if (tfi != 0xD5 || cmd != static_cast<uint8_t>(command + 1)) {
      on_bus_error_();
      return false;
    }

    uint8_t dcs = 0;
    for (size_t i = 6; i < 6 + len; i++)
      dcs += rx_frame_[i];
    dcs += rx_frame_[6 + len];
    if (dcs != 0x00 || rx_frame_[6 + len + 1] != 0x00) {
      on_bus_error_();
      return false;
    }

    const size_t payload_len = static_cast<size_t>(len) - 2;
    if (payload_len > resp_max) {
      on_bus_error_();
      return false;
    }

    if (payload_len > 0)
      memcpy(resp, &rx_frame_[8], payload_len);
    resp_len = payload_len;
    reset_bus_error_state_();
    return true;
  }
  if (count_timeout_as_error)
    on_bus_error_();
  return false;
}

bool DesfireReaderComponent::pn532_init_() {
  if (!pn532_wakeup_())
    return false;
  delay(10);
  static const uint8_t sam_cmd[] = {PN532_CMD_SAM_CONFIGURATION, 0x01, 0x14, 0x00};
  size_t resp_len = 0;
  if (!write_command_(sam_cmd, sizeof(sam_cmd)))
    return false;
  if (!read_response_(PN532_CMD_SAM_CONFIGURATION, scratch_resp_, sizeof(scratch_resp_), resp_len))
    return false;
  pn532_ready_ = true;
  reset_bus_error_state_();
  return true;
}

bool DesfireReaderComponent::maybe_recover_pn532_() {
  const uint32_t now = millis();
  if (consecutive_bus_errors_ < 3)
    return pn532_ready_;
  if ((now - last_recover_ms_) < 2000)
    return false;
  last_recover_ms_ = now;
  ESP_LOGW(TAG, "Recovering PN532 after %u bus/protocol errors", consecutive_bus_errors_);
  pn532_ready_ = false;
  clear_card_state_(false);
  delay(5);
  return pn532_init_();
}

void DesfireReaderComponent::clear_card_state_(bool clear_sensors) {
  last_uid_[0] = '\0';
  last_read_ms_ = 0;
  no_card_count_ = 0;
  last_attempt_success_ = false;
  if (!clear_sensors)
    return;
  publish_if_changed_(auth_sensor_, false, last_auth_published_, last_auth_valid_);
  publish_if_changed_(result_sensor_, "", last_result_published_, sizeof(last_result_published_));
  publish_if_changed_(uid_sensor_, "", last_uid_published_, sizeof(last_uid_published_));
}

bool DesfireReaderComponent::should_skip_uid_(const char *uid, uint32_t now) const {
  if (!uid_equals_last_(uid))
    return false;
  const uint32_t cooldown = last_attempt_success_ ? reread_cooldown_ms_ : failed_retry_ms_;
  return (now - last_read_ms_) < cooldown;
}

void DesfireReaderComponent::mark_attempt_(const char *uid, uint32_t now, bool success) {
  copy_last_uid_(uid);
  last_read_ms_ = now;
  last_attempt_success_ = success;
}

void DesfireReaderComponent::setup() {
  ESP_LOGCONFIG(TAG, "DESFire reader setup...");
  randomSeed(analogRead(A0) ^ micros());
  aes_key_exp_(app_key_, app_rk_);
  aes_key_exp_(data_key_, data_rk_);

  if (!pn532_init_()) {
    ESP_LOGE(TAG, "PN532 init failed at I2C address 0x%02X", this->address_);
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "PN532 initialized.");
}

void DesfireReaderComponent::update() {
  if (this->is_failed())
    return;
  if (!pn532_ready_ && !pn532_init_())
    return;

  static const uint8_t detect_cmd[] = {PN532_CMD_IN_LIST_PASSIVE, 0x01, 0x00};
  size_t resp_len = 0;

  if (!write_command_(detect_cmd, sizeof(detect_cmd))) {
    maybe_recover_pn532_();
    return;
  }

  if (!read_response_(PN532_CMD_IN_LIST_PASSIVE, scratch_resp_, sizeof(scratch_resp_), resp_len, false) ||
      resp_len == 0 || scratch_resp_[0] == 0) {
    no_card_count_++;
    if (no_card_count_ > 1 && last_uid_[0] != '\0') {
      ESP_LOGD(TAG, "Card removed");
      clear_card_state_(true);
    }
    return;
  }

  no_card_count_ = 0;
  if (resp_len < 7)
    return;

  const uint8_t uid_len = scratch_resp_[5];
  if (uid_len == 0 || uid_len > 10)
    return;
  if (resp_len < static_cast<size_t>(6 + uid_len))
    return;

  char current_uid[UID_STR_LEN] = {};
  format_uid_(&scratch_resp_[6], uid_len, current_uid, sizeof(current_uid));

  const uint32_t now = millis();
  if (should_skip_uid_(current_uid, now))
    return;

  if (!df_select_app_()) {
    publish_if_changed_(auth_sensor_, false, last_auth_published_, last_auth_valid_);
    mark_attempt_(current_uid, now, false);
    maybe_recover_pn532_();
    return;
  }

  if (!df_auth_aes_(app_key_)) {
    publish_if_changed_(auth_sensor_, false, last_auth_published_, last_auth_valid_);
    mark_attempt_(current_uid, now, false);
    maybe_recover_pn532_();
    return;
  }

  size_t raw_len = 0;
  if (!df_read_file_(0x01, 16, scratch_data_, raw_len) || raw_len < 16) {
    publish_if_changed_(auth_sensor_, false, last_auth_published_, last_auth_valid_);
    mark_attempt_(current_uid, now, false);
    maybe_recover_pn532_();
    return;
  }

  uint8_t decrypted[16] = {};
  if (!aes_cbc_decrypt_(scratch_data_, 16, decrypted)) {
    publish_if_changed_(auth_sensor_, false, last_auth_published_, last_auth_valid_);
    mark_attempt_(current_uid, now, false);
    return;
  }

  char result[RESULT_STR_LEN] = {};
  uint8_t out = 0;
  for (uint8_t i = 0; i < 16 && decrypted[i] >= 0x20 && decrypted[i] <= 0x7E && out < RESULT_STR_LEN - 1; i++)
    result[out++] = static_cast<char>(decrypted[i]);
  result[out] = '\0';

  publish_if_changed_(uid_sensor_, current_uid, last_uid_published_, sizeof(last_uid_published_));
  publish_if_changed_(result_sensor_, result, last_result_published_, sizeof(last_result_published_));
  publish_if_changed_(auth_sensor_, true, last_auth_published_, last_auth_valid_);
  mark_attempt_(current_uid, now, true);
}

void DesfireReaderComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "DESFire Reader:");
  ESP_LOGCONFIG(TAG, "  App ID: %02X:%02X:%02X", app_id_[0], app_id_[1], app_id_[2]);
  ESP_LOGCONFIG(TAG, "  Re-read cooldown: %u ms", reread_cooldown_ms_);
  ESP_LOGCONFIG(TAG, "  Failed retry: %u ms", failed_retry_ms_);
  ESP_LOGCONFIG(TAG, "  Max frame: %u", static_cast<unsigned>(PN532_MAX_FRAME));
  LOG_I2C_DEVICE(this);
}

bool DesfireReaderComponent::desfire_apdu_(const uint8_t *apdu, size_t apdu_len, uint8_t *response,
                                           size_t response_max, size_t &response_len,
                                           uint8_t &sw1, uint8_t &sw2) {
  response_len = 0;
  sw1 = 0;
  sw2 = 0;
  if (apdu_len == 0 || apdu_len > DESFIRE_MAX_APDU - 2)
    return false;

  uint8_t cmd[DESFIRE_MAX_APDU + 2] = {};
  cmd[0] = PN532_CMD_IN_DATA_EXCHANGE;
  cmd[1] = 0x01;
  memcpy(&cmd[2], apdu, apdu_len);

  if (!write_command_(cmd, apdu_len + 2))
    return false;

  size_t pn_len = 0;
  if (!read_response_(PN532_CMD_IN_DATA_EXCHANGE, scratch_resp_, sizeof(scratch_resp_), pn_len))
    return false;
  if (pn_len < 3)
    return false;
  if (scratch_resp_[0] != 0x00)
    return false;

  sw1 = scratch_resp_[pn_len - 2];
  sw2 = scratch_resp_[pn_len - 1];
  response_len = pn_len - 3;
  if (response_len > response_max)
    return false;
  if (response_len > 0)
    memcpy(response, &scratch_resp_[1], response_len);
  return true;
}

bool DesfireReaderComponent::df_select_picc_() {
  const uint8_t apdu[9] = {0x90, 0x5A, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00};
  size_t resp_len = 0;
  uint8_t sw1 = 0, sw2 = 0;
  if (!desfire_apdu_(apdu, sizeof(apdu), scratch_resp_, sizeof(scratch_resp_), resp_len, sw1, sw2))
    return false;
  return sw1 == DESFIRE_SW1 && sw2 == DESFIRE_OK;
}

bool DesfireReaderComponent::df_select_app_() {
  const uint8_t apdu[9] = {0x90, 0x5A, 0x00, 0x00, 0x03, app_id_[0], app_id_[1], app_id_[2], 0x00};
  size_t resp_len = 0;
  uint8_t sw1 = 0, sw2 = 0;
  if (!desfire_apdu_(apdu, sizeof(apdu), scratch_resp_, sizeof(scratch_resp_), resp_len, sw1, sw2))
    return false;
  return sw1 == DESFIRE_SW1 && sw2 == DESFIRE_OK;
}

bool DesfireReaderComponent::df_auth_aes_(const uint8_t *key) {
  (void) key;
  const uint8_t apdu1[7] = {0x90, 0xAA, 0x00, 0x00, 0x01, 0x00, 0x00};
  uint8_t resp1[16] = {};
  size_t resp1_len = 0;
  uint8_t sw1 = 0, sw2 = 0;

  if (!desfire_apdu_(apdu1, sizeof(apdu1), resp1, sizeof(resp1), resp1_len, sw1, sw2))
    return false;
  if (sw2 != DESFIRE_MORE_FRAMES || resp1_len != 16)
    return false;

  uint8_t rnd_b[16], iv[16] = {0};
  aes_dec_block_(app_rk_, resp1, rnd_b);
  for (uint8_t i = 0; i < 16; i++)
    rnd_b[i] ^= iv[i];

  uint8_t rnd_a[16];
  random_bytes_(rnd_a, 16);
  uint8_t rnd_b_rot[16];
  for (uint8_t i = 0; i < 15; i++)
    rnd_b_rot[i] = rnd_b[i + 1];
  rnd_b_rot[15] = rnd_b[0];

  uint8_t token[32], tmp[16], enc_iv[16] = {0};
  for (uint8_t i = 0; i < 16; i++)
    tmp[i] = rnd_a[i] ^ enc_iv[i];
  aes_enc_block_(app_rk_, tmp, token);
  for (uint8_t i = 0; i < 16; i++)
    tmp[i] = rnd_b_rot[i] ^ token[i];
  aes_enc_block_(app_rk_, tmp, token + 16);

  uint8_t apdu2[38] = {0x90, 0xAF, 0x00, 0x00, 0x20};
  memcpy(&apdu2[5], token, 32);
  apdu2[37] = 0x00;

  size_t resp2_len = 0;
  if (!desfire_apdu_(apdu2, sizeof(apdu2), scratch_resp_, sizeof(scratch_resp_), resp2_len, sw1, sw2))
    return false;
  return sw1 == DESFIRE_SW1 && sw2 == DESFIRE_OK;
}

bool DesfireReaderComponent::df_read_file_(uint8_t file_id, size_t length, uint8_t *out, size_t &out_len) {
  if (length > 24)
    return false;
  const uint8_t apdu[13] = {
      0x90, 0xBD, 0x00, 0x00, 0x07, file_id, 0x00, 0x00, 0x00,
      static_cast<uint8_t>(length & 0xFF), static_cast<uint8_t>((length >> 8) & 0xFF),
      static_cast<uint8_t>((length >> 16) & 0xFF), 0x00};
  uint8_t sw1 = 0, sw2 = 0;
  out_len = 0;
  if (!desfire_apdu_(apdu, sizeof(apdu), out, DESFIRE_MAX_RESP, out_len, sw1, sw2))
    return false;
  if (sw1 != DESFIRE_SW1 || sw2 != DESFIRE_OK)
    return false;
  if (out_len > length)
    out_len = length;
  return true;
}

static const uint8_t AES_SBOX[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16};
static const uint8_t AES_RSBOX[256] = {
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
    0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
    0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
    0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
    0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
    0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
    0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
    0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
    0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
    0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
    0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d};
static const uint8_t RCON[11] = {0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36};

static uint8_t xtime_(uint8_t x) { return (x << 1) ^ ((x >> 7) * 0x1b); }
static uint8_t mul_(uint8_t a, uint8_t b) {
  return ((b & 1) * a) ^ ((b >> 1 & 1) * xtime_(a)) ^ ((b >> 2 & 1) * xtime_(xtime_(a))) ^
         ((b >> 3 & 1) * xtime_(xtime_(xtime_(a)))) ^ ((b >> 4 & 1) * xtime_(xtime_(xtime_(xtime_(a)))));
}

void aes_key_exp_(const uint8_t *key, uint8_t *rk) {
  memcpy(rk, key, 16);
  for (int i = 4; i < 44; i++) {
    uint8_t tmp[4];
    memcpy(tmp, rk + (i - 1) * 4, 4);
    if (i % 4 == 0) {
      uint8_t t = tmp[0];
      tmp[0] = AES_SBOX[tmp[1]] ^ RCON[i / 4];
      tmp[1] = AES_SBOX[tmp[2]];
      tmp[2] = AES_SBOX[tmp[3]];
      tmp[3] = AES_SBOX[t];
    }
    for (int j = 0; j < 4; j++)
      rk[i * 4 + j] = rk[(i - 4) * 4 + j] ^ tmp[j];
  }
}

void aes_enc_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out) {
  uint8_t s[16];
  memcpy(s, in, 16);
  for (int i = 0; i < 16; i++)
    s[i] ^= rk[i];
  for (int r = 1; r <= 10; r++) {
    for (int i = 0; i < 16; i++)
      s[i] = AES_SBOX[s[i]];
    uint8_t t;
    t = s[1];
    s[1] = s[5];
    s[5] = s[9];
    s[9] = s[13];
    s[13] = t;
    t = s[2];
    s[2] = s[10];
    s[10] = t;
    t = s[6];
    s[6] = s[14];
    s[14] = t;
    t = s[15];
    s[15] = s[11];
    s[11] = s[7];
    s[7] = s[3];
    s[3] = t;
    if (r < 10) {
      for (int c = 0; c < 4; c++) {
        uint8_t *col = s + c * 4, a = col[0], b = col[1], cc = col[2], d = col[3];
        col[0] = xtime_(a) ^ xtime_(b) ^ b ^ cc ^ d;
        col[1] = a ^ xtime_(b) ^ xtime_(cc) ^ cc ^ d;
        col[2] = a ^ b ^ xtime_(cc) ^ xtime_(d) ^ d;
        col[3] = xtime_(a) ^ a ^ b ^ cc ^ xtime_(d);
      }
    }
    for (int i = 0; i < 16; i++)
      s[i] ^= rk[r * 16 + i];
  }
  memcpy(out, s, 16);
}

void aes_dec_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out) {
  uint8_t s[16];
  memcpy(s, in, 16);
  for (int i = 0; i < 16; i++)
    s[i] ^= rk[10 * 16 + i];
  for (int r = 9; r >= 0; r--) {
    uint8_t t;
    t = s[13];
    s[13] = s[9];
    s[9] = s[5];
    s[5] = s[1];
    s[1] = t;
    t = s[10];
    s[10] = s[2];
    s[2] = t;
    t = s[14];
    s[14] = s[6];
    s[6] = t;
    t = s[3];
    s[3] = s[7];
    s[7] = s[11];
    s[11] = s[15];
    s[15] = t;
    for (int i = 0; i < 16; i++)
      s[i] = AES_RSBOX[s[i]];
    for (int i = 0; i < 16; i++)
      s[i] ^= rk[r * 16 + i];
    if (r == 0)
      break;
    for (int c = 0; c < 4; c++) {
      uint8_t *col = s + c * 4, a = col[0], b = col[1], cc = col[2], d = col[3];
      col[0] = mul_(a, 0xe) ^ mul_(b, 0xb) ^ mul_(cc, 0xd) ^ mul_(d, 0x9);
      col[1] = mul_(a, 0x9) ^ mul_(b, 0xe) ^ mul_(cc, 0xb) ^ mul_(d, 0xd);
      col[2] = mul_(a, 0xd) ^ mul_(b, 0x9) ^ mul_(cc, 0xe) ^ mul_(d, 0xb);
      col[3] = mul_(a, 0xb) ^ mul_(b, 0xd) ^ mul_(cc, 0x9) ^ mul_(d, 0xe);
    }
  }
  memcpy(out, s, 16);
}

bool DesfireReaderComponent::aes_cbc_decrypt_(const uint8_t *in, size_t len, uint8_t *out) {
  if (len % 16 != 0)
    return false;
  uint8_t iv[16] = {0};
  for (size_t b = 0; b < len; b += 16) {
    aes_dec_block_(data_rk_, in + b, out + b);
    for (int i = 0; i < 16; i++)
      out[b + i] ^= iv[i];
    memcpy(iv, in + b, 16);
  }
  return true;
}

void DesfireReaderComponent::random_bytes_(uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i++)
    buf[i] = static_cast<uint8_t>(random(256));
}

}  // namespace desfire_reader
}  // namespace esphome