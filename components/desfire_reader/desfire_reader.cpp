#include "desfire_reader.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <string.h>

namespace esphome {
namespace desfire_reader {

// ═══════════════════════════════════════════════════════════════
//  Security helper
// ═══════════════════════════════════════════════════════════════

void DesfireReaderComponent::secure_zero_(volatile uint8_t *buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++)
    buf[i] = 0;
}

// ═══════════════════════════════════════════════════════════════
//  PN532 I2C — non-blocking primitives
// ═══════════════════════════════════════════════════════════════

bool DesfireReaderComponent::write_command_(const uint8_t *cmd, uint8_t cmd_len) {
  uint8_t frame[PN532_BUF_SIZE];
  uint8_t len = cmd_len + 1;
  uint8_t total = 6 + cmd_len + 2;

  if (total > sizeof(frame))
    return false;

  uint8_t dcs = 0xD4;
  for (uint8_t i = 0; i < cmd_len; i++)
    dcs += cmd[i];
  dcs = (uint8_t)(0x100u - dcs);

  frame[0] = 0x00;
  frame[1] = 0x00;
  frame[2] = 0xFF;
  frame[3] = len;
  frame[4] = (uint8_t)(0x100u - len);
  frame[5] = 0xD4;
  memcpy(frame + 6, cmd, cmd_len);
  frame[6 + cmd_len] = dcs;
  frame[7 + cmd_len] = 0x00;

  if (this->write(frame, total) != i2c::ERROR_OK)
    return false;

  state_entered_at_ = millis();
  return true;
}

bool DesfireReaderComponent::try_read_ack_() {
  uint8_t ack[7];
  if (this->read(ack, 7) != i2c::ERROR_OK || ack[0] != 0x01)
    return false;
  return (ack[1] == 0x00 && ack[2] == 0x00 && ack[3] == 0xFF &&
          ack[4] == 0x00 && ack[5] == 0xFF && ack[6] == 0x00);
}

bool DesfireReaderComponent::try_read_response_(uint8_t command,
                                                 uint8_t *resp, uint8_t resp_cap,
                                                 uint8_t &resp_len) {
  resp_len = 0;
  uint8_t buf[PN532_BUF_SIZE];

  if (this->read(buf, sizeof(buf)) != i2c::ERROR_OK || buf[0] != 0x01)
    return false;

  if (buf[1] != 0x00 || buf[2] != 0x00 || buf[3] != 0xFF)
    return false;

  uint8_t frame_len = buf[4];
  if ((uint8_t)(frame_len + buf[5]) != 0)
    return false;

  if (buf[6] != 0xD5 || buf[7] != (uint8_t)(command + 1))
    return false;

  if (frame_len < 2)
    return false;
  uint8_t payload_len = frame_len - 2;

  if ((uint16_t)(10 + payload_len) > sizeof(buf))
    return false;

  uint8_t dcs_sum = 0;
  for (uint8_t i = 0; i < frame_len; i++)
    dcs_sum += buf[6 + i];
  dcs_sum += buf[6 + frame_len];
  if (dcs_sum != 0)
    return false;

  if (buf[7 + frame_len] != 0x00)
    return false;

  uint8_t copy_len = (payload_len <= resp_cap) ? payload_len : resp_cap;
  memcpy(resp, buf + 8, copy_len);
  resp_len = copy_len;
  return true;
}

// ═══════════════════════════════════════════════════════════════
//  DESFire APDU — split send/receive
// ═══════════════════════════════════════════════════════════════

bool DesfireReaderComponent::send_desfire_apdu_(const uint8_t *apdu, uint8_t apdu_len) {
  uint8_t cmd[PN532_BUF_SIZE];
  if ((uint16_t)(2 + apdu_len) > sizeof(cmd))
    return false;
  cmd[0] = PN532_CMD_IN_DATA_EXCHANGE;
  cmd[1] = 0x01;
  memcpy(cmd + 2, apdu, apdu_len);
  return this->write_command_(cmd, 2 + apdu_len);
}

bool DesfireReaderComponent::read_desfire_apdu_(uint8_t *response, uint8_t resp_cap,
                                                 uint8_t &resp_len, uint8_t &sw1,
                                                 uint8_t &sw2) {
  resp_len = 0;
  uint8_t raw[PN532_BUF_SIZE];
  uint8_t raw_len;

  if (!this->try_read_response_(PN532_CMD_IN_DATA_EXCHANGE, raw, sizeof(raw), raw_len))
    return false;

  if (raw_len == 0 || raw[0] != 0x00)
    return false;
  if (raw_len < 3)
    return false;

  sw1 = raw[raw_len - 2];
  sw2 = raw[raw_len - 1];

  uint8_t payload_len = raw_len - 3;
  uint8_t copy_len = (payload_len <= resp_cap) ? payload_len : resp_cap;
  if (copy_len > 0)
    memcpy(response, raw + 1, copy_len);
  resp_len = copy_len;
  return true;
}

// ═══════════════════════════════════════════════════════════════
//  Publish helpers
// ═══════════════════════════════════════════════════════════════

void DesfireReaderComponent::format_uid_(const uint8_t *uid_bytes,
                                         uint8_t uid_len, char *out) {
  uint8_t pos = 0;
  for (uint8_t i = 0; i < uid_len; i++) {
    if (i > 0) out[pos++] = ':';
    uint8_t hi = uid_bytes[i] >> 4;
    uint8_t lo = uid_bytes[i] & 0x0F;
    out[pos++] = (hi < 10) ? ('0' + hi) : ('A' + hi - 10);
    out[pos++] = (lo < 10) ? ('0' + lo) : ('A' + lo - 10);
  }
  out[pos] = '\0';
}

void DesfireReaderComponent::publish_uid_(const char *uid_str) {
  if (!uid_sensor_) return;
  if (strcmp(uid_str, last_uid_) != 0) {
    size_t len = strlen(uid_str);
    if (len >= sizeof(last_uid_)) len = sizeof(last_uid_) - 1;
    memcpy(last_uid_, uid_str, len);
    last_uid_[len] = '\0';
    uid_sensor_->publish_state(last_uid_);
  }
}

void DesfireReaderComponent::publish_auth_(bool state) {
  if (!auth_sensor_) return;
  if (state != last_auth_) {
    last_auth_ = state;
    auth_sensor_->publish_state(state);
  }
}

void DesfireReaderComponent::publish_result_(const char *str) {
  if (!result_sensor_) return;
  if (strcmp(str, last_result_) != 0) {
    size_t len = strlen(str);
    if (len >= sizeof(last_result_)) len = sizeof(last_result_) - 1;
    memcpy(last_result_, str, len);
    last_result_[len] = '\0';
    result_sensor_->publish_state(last_result_);
  }
}

// ═══════════════════════════════════════════════════════════════
//  State machine helpers
// ═══════════════════════════════════════════════════════════════

void DesfireReaderComponent::handle_fail_() {
  if (consecutive_fails_ < 255) consecutive_fails_++;
  uint32_t delay_ms = COOLDOWN_FAIL_BASE_MS;
  for (uint8_t i = 1; i < consecutive_fails_ && delay_ms < COOLDOWN_FAIL_MAX_MS; i++)
    delay_ms = (delay_ms * 2 > COOLDOWN_FAIL_MAX_MS) ? COOLDOWN_FAIL_MAX_MS : delay_ms * 2;
  ESP_LOGD(TAG, "Fail #%d — cooldown %lu ms", consecutive_fails_, (unsigned long)delay_ms);
  publish_auth_(false);
  reset_to_idle_(delay_ms);
}

void DesfireReaderComponent::reset_to_idle_(uint32_t cooldown_ms) {
  state_ = NfcState::IDLE;
  cooldown_until_ = millis() + cooldown_ms;
  secure_zero_((volatile uint8_t *)enc_rnd_b_, sizeof(enc_rnd_b_));
  secure_zero_((volatile uint8_t *)rnd_a_, sizeof(rnd_a_));
  secure_zero_((volatile uint8_t *)token_, sizeof(token_));
}

// ═══════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════

void DesfireReaderComponent::setup() {
  ESP_LOGCONFIG(TAG, "DESFire reader setup...");
  aes_key_exp_(app_key_, app_rk_);
  aes_key_exp_(data_key_, data_rk_);

  static const uint8_t wakeup[] = {
      0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  this->write(wakeup, sizeof(wakeup));
  delay(10);

  // SAMConfiguration — brief blocking at boot only
  const uint8_t sam_cmd[] = {0x14, 0x01, 0x14, 0x00};
  if (!this->write_command_(sam_cmd, sizeof(sam_cmd))) {
    ESP_LOGE(TAG, "PN532 not responding — check I2C wiring (addr 0x%02X)",
             this->address_);
    this->mark_failed();
    return;
  }

  // Wait for ACK (max 50ms)
  uint32_t start = millis();
  bool ack_ok = false;
  while (millis() - start < 50) {
    if (try_read_ack_()) { ack_ok = true; break; }
    delay(2);
  }
  if (!ack_ok) {
    ESP_LOGE(TAG, "PN532 SAM ACK timeout — check wiring");
    this->mark_failed();
    return;
  }

  // Wait for SAM response (max 100ms)
  uint8_t sam_resp[8];
  uint8_t sam_len = 0;
  start = millis();
  while (millis() - start < 100) {
    if (try_read_response_(0x14, sam_resp, sizeof(sam_resp), sam_len))
      break;
    delay(3);
  }

  // RFConfiguration MaxRetries
  const uint8_t rf_cfg[] = {0x12, 0x05, 0xFF, 0x01, 0x02};
  if (this->write_command_(rf_cfg, sizeof(rf_cfg))) {
    start = millis();
    ack_ok = false;
    while (millis() - start < 50) {
      if (try_read_ack_()) { ack_ok = true; break; }
      delay(2);
    }
    if (ack_ok) {
      uint8_t rf_resp[4];
      uint8_t rf_len;
      start = millis();
      while (millis() - start < 100) {
        if (try_read_response_(0x12, rf_resp, sizeof(rf_resp), rf_len)) {
          ESP_LOGCONFIG(TAG, "PN532 MaxRetries configured OK.");
          break;
        }
        delay(3);
      }
    } else {
      ESP_LOGW(TAG, "PN532 RFConfiguration ACK failed — using defaults.");
    }
  } else {
    ESP_LOGW(TAG, "PN532 RFConfiguration write failed — using defaults.");
  }

  state_ = NfcState::IDLE;
  cooldown_until_ = 0;
  update_requested_ = false;
  ESP_LOGCONFIG(TAG, "PN532 initialized.");
}

// ═══════════════════════════════════════════════════════════════
//  update() — called at update_interval, just flags "time to poll"
// ═══════════════════════════════════════════════════════════════

void DesfireReaderComponent::update() {
  // Only request a new detection if we're idle
  if (state_ == NfcState::IDLE)
    update_requested_ = true;
}

// ═══════════════════════════════════════════════════════════════
//  loop() — called every ~16ms by ESPHome main loop
//  Does ONE I2C operation per call, never blocks
// ═══════════════════════════════════════════════════════════════

void DesfireReaderComponent::loop() {
  uint32_t now = millis();

  switch (state_) {

  // ─────────────────────────────────────────────
  case NfcState::IDLE: {
    if (!update_requested_)
      return;
    if ((int32_t)(cooldown_until_ - now) > 0)
      return;

    update_requested_ = false;

    const uint8_t detect_cmd[] = {PN532_CMD_IN_LIST_PASSIVE, 0x01, 0x00};
    if (!this->write_command_(detect_cmd, sizeof(detect_cmd))) {
      reset_to_idle_(100);
      return;
    }
    state_ = NfcState::DETECT_WAIT_ACK;
    state_entered_at_ = now;
    return;
  }

  // ─────────────────────────────────────────────
  case NfcState::DETECT_WAIT_ACK: {
    if (try_read_ack_()) {
      state_ = NfcState::DETECT_WAIT_RESP;
      state_entered_at_ = millis();
      return;
    }
    if (millis() - state_entered_at_ > ACK_TIMEOUT_MS) {
      ESP_LOGD(TAG, "Detect ACK timeout");
      reset_to_idle_(100);
    }
    return;
  }

  // ─────────────────────────────────────────────
  case NfcState::DETECT_WAIT_RESP: {
    uint8_t resp[48];
    uint8_t resp_len;

    if (try_read_response_(PN532_CMD_IN_LIST_PASSIVE, resp, sizeof(resp), resp_len)) {
      if (resp_len > 0 && resp[0] != 0) {
        // Card detected
        no_card_count_ = 0;

        uint8_t uid_len = 0;
        const uint8_t *uid_ptr = nullptr;
        if (resp_len >= 7) {
          uid_len = resp[5];
          if (uid_len > 7) uid_len = 7;
          if (resp_len >= (uint8_t)(6 + uid_len))
            uid_ptr = resp + 6;
          else
            uid_len = 0;
        }

        // Same card still present
        if (uid_len > 0 && uid_len == prev_uid_len_ &&
            memcmp(uid_ptr, prev_uid_, uid_len) == 0) {
          reset_to_idle_(COOLDOWN_SUCCESS_MS);
          return;
        }

        // New card
        current_uid_len_ = 0;
        current_uid_str_[0] = '\0';
        if (uid_len > 0 && uid_ptr != nullptr) {
          memcpy(prev_uid_, uid_ptr, uid_len);
          prev_uid_len_ = uid_len;
          memcpy(current_uid_bytes_, uid_ptr, uid_len);
          current_uid_len_ = uid_len;
          format_uid_(uid_ptr, uid_len, current_uid_str_);
          ESP_LOGI(TAG, "UID: %s", current_uid_str_);
        }
        card_present_ = true;
        ESP_LOGI(TAG, "New card — starting DESFire workflow");

        // Send SelectApplication
        uint8_t apdu[] = {0x90, 0x5A, 0x00, 0x00, 0x03,
                          app_id_[0], app_id_[1], app_id_[2], 0x00};
        if (!send_desfire_apdu_(apdu, sizeof(apdu))) {
          ESP_LOGE(TAG, "SelectApp send failed");
          handle_fail_();
          return;
        }
        state_ = NfcState::SELECT_WAIT_ACK;
        state_entered_at_ = millis();
        return;
      }

      // No card
      if (no_card_count_ < 255) no_card_count_++;
      if (no_card_count_ >= 3 && card_present_) {
        card_present_ = false;
        prev_uid_len_ = 0;
        publish_auth_(false);
        publish_result_("");
        publish_uid_("");
        ESP_LOGD(TAG, "Card removed");
      }
      reset_to_idle_(0);
      return;
    }

    if (millis() - state_entered_at_ > RESP_TIMEOUT_MS) {
      if (no_card_count_ < 255) no_card_count_++;
      if (no_card_count_ >= 3 && card_present_) {
        card_present_ = false;
        prev_uid_len_ = 0;
        publish_auth_(false);
        publish_result_("");
        publish_uid_("");
        ESP_LOGD(TAG, "Card removed (timeout)");
      }
      reset_to_idle_(100);
    }
    return;
  }

  // ─────────────────────────────────────────────
  case NfcState::SELECT_WAIT_ACK: {
    if (try_read_ack_()) {
      state_ = NfcState::SELECT_WAIT_RESP;
      state_entered_at_ = millis();
      return;
    }
    if (millis() - state_entered_at_ > ACK_TIMEOUT_MS) {
      ESP_LOGE(TAG, "SelectApp ACK timeout");
      handle_fail_();
    }
    return;
  }

  // ─────────────────────────────────────────────
  case NfcState::SELECT_WAIT_RESP: {
    uint8_t resp[8];
    uint8_t resp_len, sw1, sw2;

    if (read_desfire_apdu_(resp, sizeof(resp), resp_len, sw1, sw2)) {
      if (sw1 == DESFIRE_SW1 && sw2 == DESFIRE_OK) {
        ESP_LOGD(TAG, "SelectApp OK");

        uint8_t apdu1[] = {0x90, 0xAA, 0x00, 0x00, 0x01, 0x00, 0x00};
        if (!send_desfire_apdu_(apdu1, sizeof(apdu1))) {
          ESP_LOGE(TAG, "Auth1 send failed");
          handle_fail_();
          return;
        }
        state_ = NfcState::AUTH1_WAIT_ACK;
        state_entered_at_ = millis();
        return;
      }
      ESP_LOGE(TAG, "SelectApp FAILED — app %02X%02X%02X not on card?",
               app_id_[0], app_id_[1], app_id_[2]);
      handle_fail_();
      return;
    }

    if (millis() - state_entered_at_ > RESP_TIMEOUT_MS) {
      ESP_LOGE(TAG, "SelectApp read timeout");
      handle_fail_();
    }
    return;
  }

  // ─────────────────────────────────────────────
  case NfcState::AUTH1_WAIT_ACK: {
    if (try_read_ack_()) {
      state_ = NfcState::AUTH1_WAIT_RESP;
      state_entered_at_ = millis();
      return;
    }
    if (millis() - state_entered_at_ > ACK_TIMEOUT_MS) {
      ESP_LOGE(TAG, "Auth1 ACK timeout");
      handle_fail_();
    }
    return;
  }

  // ─────────────────────────────────────────────
  case NfcState::AUTH1_WAIT_RESP: {
    uint8_t resp1[32];
    uint8_t resp1_len, sw1, sw2;

    if (read_desfire_apdu_(resp1, sizeof(resp1), resp1_len, sw1, sw2)) {
      if (sw2 != DESFIRE_MORE_FRAMES || resp1_len != 16) {
        ESP_LOGE(TAG, "Auth1 unexpected response (sw2=0x%02X, len=%d)", sw2, resp1_len);
        handle_fail_();
        return;
      }

      // Compute auth phase 2
      memcpy(enc_rnd_b_, resp1, 16);

      uint8_t rnd_b[16];
      aes_dec_block_(app_rk_, enc_rnd_b_, rnd_b);

      random_bytes_(rnd_a_, 16);

      uint8_t rnd_b_rot[16];
      memcpy(rnd_b_rot, rnd_b + 1, 15);
      rnd_b_rot[15] = rnd_b[0];

      uint8_t xor_buf[16];
      for (uint8_t i = 0; i < 16; i++)
        xor_buf[i] = rnd_a_[i] ^ enc_rnd_b_[i];
      aes_enc_block_(app_rk_, xor_buf, token_);

      for (uint8_t i = 0; i < 16; i++)
        xor_buf[i] = rnd_b_rot[i] ^ token_[i];
      aes_enc_block_(app_rk_, xor_buf, token_ + 16);

      uint8_t apdu2[38];
      apdu2[0] = 0x90;
      apdu2[1] = 0xAF;
      apdu2[2] = 0x00;
      apdu2[3] = 0x00;
      apdu2[4] = 0x20;
      memcpy(apdu2 + 5, token_, 32);
      apdu2[37] = 0x00;

      secure_zero_((volatile uint8_t *)rnd_b, 16);
      secure_zero_((volatile uint8_t *)rnd_b_rot, 16);
      secure_zero_((volatile uint8_t *)xor_buf, 16);

      if (!send_desfire_apdu_(apdu2, sizeof(apdu2))) {
        ESP_LOGE(TAG, "Auth2 send failed");
        handle_fail_();
        return;
      }
      state_ = NfcState::AUTH2_WAIT_ACK;
      state_entered_at_ = millis();
      return;
    }

    if (millis() - state_entered_at_ > RESP_TIMEOUT_MS) {
      ESP_LOGE(TAG, "Auth1 read timeout");
      handle_fail_();
    }
    return;
  }

  // ─────────────────────────────────────────────
  case NfcState::AUTH2_WAIT_ACK: {
    if (try_read_ack_()) {
      state_ = NfcState::AUTH2_WAIT_RESP;
      state_entered_at_ = millis();
      return;
    }
    if (millis() - state_entered_at_ > ACK_TIMEOUT_MS) {
      ESP_LOGE(TAG, "Auth2 ACK timeout");
      handle_fail_();
    }
    return;
  }

  // ─────────────────────────────────────────────
  case NfcState::AUTH2_WAIT_RESP: {
    uint8_t resp2[32];
    uint8_t resp2_len, sw1, sw2;

    if (read_desfire_apdu_(resp2, sizeof(resp2), resp2_len, sw1, sw2)) {
      if (!(sw1 == DESFIRE_SW1 && sw2 == DESFIRE_OK)) {
        ESP_LOGE(TAG, "AES auth FAILED — wrong app key?");
        handle_fail_();
        return;
      }
      if (resp2_len != 16) {
        ESP_LOGE(TAG, "Auth response wrong length (%d)", resp2_len);
        handle_fail_();
        return;
      }

      // Verify mutual auth
      uint8_t rnd_a_expected[16];
      memcpy(rnd_a_expected, rnd_a_ + 1, 15);
      rnd_a_expected[15] = rnd_a_[0];

      uint8_t dec_tmp[16];
      aes_dec_block_(app_rk_, resp2, dec_tmp);
      uint8_t rnd_a_received[16];
      for (uint8_t i = 0; i < 16; i++)
        rnd_a_received[i] = dec_tmp[i] ^ token_[16 + i];

      uint8_t diff = 0;
      for (uint8_t i = 0; i < 16; i++)
        diff |= rnd_a_received[i] ^ rnd_a_expected[i];

      secure_zero_((volatile uint8_t *)rnd_a_expected, 16);
      secure_zero_((volatile uint8_t *)rnd_a_received, 16);
      secure_zero_((volatile uint8_t *)dec_tmp, 16);

      if (diff != 0) {
        ESP_LOGE(TAG, "Mutual auth FAILED — RndA' mismatch");
        handle_fail_();
        return;
      }

      ESP_LOGI(TAG, "Mutual AES auth verified — card is genuine");

      // Send ReadFile (file 0x01, length 0 = all)
      uint8_t apdu[] = {
          0x90, 0xBD, 0x00, 0x00, 0x07, 0x01,
          0x00, 0x00, 0x00,
          0x00, 0x00, 0x00,
          0x00};
      if (!send_desfire_apdu_(apdu, sizeof(apdu))) {
        ESP_LOGE(TAG, "ReadFile send failed");
        handle_fail_();
        return;
      }
      state_ = NfcState::READ_WAIT_ACK;
      state_entered_at_ = millis();
      return;
    }

    if (millis() - state_entered_at_ > RESP_TIMEOUT_MS) {
      ESP_LOGE(TAG, "Auth2 read timeout");
      handle_fail_();
    }
    return;
  }

  // ─────────────────────────────────────────────
  case NfcState::READ_WAIT_ACK: {
    if (try_read_ack_()) {
      state_ = NfcState::READ_WAIT_RESP;
      state_entered_at_ = millis();
      return;
    }
    if (millis() - state_entered_at_ > ACK_TIMEOUT_MS) {
      ESP_LOGE(TAG, "ReadFile ACK timeout");
      handle_fail_();
    }
    return;
  }

  // ─────────────────────────────────────────────
  case NfcState::READ_WAIT_RESP: {
    uint8_t resp[48];
    uint8_t resp_len, sw1, sw2;

    if (read_desfire_apdu_(resp, sizeof(resp), resp_len, sw1, sw2)) {
      if (!(sw1 == DESFIRE_SW1 && sw2 == DESFIRE_OK)) {
        ESP_LOGE(TAG, "ReadFile FAILED (sw1=0x%02X sw2=0x%02X)", sw1, sw2);
        handle_fail_();
        return;
      }

      memcpy(raw_file_, resp, resp_len);
      raw_file_len_ = resp_len;
      state_ = NfcState::PUBLISH;
      return;
    }

    if (millis() - state_entered_at_ > RESP_TIMEOUT_MS) {
      ESP_LOGE(TAG, "ReadFile read timeout");
      handle_fail_();
    }
    return;
  }

  // ─────────────────────────────────────────────
  case NfcState::PUBLISH: {
    ESP_LOGD(TAG, "ReadFile OK — %d bytes", raw_file_len_);

    uint8_t cipher_len = (raw_file_len_ / 16) * 16;

    if (cipher_len == 0 || cipher_len > 48) {
      ESP_LOGE(TAG, "Bad cipher length (%d from %d raw bytes)", cipher_len, raw_file_len_);
      handle_fail_();
      return;
    }

    if (raw_file_len_ != cipher_len) {
      ESP_LOGD(TAG, "Stripped %d trailing bytes (CMAC)", raw_file_len_ - cipher_len);
    }

    uint8_t decrypted[48];
    uint8_t zero_iv[16] = {0};
    if (!aes_cbc_decrypt_(raw_file_, cipher_len, zero_iv, decrypted)) {
      ESP_LOGE(TAG, "AES decrypt FAILED");
      secure_zero_((volatile uint8_t *)decrypted, sizeof(decrypted));
      handle_fail_();
      return;
    }

    char result[49];
    uint8_t rlen = 0;
    for (uint8_t i = 0; i < cipher_len && i < 48 &&
         decrypted[i] >= 0x20 && decrypted[i] <= 0x7E; i++)
      result[rlen++] = (char)decrypted[i];
    result[rlen] = '\0';

    secure_zero_((volatile uint8_t *)decrypted, sizeof(decrypted));

    ESP_LOGI(TAG, "Auth + read OK (%d bytes)", rlen);

    if (current_uid_str_[0] != '\0')
      publish_uid_(current_uid_str_);
    publish_auth_(true);
    publish_result_(result);

    secure_zero_((volatile uint8_t *)result, sizeof(result));

    consecutive_fails_ = 0;
    reset_to_idle_(COOLDOWN_SUCCESS_MS);
    return;
  }

  }  // end switch
}

void DesfireReaderComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "DESFire Reader:");
  ESP_LOGCONFIG(TAG, "  App ID: %02X:%02X:%02X", app_id_[0], app_id_[1], app_id_[2]);
  ESP_LOGCONFIG(TAG, "  Mode: non-blocking state machine (loop)");
  LOG_I2C_DEVICE(this);
}

// ═══════════════════════════════════════════════════════════════
//  AES-128 portable (no external libs)
// ═══════════════════════════════════════════════════════════════

static const uint8_t AES_SBOX[256] PROGMEM = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16};
static const uint8_t AES_RSBOX[256] PROGMEM = {
    0x52,0x09,0x6a,0xd5,0x30,0x36,0xa5,0x38,0xbf,0x40,0xa3,0x9e,0x81,0xf3,0xd7,0xfb,
    0x7c,0xe3,0x39,0x82,0x9b,0x2f,0xff,0x87,0x34,0x8e,0x43,0x44,0xc4,0xde,0xe9,0xcb,
    0x54,0x7b,0x94,0x32,0xa6,0xc2,0x23,0x3d,0xee,0x4c,0x95,0x0b,0x42,0xfa,0xc3,0x4e,
    0x08,0x2e,0xa1,0x66,0x28,0xd9,0x24,0xb2,0x76,0x5b,0xa2,0x49,0x6d,0x8b,0xd1,0x25,
    0x72,0xf8,0xf6,0x64,0x86,0x68,0x98,0x16,0xd4,0xa4,0x5c,0xcc,0x5d,0x65,0xb6,0x92,
    0x6c,0x70,0x48,0x50,0xfd,0xed,0xb9,0xda,0x5e,0x15,0x46,0x57,0xa7,0x8d,0x9d,0x84,
    0x90,0xd8,0xab,0x00,0x8c,0xbc,0xd3,0x0a,0xf7,0xe4,0x58,0x05,0xb8,0xb3,0x45,0x06,
    0xd0,0x2c,0x1e,0x8f,0xca,0x3f,0x0f,0x02,0xc1,0xaf,0xbd,0x03,0x01,0x13,0x8a,0x6b,
    0x3a,0x91,0x11,0x41,0x4f,0x67,0xdc,0xea,0x97,0xf2,0xcf,0xce,0xf0,0xb4,0xe6,0x73,
    0x96,0xac,0x74,0x22,0xe7,0xad,0x35,0x85,0xe2,0xf9,0x37,0xe8,0x1c,0x75,0xdf,0x6e,
    0x47,0xf1,0x1a,0x71,0x1d,0x29,0xc5,0x89,0x6f,0xb7,0x62,0x0e,0xaa,0x18,0xbe,0x1b,
    0xfc,0x56,0x3e,0x4b,0xc6,0xd2,0x79,0x20,0x9a,0xdb,0xc0,0xfe,0x78,0xcd,0x5a,0xf4,
    0x1f,0xdd,0xa8,0x33,0x88,0x07,0xc7,0x31,0xb1,0x12,0x10,0x59,0x27,0x80,0xec,0x5f,
    0x60,0x51,0x7f,0xa9,0x19,0xb5,0x4a,0x0d,0x2d,0xe5,0x7a,0x9f,0x93,0xc9,0x9c,0xef,
    0xa0,0xe0,0x3b,0x4d,0xae,0x2a,0xf5,0xb0,0xc8,0xeb,0xbb,0x3c,0x83,0x53,0x99,0x61,
    0x17,0x2b,0x04,0x7e,0xba,0x77,0xd6,0x26,0xe1,0x69,0x14,0x63,0x55,0x21,0x0c,0x7d};
static const uint8_t RCON[11] PROGMEM = {
    0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36};

static inline uint8_t sbox_rd(const uint8_t *tbl, uint8_t idx) {
#ifdef USE_ESP8266
  return pgm_read_byte(tbl + idx);
#else
  return tbl[idx];
#endif
}

static uint8_t xtime_(uint8_t x) { return (x << 1) ^ ((x >> 7) * 0x1b); }
static uint8_t mul_(uint8_t a, uint8_t b) {
  return ((b & 1) * a) ^ ((b >> 1 & 1) * xtime_(a)) ^
         ((b >> 2 & 1) * xtime_(xtime_(a))) ^
         ((b >> 3 & 1) * xtime_(xtime_(xtime_(a)))) ^
         ((b >> 4 & 1) * xtime_(xtime_(xtime_(xtime_(a)))));
}

void aes_key_exp_(const uint8_t *key, uint8_t *rk) {
  memcpy(rk, key, 16);
  for (int i = 4; i < 44; i++) {
    uint8_t tmp[4];
    memcpy(tmp, rk + (i - 1) * 4, 4);
    if (i % 4 == 0) {
      uint8_t t = tmp[0];
      tmp[0] = sbox_rd(AES_SBOX, tmp[1]) ^ sbox_rd(RCON, i / 4);
      tmp[1] = sbox_rd(AES_SBOX, tmp[2]);
      tmp[2] = sbox_rd(AES_SBOX, tmp[3]);
      tmp[3] = sbox_rd(AES_SBOX, t);
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
      s[i] = sbox_rd(AES_SBOX, s[i]);
    uint8_t t;
    t = s[1]; s[1] = s[5]; s[5] = s[9]; s[9] = s[13]; s[13] = t;
    t = s[2]; s[2] = s[10]; s[10] = t;
    t = s[6]; s[6] = s[14]; s[14] = t;
    t = s[15]; s[15] = s[11]; s[11] = s[7]; s[7] = s[3]; s[3] = t;
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
    t = s[13]; s[13] = s[9]; s[9] = s[5]; s[5] = s[1]; s[1] = t;
    t = s[10]; s[10] = s[2]; s[2] = t;
    t = s[14]; s[14] = s[6]; s[6] = t;
    t = s[3]; s[3] = s[7]; s[7] = s[11]; s[11] = s[15]; s[15] = t;
    for (int i = 0; i < 16; i++)
      s[i] = sbox_rd(AES_RSBOX, s[i]);
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

bool DesfireReaderComponent::aes_cbc_decrypt_(const uint8_t *in, uint8_t len,
                                              const uint8_t *iv_in,
                                              uint8_t *out) {
  if (len % 16 != 0)
    return false;
  uint8_t iv[16];
  if (iv_in != nullptr)
    memcpy(iv, iv_in, 16);
  else
    memset(iv, 0, 16);
  for (uint8_t b = 0; b < len; b += 16) {
    aes_dec_block_(data_rk_, in + b, out + b);
    for (uint8_t i = 0; i < 16; i++)
      out[b + i] ^= iv[i];
    memcpy(iv, in + b, 16);
  }
  return true;
}

void DesfireReaderComponent::random_bytes_(uint8_t *buf, uint8_t len) {
#ifdef USE_ESP32
  esp_fill_random(buf, len);
#elif defined(USE_ESP8266)
  for (uint8_t i = 0; i < len; i++)
    buf[i] = static_cast<uint8_t>(*(volatile uint32_t *)0x3FF20E44);
#else
  for (uint8_t i = 0; i < len; i++)
    buf[i] = static_cast<uint8_t>(esp_random());
#endif
}

}  // namespace desfire_reader
}  // namespace esphome