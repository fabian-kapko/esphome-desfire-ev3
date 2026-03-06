#include "desfire_reader.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <string.h>

namespace esphome
{
  namespace desfire_reader
  {

    // ═══════════════════════════════════════════════════════════════
    //  Raw PN532 I2C frame protocol
    //
    //  The PN532 prepends a status byte (0x01=ready, 0x00=busy) to
    //  every I2C read transaction.
    //
    //  Normal information frame layout (host → PN532):
    //    [0x00] PREAMBLE
    //    [0x00 0xFF] START CODE
    //    [LEN]  byte count of (TFI + data)
    //    [LCS]  LEN + LCS == 0x00 mod 256
    //    [0xD4] TFI (host to PN532)
    //    [data...] command byte(s)
    //    [DCS]  TFI + sum(data) + DCS == 0x00 mod 256
    //    [0x00] POSTAMBLE
    //
    //  ACK frame (PN532 → host, 7 bytes with status prepended):
    //    [0x01] status  [0x00 0x00 0xFF 0x00 0xFF 0x00]
    // ═══════════════════════════════════════════════════════════════

    bool DesfireReaderComponent::write_command_(const std::vector<uint8_t> &cmd)
    {
      uint8_t len = (uint8_t)(cmd.size() + 1); // +1 for TFI
      uint8_t lcs = (uint8_t)(0x100u - len);

      uint8_t dcs = 0xD4; // TFI counts in checksum
      for (auto b : cmd)
        dcs += b;
      dcs = (uint8_t)(0x100u - dcs);

      std::vector<uint8_t> frame;
      frame.push_back(0x00); // PREAMBLE
      frame.push_back(0x00); // START CODE
      frame.push_back(0xFF);
      frame.push_back(len);
      frame.push_back(lcs);
      frame.push_back(0xD4); // TFI
      frame.insert(frame.end(), cmd.begin(), cmd.end());
      frame.push_back(dcs);
      frame.push_back(0x00); // POSTAMBLE

      if (!this->write_bytes_raw(frame.data(), (uint8_t)frame.size()))
        return false;

      delay(2);

      // Poll for ACK: each read prepends the status byte, so we read 7 bytes
      // (1 status + 6 ACK frame bytes).
      for (int retry = 0; retry < 20; retry++)
      {
        uint8_t buf[7] = {};
        if (this->read_bytes_raw(buf, 7) && buf[0] == 0x01)
        {
          return (buf[1] == 0x00 && buf[2] == 0x00 && buf[3] == 0xFF &&
                  buf[4] == 0x00 && buf[5] == 0xFF && buf[6] == 0x00);
        }
        delay(5);
      }
      return false;
    }

    bool DesfireReaderComponent::read_response_(uint8_t command, std::vector<uint8_t> &resp)
    {
      // Response layout in buf (status byte always first):
      //   buf[0]  = 0x01 (ready)
      //   buf[1]  = 0x00 (PREAMBLE)
      //   buf[2]  = 0x00  \
      //   buf[3]  = 0xFF  /  START CODE
      //   buf[4]  = LEN   (TFI + CMD + payload bytes)
      //   buf[5]  = LCS
      //   buf[6]  = 0xD5  (TFI, PN532 → host)
      //   buf[7]  = command + 1
      //   buf[8+] = payload  (LEN - 2 bytes)
      //   ...DCS + POSTAMBLE...
      for (int retry = 0; retry < 100; retry++)
      {
        uint8_t buf[64] = {};
        if (this->read_bytes_raw(buf, sizeof(buf)) && buf[0] == 0x01)
        {
          if (buf[6] != 0xD5 || buf[7] != command + 1)
            return false;
          uint8_t data_len = buf[4] - 2; // LEN excludes PREAMBLE/STARTCODE/LCS/DCS/POSTAMBLE
          resp.clear();
          for (int i = 0; i < data_len; i++)
            resp.push_back(buf[8 + i]);
          return true;
        }
        delay(10);
      }
      return false;
    }

    // ═══════════════════════════════════════════════════════════════
    //  Setup / Update
    // ═══════════════════════════════════════════════════════════════

    void DesfireReaderComponent::setup()
    {
      ESP_LOGCONFIG(TAG, "DESFire reader setup...");
      randomSeed(analogRead(A0) ^ micros());
      aes_key_exp_(app_key_, app_rk_);
      aes_key_exp_(data_key_, data_rk_);

      // Wake the PN532 from any low-power state.
      static const uint8_t wakeup[] = {
          0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      this->write_bytes_raw(wakeup, sizeof(wakeup));
      delay(10);

      // SAMConfiguration: Normal mode (0x01), timeout=0x14, IRQ disabled (0x00)
      std::vector<uint8_t> sam_cmd = {0x14, 0x01, 0x14, 0x00};
      if (!this->write_command_(sam_cmd))
      {
        ESP_LOGE(TAG, "PN532 not responding — check I2C wiring (address 0x%02X)",
                 this->address_);
        this->mark_failed();
        return;
      }
      std::vector<uint8_t> resp;
      this->read_response_(0x14, resp); // clear response
      ESP_LOGCONFIG(TAG, "PN532 initialized.");
    }

    void DesfireReaderComponent::update()
    {
      std::vector<uint8_t> detect_cmd = {PN532_CMD_IN_LIST_PASSIVE, 0x01, 0x00};

      if (!this->write_command_(detect_cmd))
      {
        no_card_count_++;
        if (last_card_state_ && no_card_count_ > 1)
        {
          ESP_LOGD(TAG, "Card removed");
          last_card_state_ = false;
          if (auth_sensor_)
            auth_sensor_->publish_state(false);
          if (result_sensor_)
            result_sensor_->publish_state("");
          if (uid_sensor_)
            uid_sensor_->publish_state("");
        }
        return;
      }

      std::vector<uint8_t> resp;
      if (!this->read_response_(PN532_CMD_IN_LIST_PASSIVE, resp) ||
          resp.empty() || resp[0] == 0)
      {
        no_card_count_++;
        if (last_card_state_ && no_card_count_ > 1)
        {
          ESP_LOGD(TAG, "Card removed");
          last_card_state_ = false;
          if (auth_sensor_)
            auth_sensor_->publish_state(false);
          if (result_sensor_)
            result_sensor_->publish_state("");
          if (uid_sensor_)
            uid_sensor_->publish_state("");
        }
        return;
      }

      no_card_count_ = 0;

      if (last_card_state_)
        return;
      last_card_state_ = true;

      ESP_LOGI(TAG, "New card — starting DESFire workflow");

      if (!df_select_app_())
      {
        ESP_LOGE(TAG, "SelectApp FAILED — app %02X%02X%02X not on card?",
                 app_id_[0], app_id_[1], app_id_[2]);
        if (auth_sensor_)
          auth_sensor_->publish_state(false);
        return;
      }

      if (!df_auth_aes_(app_key_))
      {
        ESP_LOGE(TAG, "AES auth FAILED — wrong app key?");
        if (auth_sensor_)
          auth_sensor_->publish_state(false);
        return;
      }

      std::vector<uint8_t> raw;
      if (!df_read_file_(0x01, 16, raw))
      {
        ESP_LOGE(TAG, "ReadFile FAILED");
        if (auth_sensor_)
          auth_sensor_->publish_state(false);
        return;
      }

      if (raw.size() < 16)
      {
        ESP_LOGE(TAG, "Data too short");
        if (auth_sensor_)
          auth_sensor_->publish_state(false);
        return;
      }

      uint8_t decrypted[16] = {0};
      if (!aes_cbc_decrypt_(raw.data(), 16, decrypted))
      {
        ESP_LOGE(TAG, "AES decrypt FAILED");
        if (auth_sensor_)
          auth_sensor_->publish_state(false);
        return;
      }

      std::string result;
      for (int i = 0; i < 16 && decrypted[i] >= 0x20 && decrypted[i] <= 0x7E; i++)
        result += (char)decrypted[i];

      ESP_LOGI(TAG, "SUCCESS — '%s'", result.c_str());
      if (auth_sensor_)
        auth_sensor_->publish_state(true);
      if (result_sensor_)
        result_sensor_->publish_state(result);
    }

    void DesfireReaderComponent::dump_config()
    {
      ESP_LOGCONFIG(TAG, "DESFire Reader:");
      ESP_LOGCONFIG(TAG, "  App ID: %02X:%02X:%02X", app_id_[0], app_id_[1], app_id_[2]);
      LOG_I2C_DEVICE(this);
    }

    // ═══════════════════════════════════════════════════════════════
    //  DESFire APDU via PN532 InDataExchange
    // ═══════════════════════════════════════════════════════════════

    bool DesfireReaderComponent::desfire_apdu_(const std::vector<uint8_t> &apdu,
                                               std::vector<uint8_t> &response,
                                               uint8_t &sw1, uint8_t &sw2)
    {
      std::vector<uint8_t> cmd;
      cmd.push_back(PN532_CMD_IN_DATA_EXCHANGE);
      cmd.push_back(0x01);
      for (auto b : apdu)
        cmd.push_back(b);

      if (!this->write_command_(cmd))
      {
        ESP_LOGW(TAG, "desfire_apdu_: write_command_ failed");
        return false;
      }

      std::vector<uint8_t> resp;
      if (!this->read_response_(PN532_CMD_IN_DATA_EXCHANGE, resp))
      {
        ESP_LOGW(TAG, "desfire_apdu_: read_response_ failed");
        return false;
      }

      if (resp.empty() || resp[0] != 0x00)
      {
        ESP_LOGW(TAG, "desfire_apdu_: PN532 error 0x%02X", resp.empty() ? 0xFF : resp[0]);
        return false;
      }
      if (resp.size() < 3)
      {
        ESP_LOGW(TAG, "desfire_apdu_: too short (%d)", (int)resp.size());
        return false;
      }

      sw1 = resp[resp.size() - 2];
      sw2 = resp[resp.size() - 1];
      response.clear();
      for (size_t i = 1; i < resp.size() - 2; i++)
        response.push_back(resp[i]);

      return true;
    }

    // ═══════════════════════════════════════════════════════════════
    //  DESFire Operations
    // ═══════════════════════════════════════════════════════════════

    bool DesfireReaderComponent::df_select_picc_()
    {
      std::vector<uint8_t> apdu = {0x90, 0x5A, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00};
      std::vector<uint8_t> resp;
      uint8_t sw1, sw2;
      if (!desfire_apdu_(apdu, resp, sw1, sw2))
        return false;
      return sw1 == DESFIRE_SW1 && sw2 == DESFIRE_OK;
    }

    bool DesfireReaderComponent::df_select_app_()
    {
      std::vector<uint8_t> apdu = {
          0x90, 0x5A, 0x00, 0x00, 0x03,
          app_id_[0], app_id_[1], app_id_[2], 0x00};
      std::vector<uint8_t> resp;
      uint8_t sw1, sw2;
      if (!desfire_apdu_(apdu, resp, sw1, sw2))
        return false;
      return sw1 == DESFIRE_SW1 && sw2 == DESFIRE_OK;
    }

    bool DesfireReaderComponent::df_auth_aes_(const uint8_t *key)
    {
      // Step 1: AuthenticateAES (INS=0xAA), key number 0
      std::vector<uint8_t> apdu1 = {0x90, 0xAA, 0x00, 0x00, 0x01, 0x00, 0x00};
      std::vector<uint8_t> resp1;
      uint8_t sw1, sw2;

      if (!desfire_apdu_(apdu1, resp1, sw1, sw2))
        return false;
      if (sw2 != DESFIRE_MORE_FRAMES || resp1.size() != 16)
      {
        ESP_LOGW(TAG, "df_auth_aes_ step1 SW:%02X%02X len:%d", sw1, sw2, (int)resp1.size());
        return false;
      }

      uint8_t rnd_b[16], iv[16] = {0};
      aes_dec_block_(app_rk_, resp1.data(), rnd_b);
      for (int i = 0; i < 16; i++)
        rnd_b[i] ^= iv[i];

      // Generate RndA, rotate RndB left 1
      uint8_t rnd_a[16];
      random_bytes_(rnd_a, 16);
      uint8_t rnd_b_rot[16];
      for (int i = 0; i < 15; i++)
        rnd_b_rot[i] = rnd_b[i + 1];
      rnd_b_rot[15] = rnd_b[0];

      // Encrypt (RndA || RndB_rot) with AES-CBC, IV=zeros
      uint8_t token[32], tmp[16], enc_iv[16] = {0};
      for (int i = 0; i < 16; i++)
        tmp[i] = rnd_a[i] ^ enc_iv[i];
      aes_enc_block_(app_rk_, tmp, token);
      for (int i = 0; i < 16; i++)
        tmp[i] = rnd_b_rot[i] ^ token[i];
      aes_enc_block_(app_rk_, tmp, token + 16);

      // Send 32-byte token
      std::vector<uint8_t> apdu2 = {0x90, 0xAF, 0x00, 0x00, 0x20};
      for (int i = 0; i < 32; i++)
        apdu2.push_back(token[i]);
      apdu2.push_back(0x00);

      std::vector<uint8_t> resp2;
      if (!desfire_apdu_(apdu2, resp2, sw1, sw2))
        return false;
      if (!(sw1 == DESFIRE_SW1 && sw2 == DESFIRE_OK))
      {
        ESP_LOGW(TAG, "df_auth_aes_ step2 SW:%02X%02X", sw1, sw2);
        return false;
      }
      return true;
    }

    bool DesfireReaderComponent::df_read_file_(uint8_t file_id, size_t length,
                                               std::vector<uint8_t> &out)
    {
      std::vector<uint8_t> apdu = {
          0x90, 0xBD, 0x00, 0x00, 0x07, file_id,
          0x00, 0x00, 0x00,
          (uint8_t)(length & 0xFF),
          (uint8_t)((length >> 8) & 0xFF),
          (uint8_t)((length >> 16) & 0xFF),
          0x00};
      uint8_t sw1, sw2;
      if (!desfire_apdu_(apdu, out, sw1, sw2))
        return false;
      if (!(sw1 == DESFIRE_SW1 && sw2 == DESFIRE_OK))
      {
        ESP_LOGW(TAG, "df_read_file_ SW:%02X%02X", sw1, sw2);
        return false;
      }
      // Card may append an 8-byte MAC — truncate to requested length
      if (out.size() > length)
        out.resize(length);
      return true;
    }

    // ═══════════════════════════════════════════════════════════════
    //  AES-128 portable (no external libs)
    // ═══════════════════════════════════════════════════════════════

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
    static uint8_t mul_(uint8_t a, uint8_t b)
    {
      return ((b & 1) * a) ^ ((b >> 1 & 1) * xtime_(a)) ^ ((b >> 2 & 1) * xtime_(xtime_(a))) ^
             ((b >> 3 & 1) * xtime_(xtime_(xtime_(a)))) ^
             ((b >> 4 & 1) * xtime_(xtime_(xtime_(xtime_(a)))));
    }

    void aes_key_exp_(const uint8_t *key, uint8_t *rk)
    {
      memcpy(rk, key, 16);
      for (int i = 4; i < 44; i++)
      {
        uint8_t tmp[4];
        memcpy(tmp, rk + (i - 1) * 4, 4);
        if (i % 4 == 0)
        {
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

    void aes_enc_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out)
    {
      uint8_t s[16];
      memcpy(s, in, 16);
      for (int i = 0; i < 16; i++)
        s[i] ^= rk[i];
      for (int r = 1; r <= 10; r++)
      {
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
        if (r < 10)
        {
          for (int c = 0; c < 4; c++)
          {
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

    void aes_dec_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out)
    {
      uint8_t s[16];
      memcpy(s, in, 16);
      for (int i = 0; i < 16; i++)
        s[i] ^= rk[10 * 16 + i];
      for (int r = 9; r >= 0; r--)
      {
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
        for (int c = 0; c < 4; c++)
        {
          uint8_t *col = s + c * 4, a = col[0], b = col[1], cc = col[2], d = col[3];
          col[0] = mul_(a, 0xe) ^ mul_(b, 0xb) ^ mul_(cc, 0xd) ^ mul_(d, 0x9);
          col[1] = mul_(a, 0x9) ^ mul_(b, 0xe) ^ mul_(cc, 0xb) ^ mul_(d, 0xd);
          col[2] = mul_(a, 0xd) ^ mul_(b, 0x9) ^ mul_(cc, 0xe) ^ mul_(d, 0xb);
          col[3] = mul_(a, 0xb) ^ mul_(b, 0xd) ^ mul_(cc, 0x9) ^ mul_(d, 0xe);
        }
      }
      memcpy(out, s, 16);
    }

    bool DesfireReaderComponent::aes_cbc_decrypt_(const uint8_t *in, size_t len, uint8_t *out)
    {
      if (len % 16 != 0)
        return false;
      uint8_t iv[16] = {0};
      for (size_t b = 0; b < len; b += 16)
      {
        aes_dec_block_(data_rk_, in + b, out + b);
        for (int i = 0; i < 16; i++)
          out[b + i] ^= iv[i];
        memcpy(iv, in + b, 16);
      }
      return true;
    }

    void DesfireReaderComponent::random_bytes_(uint8_t *buf, size_t len)
    {
      for (size_t i = 0; i < len; i++)
        buf[i] = (uint8_t)(random(256));
    }

  } // namespace desfire_reader
} // namespace esphome