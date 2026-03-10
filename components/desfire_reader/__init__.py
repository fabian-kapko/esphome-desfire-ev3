#pragma once

#include <vector>
#include <stdint.h>

class DesfireReader {
 public:
  bool authenticate_aes(uint8_t key_no, const uint8_t *key);
  bool read_file_secure(uint8_t file_no, uint32_t offset, uint32_t length, std::vector<uint8_t> &out);

 private:
  bool desfire_apdu_(uint8_t ins,
                     const std::vector<uint8_t> &tx,
                     std::vector<uint8_t> &rx);

  void derive_session_key_(const uint8_t *rndA, const uint8_t *rndB);

  void aes_cbc_encrypt_(const uint8_t *key,
                        uint8_t *iv,
                        const uint8_t *in,
                        uint8_t *out,
                        size_t len);

  void aes_cbc_decrypt_(const uint8_t *key,
                        uint8_t *iv,
                        const uint8_t *in,
                        uint8_t *out,
                        size_t len);

  void aes_cmac_(const uint8_t *key,
                 const uint8_t *data,
                 size_t len,
                 uint8_t *out);

  void rotate_left_(uint8_t *data, size_t len);

  bool session_active_{false};

  uint8_t session_key_[16];
  uint8_t session_iv_[16];

  uint8_t auth_key_no_{0};
};