#include "desfire_reader_secure.h"

#include <string.h>

static void xor_block(uint8_t *dst, const uint8_t *src, size_t len)
{
  for (size_t i = 0; i < len; i++)
    dst[i] ^= src[i];
}

void DesfireReader::rotate_left_(uint8_t *data, size_t len)
{
  uint8_t first = data[0];
  for (size_t i = 0; i < len - 1; i++)
    data[i] = data[i + 1];
  data[len - 1] = first;
}

bool DesfireReader::authenticate_aes(uint8_t key_no, const uint8_t *key)
{
  std::vector<uint8_t> rx;

  if (!desfire_apdu_(0xAA, {key_no}, rx))
    return false;

  if (rx.size() != 16)
    return false;

  uint8_t rndB[16];
  memcpy(rndB, rx.data(), 16);

  uint8_t iv[16];
  memset(iv, 0, 16);

  uint8_t rndB_dec[16];
  aes_cbc_decrypt_(key, iv, rndB, rndB_dec, 16);

  uint8_t rndA[16];
  for (int i = 0; i < 16; i++)
    rndA[i] = rand() & 0xFF;

  uint8_t rndB_rot[16];
  memcpy(rndB_rot, rndB_dec, 16);
  rotate_left_(rndB_rot, 16);

  uint8_t msg[32];
  memcpy(msg, rndA, 16);
  memcpy(msg + 16, rndB_rot, 16);

  memset(iv, 0, 16);

  uint8_t enc[32];
  aes_cbc_encrypt_(key, iv, msg, enc, 32);

  std::vector<uint8_t> tx(enc, enc + 32);

  if (!desfire_apdu_(0xAF, tx, rx))
    return false;

  if (rx.size() != 16)
    return false;

  uint8_t rndA_rot_card[16];

  aes_cbc_decrypt_(key, iv, rx.data(), rndA_rot_card, 16);

  uint8_t rndA_rot[16];
  memcpy(rndA_rot, rndA, 16);
  rotate_left_(rndA_rot, 16);

  if (memcmp(rndA_rot_card, rndA_rot, 16) != 0)
    return false;

  derive_session_key_(rndA, rndB_dec);

  memset(session_iv_, 0, 16);

  session_active_ = true;
  auth_key_no_ = key_no;

  return true;
}

void DesfireReader::derive_session_key_(const uint8_t *rndA, const uint8_t *rndB)
{
  session_key_[0] = rndA[0];
  session_key_[1] = rndA[1];
  session_key_[2] = rndA[2];
  session_key_[3] = rndA[3];

  session_key_[4] = rndB[0];
  session_key_[5] = rndB[1];
  session_key_[6] = rndB[2];
  session_key_[7] = rndB[3];

  session_key_[8] = rndA[12];
  session_key_[9] = rndA[13];
  session_key_[10] = rndA[14];
  session_key_[11] = rndA[15];

  session_key_[12] = rndB[12];
  session_key_[13] = rndB[13];
  session_key_[14] = rndB[14];
  session_key_[15] = rndB[15];
}

bool DesfireReader::read_file_secure(uint8_t file_no,
                                     uint32_t offset,
                                     uint32_t length,
                                     std::vector<uint8_t> &out)
{
  if (!session_active_)
    return false;

  std::vector<uint8_t> cmd;

  cmd.push_back(file_no);

  cmd.push_back(offset & 0xFF);
  cmd.push_back((offset >> 8) & 0xFF);
  cmd.push_back((offset >> 16) & 0xFF);

  cmd.push_back(length & 0xFF);
  cmd.push_back((length >> 8) & 0xFF);
  cmd.push_back((length >> 16) & 0xFF);

  uint8_t cmac[16];

  aes_cmac_(session_key_, cmd.data(), cmd.size(), cmac);

  std::vector<uint8_t> tx = cmd;

  tx.insert(tx.end(), cmac, cmac + 8);

  std::vector<uint8_t> rx;

  if (!desfire_apdu_(0xBD, tx, rx))
    return false;

  if (rx.size() < 8)
    return false;

  size_t enc_len = rx.size() - 8;

  uint8_t iv[16];
  memcpy(iv, session_iv_, 16);

  std::vector<uint8_t> dec(enc_len);

  aes_cbc_decrypt_(session_key_,
                   iv,
                   rx.data(),
                   dec.data(),
                   enc_len);

  out = dec;

  return true;
}