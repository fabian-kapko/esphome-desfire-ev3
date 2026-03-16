#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <string.h>
#include <vector>

#ifdef USE_ESP32
#include <esp_random.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#endif

namespace esphome {
namespace desfire_reader {

static const char *const TAG = "desfire";

static const uint8_t DESFIRE_SW1 = 0x91;
static const uint8_t DESFIRE_OK = 0x00;
static const uint8_t DESFIRE_MORE_FRAMES = 0xAF;

static const uint8_t PN532_CMD_IN_DATA_EXCHANGE = 0x40;
static const uint8_t PN532_CMD_IN_LIST_PASSIVE = 0x4A;
static const uint8_t PN532_CMD_IN_RELEASE = 0x52;
// Must be uint16_t: a uint8_t constant wraps silently to 0 if set above 255,
// causing immediate stack corruption. 128 bytes gives comfortable headroom
// over the 64-byte max used by current APDUs and prevents silent truncation
// of longer PN532 responses (e.g. multi-block ReadData, extended UID).
static const uint16_t PN532_BUF_SIZE = 128;

static const uint16_t COOLDOWN_SUCCESS_MS = 150;
static const uint16_t COOLDOWN_RETRY_MS = 50;
static const uint16_t COOLDOWN_FAIL_BASE_MS = 100;
static const uint32_t COOLDOWN_FAIL_MAX_MS = 5000;
static const uint8_t NO_CARD_THRESHOLD = 2;
static const uint8_t MAX_RETRIES = 2;

// Proximity check: number of random bytes exchanged (standard = 7)
static const uint8_t PC_NUM_ROUNDS = 7;

// File communication modes (matches DESFire spec)
enum class CommMode : uint8_t {
  PLAIN = 0,   // No protection after auth (legacy, backward compat)
  MAC   = 1,   // Data + CMAC integrity
  FULL  = 2,   // AES-CBC encrypted + CMAC
};

void aes_key_exp_(const uint8_t *key, uint8_t *rk);
void aes_enc_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out);
void aes_dec_block_(const uint8_t *rk, const uint8_t *in, uint8_t *out);
void aes_cmac_(const uint8_t *rk, const uint8_t *msg, uint16_t msg_len,
               uint8_t *mac);
void aes_cmac_truncate_(const uint8_t *full_mac, uint8_t *mac8);

enum class NfcState : uint8_t {
  IDLE,
  BUS_RECOVERY,
  DETECT_WAIT_ACK,
  DETECT_WAIT_RESP,
  SELECT_WAIT_ACK,
  SELECT_WAIT_RESP,
  AUTH1_WAIT_ACK,
  AUTH1_WAIT_RESP,
  AUTH2_WAIT_ACK,
  AUTH2_WAIT_RESP,
  // ── Proximity check (after auth, before read) ──
  PC_PREP_WAIT_ACK,
  PC_PREP_WAIT_RESP,
  PC_ROUND_WAIT_ACK,
  PC_ROUND_WAIT_RESP,
  PC_VERIFY_WAIT_ACK,
  PC_VERIFY_WAIT_RESP,
  // ── Data read ──
  READ_WAIT_ACK,
  READ_WAIT_RESP,
  PUBLISH,
  RELEASE_WAIT_ACK,
  RELEASE_WAIT_RESP,
};

class DesfireReaderComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_app_id(uint8_t a, uint8_t b, uint8_t c) {
    app_id_[0] = a;
    app_id_[1] = b;
    app_id_[2] = c;
  }

  void set_app_key(const uint8_t *key) { memcpy(app_key_, key, 16); }
  void set_data_key(const uint8_t *key) { memcpy(data_key_, key, 16); }

  void set_app_key(const std::vector<uint8_t> &key) {
    if (key.size() < 16) return;
    for (int i = 0; i < 16; i++) app_key_[i] = key[i];
  }

  void set_data_key(const std::vector<uint8_t> &key) {
    if (key.size() < 16) return;
    for (int i = 0; i < 16; i++) data_key_[i] = key[i];
    has_data_key_ = true;
  }

  void set_aes_key(const std::vector<uint8_t> &key) { set_data_key(key); }
  void set_des_key(const std::vector<uint8_t> &) {}

  void set_result_sensor(text_sensor::TextSensor *s) { result_sensor_ = s; }
  void set_auth_sensor(binary_sensor::BinarySensor *s) { auth_sensor_ = s; }
  void set_uid_sensor(text_sensor::TextSensor *s) { uid_sensor_ = s; }
  void set_nonauthorised_uid_sensor(text_sensor::TextSensor *s) {
    nonauthorised_uid_sensor_ = s;
  }

  void set_sda_pin(int pin) { sda_pin_ = pin; }
  void set_scl_pin(int pin) { scl_pin_ = pin; }

  void set_proximity_check(bool en) { proximity_check_enabled_ = en; }
  void set_comm_mode(CommMode m) { comm_mode_ = m; }

 protected:
  // ── PN532 I2C primitives ──
  bool write_command_(const uint8_t *cmd, uint8_t cmd_len);
  bool try_read_ack_();
  bool try_read_response_(uint8_t command, uint8_t *resp, uint8_t resp_cap,
                          uint8_t &resp_len);

  // ── DESFire APDU helpers ──
  bool send_desfire_apdu_(const uint8_t *apdu, uint8_t apdu_len);
  bool read_desfire_apdu_(uint8_t *response, uint8_t resp_cap,
                          uint8_t &resp_len, uint8_t &sw1, uint8_t &sw2);

  // ── AES helpers ──
  bool aes_cbc_decrypt_with_key_(const uint8_t *rk, const uint8_t *in,
                                  uint8_t len, const uint8_t *iv, uint8_t *out);
  bool aes_cbc_encrypt_with_key_(const uint8_t *rk, const uint8_t *in,
                                  uint8_t len, const uint8_t *iv, uint8_t *out);
  void random_bytes_(uint8_t *buf, uint8_t len);
  static void secure_zero_(volatile uint8_t *buf, uint8_t len);

  // ── EV2 session key derivation ──
  void derive_session_keys_();
  void compute_ev2_iv_(bool for_response, uint8_t *iv);
  void compute_cmd_mac_(uint8_t cmd, const uint8_t *cmd_data,
                        uint8_t data_len, uint8_t *mac8);
  bool verify_resp_mac_(uint8_t sw2, const uint8_t *resp_data,
                        uint8_t data_len, const uint8_t *mac8);

  // ── State machine flow helpers ──
  void format_uid_(const uint8_t *uid_bytes, uint8_t uid_len, char *out);
  void publish_uid_(const char *uid_str);
  void publish_nonauthorised_uid_(const char *uid_str);
  void note_nonauthorised_failure_();
  void publish_auth_(bool state);
  void publish_result_(const char *str);

  void handle_fail_();
  void handle_retry_();
  void reset_to_idle_(uint32_t cooldown_ms);
  void clear_card_state_();
  void start_release_();
  void start_select_app_();
  void start_proximity_check_();
  void start_read_file_();
  void recover_i2c_bus_();
  void pn532_wakeup_();

  // ── Configuration ──
  uint8_t app_id_[3]{0xA1, 0xB2, 0xC3};
  uint8_t app_key_[16]{};
  uint8_t data_key_[16]{};
  uint8_t app_rk_[176]{};
  uint8_t data_rk_[176]{};
  bool has_data_key_{false};
  CommMode comm_mode_{CommMode::PLAIN};
  bool proximity_check_enabled_{true};

  int sda_pin_{-1};
  int scl_pin_{-1};

  // ── Sensors ──
  text_sensor::TextSensor *result_sensor_{nullptr};
  text_sensor::TextSensor *uid_sensor_{nullptr};
  text_sensor::TextSensor *nonauthorised_uid_sensor_{nullptr};
  binary_sensor::BinarySensor *auth_sensor_{nullptr};

  char last_uid_[24]{};
  char last_nonauthorised_uid_[24]{};
  char last_result_[50]{};
  bool last_auth_{false};

  // ── State machine ──
  NfcState state_{NfcState::IDLE};
  uint32_t cooldown_until_{0};
  uint32_t state_entered_at_{0};
  bool update_requested_{false};

  uint8_t prev_uid_[7]{};
  uint8_t prev_uid_len_{0};
  bool card_present_{false};
  uint8_t no_card_count_{0};
  uint8_t consecutive_fails_{0};
  uint8_t retry_count_{0};
  uint8_t nonauthorised_fail_count_{0};

  // ── Card interaction buffers ──
  char current_uid_str_[24]{};
  uint8_t current_uid_bytes_[7]{};
  uint8_t current_uid_len_{0};
  uint8_t enc_rnd_b_[16]{};
  uint8_t rnd_a_[16]{};
  uint8_t rnd_b_dec_[16]{};   // Decrypted RndB — kept for session key derivation
  uint8_t token_[32]{};
  uint8_t raw_file_[64]{};    // Increased for encrypted + CMAC response
  uint8_t raw_file_len_{0};

  // ── EV2 secure channel state ──
  uint8_t ses_auth_enc_key_[16]{};
  uint8_t ses_auth_mac_key_[16]{};
  uint8_t ses_auth_enc_rk_[176]{};
  uint8_t ses_auth_mac_rk_[176]{};
  uint8_t ti_[4]{};              // Transaction Identifier from auth
  uint16_t cmd_ctr_{0};          // Command counter (increments per cmd-resp pair)
  bool ev2_authenticated_{false};

  // ── Proximity check state ──
  uint8_t pc_rnd_r_[PC_NUM_ROUNDS]{};  // Reader's random bytes
  uint8_t pc_rnd_c_[PC_NUM_ROUNDS]{};  // Card's random bytes
  uint8_t pc_current_round_{0};

  static const uint16_t ACK_TIMEOUT_MS = 50;
  static const uint16_t RESP_TIMEOUT_MS = 150;
  static const uint16_t DETECT_TIMEOUT_MS = 200;
  static const uint16_t BUS_RECOVERY_MS = 30;
};

}  // namespace desfire_reader
}  // namespace esphome