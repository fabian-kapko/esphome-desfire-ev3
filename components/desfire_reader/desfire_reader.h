#include "desfire_reader.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <string.h>

namespace esphome {
namespace desfire_reader {

void DesfireReaderComponent::secure_zero_(volatile uint8_t *buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++)
    buf[i] = 0;
}

// ═══════════════════════════════════════════════════════════════
//  I2C bus recovery — clock out stuck slave, re-init hardware
// ═══════════════════════════════════════════════════════════════

void DesfireReaderComponent::recover_i2c_bus_() {
#ifdef USE_ESP32
  if (sda_pin_ < 0 || scl_pin_ < 0) {
    ESP_LOGW(TAG, "I2C bus recovery skipped — SDA/SCL pins not configured");
    delay(10);
    return;
  }

  ESP_LOGD(TAG, "I2C bus recovery — clocking out stuck state (SDA=%d SCL=%d)",
           sda_pin_, scl_pin_);

  gpio_num_t sda = (gpio_num_t)sda_pin_;
  gpio_num_t scl = (gpio_num_t)scl_pin_;

  gpio_reset_pin(scl);
  gpio_set_direction(scl, GPIO_MODE_OUTPUT_OD);
  gpio_set_level(scl, 1);

  gpio_reset_pin(sda);
  gpio_set_direction(sda, GPIO_MODE_INPUT);
  gpio_set_pull_mode(sda, GPIO_PULLUP_ONLY);

  for (int i = 0; i < 9; i++) {
    gpio_set_level(scl, 0);
    delayMicroseconds(5);
    gpio_set_level(scl, 1);
    delayMicroseconds(5);
    if (gpio_get_level(sda) == 1)
      break;
  }

  gpio_set_direction(sda, GPIO_MODE_OUTPUT_OD);
  gpio_set_level(sda, 0);
  delayMicroseconds(5);
  gpio_set_level(scl, 1);
  delayMicroseconds(5);
  gpio_set_level(sda, 1);
  delayMicroseconds(5);

  gpio_reset_pin(sda);
  gpio_reset_pin(scl);

  uint8_t dummy;
  this->read(&dummy, 1);
#else
  delay(10);
#endif
}

void DesfireReaderComponent::pn532_wakeup_() {
  static const uint8_t wakeup[] = {
      0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  this->write(wakeup, sizeof(wakeup));
}

// ═══════════════════════════════════════════════════════════════
//  PN532 I2C — non-blocking primitives
// ═══════════════════*
