#include "dht11.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"  // IWYU pragma: keep.
#include "esp_err.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep.
#include "util.h"

static bool dht11_await_pin_state(gpio_num_t pin, uint8_t timeout,
                                  int expected_pin_state, uint8_t* duration) {
  for (uint8_t i = 0; i < timeout; i += 2) {
    spin_delay(2);
    if (gpio_get_level(pin) == expected_pin_state) {
      if (duration) {
        *duration = i;
      }

      return true;
    }
  }

  return false;
}

static inline bool dht11_fetch_data(gpio_num_t pin, uint8_t data[40]) {
  gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
  gpio_set_level(pin, 0);

  spin_delay(20000);

  gpio_set_level(pin, 1);
  gpio_set_direction(pin, GPIO_MODE_INPUT);

  if (!dht11_await_pin_state(pin, 40, 0, NULL)) {
    return false;
  }

  if (!dht11_await_pin_state(pin, 88, 1, NULL)) {
    return false;
  }

  if (!dht11_await_pin_state(pin, 88, 0, NULL)) {
    return false;
  }

  uint8_t lo;
  uint8_t hi;
  for (uint8_t i = 0; i < 40; ++i) {
    if (!dht11_await_pin_state(pin, 65, 1, &lo)) {
      return false;
    }

    if (!dht11_await_pin_state(pin, 75, 0, &hi)) {
      return false;
    }

    uint8_t b = i / 8;
    uint8_t m = i % 8;
    if (!m) {
      data[b] = 0;
    }

    data[b] |= (hi > lo) << (7 - m);
  }

  return true;
}

float dht11_humidity(uint16_t humidity) {
  return (humidity >> 8) + (humidity & 0xff) * 0.1;
}

float dht11_temperature(uint16_t temperature) {
  uint8_t f = temperature & 0xff;
  float t = (temperature >> 8) + (f & 0x7f) * 0.1;
  return f & 0x80 ? -t : t;
}

esp_err_t dht11_read(gpio_num_t pin, uint16_t* humidity,
                     uint16_t* temperature) {
  gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
  gpio_set_level(pin, 1);

  uint8_t data[40] = {0};

  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);
  bool ok = dht11_fetch_data(pin, data);
  portEXIT_CRITICAL(&mux);

  if (!ok) {
    return ESP_ERR_TIMEOUT;
  }

  if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xff)) {
    return ESP_ERR_INVALID_CRC;
  }

  if (humidity) {
    *humidity = data[0] << 8 | data[1];
  }

  if (*temperature) {
    *temperature = data[2] << 8 | data[3];
  }

  return ESP_OK;
}
