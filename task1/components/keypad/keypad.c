#include "keypad.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_err.h"

#define TAG "keypad"

struct Keypad {
  const struct KeypadConfig* config;
  uint32_t next_event;
  uint32_t prev_event;
};

esp_err_t keypad_init(const struct KeypadConfig* config,
                      struct Keypad** keypad) {
  ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG,
                      "config must not be null");
  ESP_RETURN_ON_FALSE(keypad, ESP_ERR_INVALID_ARG, TAG,
                      "keypad must not be null");

  struct Keypad* kp = calloc(1, sizeof(struct Keypad));
  ESP_RETURN_ON_FALSE(kp, ESP_ERR_NO_MEM, TAG, "keypad alloc error");
  kp->config = config;

  esp_err_t ret = ESP_OK;

  gpio_config_t io_conf = {
      .pin_bit_mask = 0,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  gpio_num_t pin;
  for (uint8_t i = 0; i < config->num_row_pins; ++i) {
    pin = config->row_pins[i];
    io_conf.pin_bit_mask = 1ULL << pin;
    ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG,
                      "error configuring row pin %d", pin);
    ESP_GOTO_ON_ERROR(gpio_set_level(pin, 1), err, TAG,
                      "error setting level on row pin %d", pin);
  }

  io_conf.mode = GPIO_MODE_INPUT;
  for (uint8_t i = 0; i < config->num_col_pins; ++i) {
    pin = config->col_pins[i];
    io_conf.pin_bit_mask = 1ULL << pin;
    ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG,
                      "error configuring column pin %d", pin);
  }

  *keypad = kp;
  return ESP_OK;
err:
  free(kp);
  return ret;
}

void keypad_deinit(struct Keypad* keypad) {
  if (keypad) {
    gpio_num_t pin;
    for (uint8_t i = 0; i < keypad->config->num_row_pins; ++i) {
      pin = keypad->config->row_pins[i];
      gpio_reset_pin(pin);
    }

    for (uint8_t i = 0; i < keypad->config->num_col_pins; ++i) {
      pin = keypad->config->col_pins[i];
      gpio_reset_pin(pin);
    }

    free(keypad);
  }
}

bool keypad_has_event(const struct Keypad* keypad) {
  return keypad && (keypad->next_event >> 16) != 0;
}

bool keypad_get_event(struct Keypad* keypad, struct KeypadEvent* event) {
  if (!keypad_has_event(keypad)) {
    return false;
  }

  if (event) {
    event->row_index = (keypad->next_event >> 8) & 0xFF;
    event->col_index = (keypad->next_event & 0xFF);
  }

  keypad->next_event = 0;
  return true;
}

void keypad_scan(struct Keypad* keypad) {
  if (!keypad) {
    return;
  }

  uint32_t event = 0;

  gpio_num_t row_pin;
  gpio_num_t col_pin;
  for (uint8_t row = 0; row < keypad->config->num_row_pins; ++row) {
    row_pin = keypad->config->row_pins[row];
    gpio_set_level(row_pin, 0);

    for (uint8_t col = 0; col < keypad->config->num_col_pins; ++col) {
      col_pin = keypad->config->col_pins[col];
      if (gpio_get_level(col_pin) == 0) {
        event = 1 << 16 | row << 8 | col;
        break;
      }
    }

    gpio_set_level(row_pin, 1);
  }

  if (keypad->prev_event != event) {
    keypad->prev_event = event;
    if (event != 0) {
      keypad->next_event = event;
    }
  }
}
