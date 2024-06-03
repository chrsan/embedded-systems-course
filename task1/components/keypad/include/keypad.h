#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

struct Keypad;

struct KeypadEvent {
  uint8_t row_index;
  uint8_t col_index;
};

struct KeypadConfig {
  const gpio_num_t* row_pins;
  const gpio_num_t* col_pins;
  uint8_t num_row_pins;
  uint8_t num_col_pins;
};

esp_err_t keypad_init(const struct KeypadConfig* config,
                      struct Keypad** keypad);

void keypad_deinit(struct Keypad* keypad);

bool keypad_has_event(const struct Keypad* keypad);

bool keypad_get_event(struct Keypad* keypad, struct KeypadEvent* event);

void keypad_scan(struct Keypad* keypad);
