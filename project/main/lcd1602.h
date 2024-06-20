#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"  // IWYU pragma: keep.
#include "esp_err.h"

esp_err_t lcd1602_init(i2c_port_num_t port, gpio_num_t sda_pin,
                       gpio_num_t scl_pin, uint8_t address);

enum Lcd1602Row {
  LCD1602_ROW_1 = 0x00,
  LCD1602_ROW_2 = 0x40,
};

esp_err_t lcd1602_set_cursor(enum Lcd1602Row row, uint8_t col);

esp_err_t lcd1602_toggle_backlight();

esp_err_t lcd1602_write_data(uint8_t value);
