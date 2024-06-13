#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_types.h"
#include "esp_err.h"

struct Lcd;

struct LcdConfig {
  i2c_port_num_t port;
  gpio_num_t sda_pin;
  gpio_num_t scl_pin;
};

esp_err_t lcd_init(const struct LcdConfig* config, uint8_t address,
                   struct Lcd** lcd);

void lcd_deinit(struct Lcd* lcd);

#define LCD_STATE_BACKLIGHT_ON 0x01
#define LCD_STATE_DISPLAY_ON 0x02

esp_err_t lcd_reset(struct Lcd* lcd, uint8_t lcd_state);

esp_err_t lcd_clear(const struct Lcd* lcd);

esp_err_t lcd_return_home(const struct Lcd* lcd);

enum LcdRow {
  LCD_ROW_1 = 0x00,
  LCD_ROW_2 = 0x40,
};

esp_err_t lcd_set_cursor(const struct Lcd* lcd, enum LcdRow row, uint8_t col);

esp_err_t lcd_toggle_state(struct Lcd* lcd, uint8_t state);

esp_err_t lcd_write_data(const struct Lcd* lcd, uint8_t value);
