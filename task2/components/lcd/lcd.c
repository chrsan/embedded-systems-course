#include "lcd.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_timer.h"

#define TAG "lcd"
#define TIMEOUT 50

struct Lcd {
  i2c_master_bus_handle_t bus_handle;
  i2c_master_dev_handle_t dev_handle;
  uint8_t state;
};

// N.B. The vTaskDelay does not seem to work for short delays.
// See: https://esp32developer.com/programming-in-c-c/delays/delays
/*
static void lcd_delay(TickType_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}
*/

#define NOP() asm volatile("nop")

// See:
// https://github.com/espressif/arduino-esp32/blob/cc50d90ce4ff7414015dfc597eb083d427b86faf/cores/esp32/esp32-hal-misc.c#L206
static void lcd_delay(uint64_t ms) {
  uint64_t m = (uint64_t)esp_timer_get_time();
  uint64_t e = (m + ms * 1000);
  if (m > e) {
    while ((uint64_t)esp_timer_get_time() > e) {
      NOP();
    }
  }

  while ((uint64_t)esp_timer_get_time() < e) {
    NOP();
  }
}

static esp_err_t lcd_i2c_write(const struct Lcd* lcd, uint8_t value) {
  if (lcd->state & LCD_STATE_BACKLIGHT_ON) {
    value |= 0x08;
  }

  return i2c_master_transmit(lcd->dev_handle, &value, 1, TIMEOUT);
}

static esp_err_t lcd_pulse(const struct Lcd* lcd, uint8_t data) {
  esp_err_t ret;
  if ((ret = lcd_i2c_write(lcd, data | 0x04)) != ESP_OK) {
    return ret;
  }

  lcd_delay(1);
  if ((ret = lcd_i2c_write(lcd, data & ~0x04)) != ESP_OK) {
    return ret;
  }

  lcd_delay(1);
  return ESP_OK;
}

static esp_err_t lcd_write_nibble(const struct Lcd* lcd, uint8_t data) {
  esp_err_t ret;
  if ((ret = lcd_i2c_write(lcd, data)) != ESP_OK) {
    return ret;
  }

  return lcd_pulse(lcd, data);
}

enum LcdMode {
  LCD_MODE_COMMAND = 0x00,
  LCD_MODE_DATA = 0x01,
};

static esp_err_t lcd_write(const struct Lcd* lcd, uint8_t data,
                           enum LcdMode mode) {
  uint8_t hi = data & 0xf0;
  uint8_t lo = (data << 4) & 0xf0;

  esp_err_t ret;
  if ((ret = lcd_write_nibble(lcd, hi | mode)) != ESP_OK) {
    return ret;
  }

  return lcd_write_nibble(lcd, lo | mode);
}

esp_err_t lcd_init(const struct LcdConfig* config, uint8_t address,
                   struct Lcd** lcd) {
  ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG,
                      "config must not be null");
  ESP_RETURN_ON_FALSE(config->port < I2C_NUM_MAX, ESP_FAIL, TAG,
                      "invalid I2C port number: %d", config->port);
  ESP_RETURN_ON_FALSE(lcd, ESP_ERR_INVALID_ARG, TAG, "lcd must not be null");

  struct Lcd* l = calloc(1, sizeof(struct Lcd));
  ESP_RETURN_ON_FALSE(l, ESP_ERR_NO_MEM, TAG, "Lcd alloc error");

  esp_err_t ret = ESP_OK;

  i2c_master_bus_config_t bus_config = {
      .i2c_port = config->port,
      .sda_io_num = config->sda_pin,
      .scl_io_num = config->scl_pin,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  ESP_GOTO_ON_ERROR(i2c_new_master_bus(&bus_config, &l->bus_handle), err, TAG,
                    "error allocating I2C master bus");

  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = address,
      .scl_speed_hz = 100000,
  };

  ESP_GOTO_ON_ERROR(
      i2c_master_bus_add_device(l->bus_handle, &dev_config, &l->dev_handle),
      err, TAG, "error adding I2C master bus device");

  *lcd = l;
  return ESP_OK;
err:
  if (l->bus_handle) {
    i2c_del_master_bus(l->bus_handle);
  }

  free(l);
  return ret;
}

void lcd_deinit(struct Lcd* lcd) {
  if (lcd) {
    if (lcd->dev_handle) {
      i2c_master_bus_rm_device(lcd->dev_handle);
    }

    if (lcd->bus_handle) {
      i2c_del_master_bus(lcd->bus_handle);
    }

    free(lcd);
  }
}

#define ENTRY_MODE_SET 0x04
#define FUNCTION_SET 0x20

esp_err_t lcd_reset(struct Lcd* lcd, uint8_t lcd_state) {
  // See datascheet on page 16.
  lcd_delay(50);

  // Put the LCD in 4 bit mode.
  esp_err_t ret;
  if ((ret = lcd_write_nibble(lcd, 0x03 << 4)) != ESP_OK) {
    return ret;
  }

  lcd_delay(5);
  if ((ret = lcd_write_nibble(lcd, 0x03 << 4)) != ESP_OK) {
    return ret;
  }

  lcd_delay(5);
  if ((ret = lcd_write_nibble(lcd, 0x03 << 4)) != ESP_OK) {
    return ret;
  }

  lcd_delay(1);
  if ((ret = lcd_write_nibble(lcd, 0x02 << 4)) != ESP_OK) {
    return ret;
  }

  // Two rows.
  if ((ret = lcd_write(lcd, FUNCTION_SET | 0x08, LCD_MODE_COMMAND)) != ESP_OK) {
    return ret;
  }

  if ((ret = lcd_toggle_state(lcd, lcd_state)) != ESP_OK) {
    return ret;
  }

  // Clear the display.
  if ((ret = lcd_clear(lcd)) != ESP_OK) {
    return ret;
  }

  // Set the entry mode.
  return lcd_write(lcd, ENTRY_MODE_SET | 0x02, LCD_MODE_COMMAND);
}

#define CLEAR 0x01

esp_err_t lcd_clear(const struct Lcd* lcd) {
  esp_err_t ret = lcd_write(lcd, CLEAR, LCD_MODE_COMMAND);
  if (ret == ESP_OK) {
    lcd_delay(2);
  }

  return ret;
}

#define RETURN_HOME 0x02

esp_err_t lcd_return_home(const struct Lcd* lcd) {
  esp_err_t ret = lcd_write(lcd, RETURN_HOME, LCD_MODE_COMMAND);
  if (ret == ESP_OK) {
    lcd_delay(2);
  }

  return ret;
}

#define SET_DISPLAY_DATA_RAM_ACCESS 0x80

esp_err_t lcd_set_cursor(const struct Lcd* lcd, enum LcdRow row, uint8_t col) {
  return lcd_write(lcd, SET_DISPLAY_DATA_RAM_ACCESS | (col + row),
                   LCD_MODE_COMMAND);
}

#define DISPLAY_CONTROL 0x08

esp_err_t lcd_toggle_state(struct Lcd* lcd, uint8_t lcd_state) {
  lcd->state = lcd_state;
  return lcd_write(
      lcd, DISPLAY_CONTROL | (lcd_state & LCD_STATE_DISPLAY_ON) ? 0x04 : 0x00,
      LCD_MODE_COMMAND);
}

esp_err_t lcd_write_data(const struct Lcd* lcd, uint8_t value) {
  return lcd_write(lcd, value, LCD_MODE_DATA);
}
