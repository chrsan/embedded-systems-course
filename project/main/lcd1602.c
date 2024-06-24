#include "lcd1602.h"

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "util.h"

#define TAG "lcd1602"

#define LCD1602_CLEAR 0x01
#define LCD1602_ENTRY_MODE_SET 0x04
#define LCD1602_DISPLAY_CONTROL 0x08
#define LCD1602_FUNCTION_SET 0x20
#define LCD1602_SET_DISPLAY_DATA_RAM_ACCESS 0x80

#define LCD1602_BACKLIGHT 0x08

i2c_master_bus_handle_t lcd1602_bus_handle = NULL;
i2c_master_dev_handle_t lcd1602_dev_handle = NULL;

bool lcd1602_backlight_on = true;

static esp_err_t lcd1602_i2c_write(uint8_t value) {
  if (lcd1602_backlight_on) {
    value |= LCD1602_BACKLIGHT;
  }

  return i2c_master_transmit(lcd1602_dev_handle, &value, 1, 1000);
}

static esp_err_t lcd1602_pulse(uint8_t data) {
  esp_err_t ret;
  if ((ret = lcd1602_i2c_write(data | 0x04)) != ESP_OK) {
    return ret;
  }

  spin_delay(1000);
  if ((ret = lcd1602_i2c_write(data & ~0x04)) != ESP_OK) {
    return ret;
  }

  spin_delay(1000);
  return ESP_OK;
}

static esp_err_t lcd1602_write_nibble(uint8_t data) {
  esp_err_t ret;
  if ((ret = lcd1602_i2c_write(data)) != ESP_OK) {
    return ret;
  }

  return lcd1602_pulse(data);
}

static esp_err_t lcd1602_write(uint8_t data, bool is_command) {
  uint8_t mode = is_command ? 0x00 : 0x01;
  uint8_t hi = (data & 0xf0) | mode;
  uint8_t lo = ((data << 4) & 0xf0) | mode;

  esp_err_t ret;
  if ((ret = lcd1602_write_nibble(hi)) != ESP_OK) {
    return ret;
  }

  return lcd1602_write_nibble(lo);
}

esp_err_t lcd1602_init(i2c_port_num_t port, gpio_num_t sda_pin,
                       gpio_num_t scl_pin, uint8_t address) {
  i2c_master_bus_config_t bus_config = {
      .i2c_port = port,
      .sda_io_num = sda_pin,
      .scl_io_num = scl_pin,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  esp_err_t ret = ESP_OK;
  if ((ret = i2c_new_master_bus(&bus_config, &lcd1602_bus_handle)) != ESP_OK) {
    goto err;
  }

  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = address,
      .scl_speed_hz = 100000,
  };

  if ((ret = i2c_master_bus_add_device(lcd1602_bus_handle, &dev_config,
                                       &lcd1602_dev_handle)) != ESP_OK) {
    goto err;
  }

  spin_delay(50000);

  if ((ret = lcd1602_write_nibble(0x03 << 4)) != ESP_OK) {
    goto err;
  }

  spin_delay(5000);
  if ((ret = lcd1602_write_nibble(0x03 << 4)) != ESP_OK) {
    goto err;
  }

  spin_delay(5000);
  if ((ret = lcd1602_write_nibble(0x03 << 4)) != ESP_OK) {
    goto err;
  }

  spin_delay(1000);
  if ((ret = lcd1602_write_nibble(0x02 << 4)) != ESP_OK) {
    goto err;
  }

  if ((ret = lcd1602_write(LCD1602_FUNCTION_SET | 0x08, true)) != ESP_OK) {
    goto err;
  }

  if ((ret = lcd1602_write(LCD1602_DISPLAY_CONTROL | 0x04, true)) != ESP_OK) {
    goto err;
  }

  if ((ret = lcd1602_write(LCD1602_CLEAR, true)) != ESP_OK) {
    goto err;
  }

  spin_delay(2000);

  if ((ret = lcd1602_write(LCD1602_ENTRY_MODE_SET | 0x02, true)) != ESP_OK) {
    goto err;
  }

  return ESP_OK;
err:
  if (lcd1602_dev_handle) {
    i2c_master_bus_rm_device(lcd1602_dev_handle);
  }

  if (lcd1602_bus_handle) {
    i2c_del_master_bus(lcd1602_bus_handle);
  }

  return ret;
}

esp_err_t lcd1602_set_cursor(enum Lcd1602Row row, uint8_t col) {
  return lcd1602_write(LCD1602_SET_DISPLAY_DATA_RAM_ACCESS | (col + row), true);
}

esp_err_t lcd1602_toggle_backlight() {
  lcd1602_backlight_on = !lcd1602_backlight_on;
  return lcd1602_i2c_write(0);
}

esp_err_t lcd1602_write_data(uint8_t value) {
  return lcd1602_write(value, false);
}
