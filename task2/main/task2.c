#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep.
#include "freertos/task.h"
#include "lcd.h"
#include "mpu6050.h"

#define LCD_ADDRESS 0x27  // 0x3f
#define LCD_I2C_PORT I2C_NUM_0
#define LCD_SDA_PIN GPIO_NUM_13
#define LCD_SCL_PIN GPIO_NUM_14

#define MPU6050_ADDRESS 0x68
#define MPU6050_I2C_PORT I2C_NUM_1
#define MPU6050_SDA_PIN GPIO_NUM_15
#define MPU6050_SCL_PIN GPIO_NUM_2

#define BUTTON_PREV_PAGE_PIN GPIO_NUM_34
#define BUTTON_NEXT_PAGE_PIN GPIO_NUM_35

static struct Lcd* lcd = NULL;

static esp_err_t init_lcd() {
  struct LcdConfig conf = {
      .port = LCD_I2C_PORT,
      .sda_pin = LCD_SDA_PIN,
      .scl_pin = LCD_SCL_PIN,
  };

  return lcd_init(&conf, LCD_ADDRESS, &lcd);
}

#define MPU6050_ACCEL MPU6050_ACCEL_FS_4G
#define MPU6050_GYRO MPU6050_GYRO_FS_500DPS

static struct Mpu6050* mpu6050 = NULL;

static esp_err_t init_mpu6050() {
  struct Mpu6050Config conf = {
      .port = MPU6050_I2C_PORT,
      .sda_pin = MPU6050_SDA_PIN,
      .scl_pin = MPU6050_SCL_PIN,
      .accel_config = MPU6050_ACCEL,
      .gyro_config = MPU6050_GYRO,
  };

  return mpu6050_init(&conf, MPU6050_ADDRESS, &mpu6050);
}

static esp_err_t init_buttons() {
  gpio_config_t conf = {
      .pin_bit_mask =
          (1ULL << BUTTON_PREV_PAGE_PIN) | (1ULL << BUTTON_NEXT_PAGE_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  return gpio_config(&conf);
}

#define BUF_SIZE 16
#define HEADER_FORMAT "%16s"
#define NUM_FORMAT "%15.3f"

static char buf[BUF_SIZE + 1];

static struct Mpu6050Value mpu6050_value;

static esp_err_t update_value(uint8_t page) {
  esp_err_t ret;
  if (page < 3) {
    ret = mpu6050_get_accel(mpu6050, &mpu6050_value);
  } else {
    ret = mpu6050_get_gyro(mpu6050, &mpu6050_value);
  }

  if (ret != ESP_OK) {
    return ret;
  }

  if ((ret = lcd_set_cursor(lcd, LCD_ROW_1, 0)) != ESP_OK) {
    return ret;
  }

  const char* header;
  float value;
  switch (page) {
    case 0:
      header = "Accelerometer X:";
      value = mpu6050_value.x;
      break;
    case 1:
      header = "Accelerometer Y:";
      value = mpu6050_value.y;
      break;
    case 2:
      header = "Accelerometer Z:";
      value = mpu6050_value.z;
      break;
    case 3:
      header = "Gyroscope X:";
      value = mpu6050_value.x;
      break;
    case 4:
      header = "Gyroscope Y:";
      value = mpu6050_value.y;
      break;
    default:
      header = "Gyroscope Z:";
      value = mpu6050_value.z;
      break;
  };

  snprintf(buf, sizeof(buf), HEADER_FORMAT, header);
  for (const char* p = buf; *p; ++p) {
    if ((ret = lcd_write_data(lcd, *p)) != ESP_OK) {
      return ret;
    }
  }

  if ((ret = lcd_set_cursor(lcd, LCD_ROW_2, 0)) != ESP_OK) {
    return ret;
  }

  float s = page < 3 ? mpu6050_accel_sensitivity(MPU6050_ACCEL)
                     : mpu6050_gyro_sensitivity(MPU6050_GYRO);
  snprintf(buf, sizeof(buf), NUM_FORMAT, value / s);
  for (const char* p = buf; *p; ++p) {
    if ((ret = lcd_write_data(lcd, *p)) != ESP_OK) {
      return ret;
    }
  }

  return ESP_OK;
}

void app_main(void) {
  ESP_ERROR_CHECK(init_lcd());
  ESP_ERROR_CHECK(init_mpu6050());
  ESP_ERROR_CHECK(init_buttons());

  ESP_ERROR_CHECK(lcd_reset(lcd));

  uint8_t page = 0;
  update_value(page);

  uint8_t prev_button = 0;
  int64_t ts = esp_timer_get_time();
  for (;;) {
    int64_t elapsed_ms = (esp_timer_get_time() - ts) / 1000;
    uint8_t button = 0;
    if (gpio_get_level(BUTTON_PREV_PAGE_PIN) == 0) {
      button = 1;
    } else if (gpio_get_level(BUTTON_NEXT_PAGE_PIN) == 0) {
      button = 2;
    }

    bool button_pressed = false;
    if (prev_button != button) {
      prev_button = button;
      if (button == 1) {
        button_pressed = true;
        page = page == 0 ? 5 : page - 1;
      } else if (button == 2) {
        button_pressed = true;
        page = page == 5 ? 0 : page + 1;
      }
    }

    if (button_pressed || elapsed_ms >= 1000) {
      ESP_ERROR_CHECK(update_value(page));
      ts = esp_timer_get_time();
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
