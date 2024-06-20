#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "dht11.h"
#include "driver/gpio.h"
#include "driver/i2c_types.h"  // IWYU pragma: keep.
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep.
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lcd1602.h"
#include "portmacro.h"

#define TAG "project"

#define LCD1602_ADDRESS 0x27
#define LCD1602_I2C_PORT I2C_NUM_0
#define LCD1602_SDA_PIN GPIO_NUM_13
#define LCD1602_SCL_PIN GPIO_NUM_14
#define LCD1602_TOGGLE_BACKLIGHT_PIN GPIO_NUM_33

#define DHT11_PIN GPIO_NUM_2
#define DHT11_SENSOR_READ_INTERVAL_MS 4000

#define QUEUE_LENGTH 8

QueueHandle_t lcd1602_queue = NULL;

enum Lcd1602QueueItemType {
  LCD1602_QUEUE_ITEM_TYPE_TOGGLE_BACKLIGHT,
  // LCD1602_QUEUE_ITEM_TYPE_
};

struct Lcd1602QueueItem {
  enum Lcd1602QueueItemType item_type;
};

void lcd1602_task(void* params) {
  struct Lcd1602QueueItem item;
  for (;;) {
    xQueueReceive(lcd1602_queue, &item, portMAX_DELAY);
    switch (item.item_type) {
      case LCD1602_QUEUE_ITEM_TYPE_TOGGLE_BACKLIGHT:
        lcd1602_toggle_backlight();
        break;
    }
  }
}

esp_err_t lcd1602_setup() {
  lcd1602_queue = xQueueCreate(QUEUE_LENGTH, sizeof(uint32_t));
  if (!lcd1602_queue) {
    return ESP_ERR_NO_MEM;
  }

  if (xTaskCreate(lcd1602_task, "LCD1602", configMINIMAL_STACK_SIZE, NULL, 1,
                  NULL) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  return lcd1602_init(LCD1602_I2C_PORT, LCD1602_SDA_PIN, LCD1602_SCL_PIN,
                      LCD1602_ADDRESS);
}

void lcd1602_buttons_task(void* params) {
  struct Lcd1602QueueItem item;
  uint8_t prev_button = 0;
  for (;;) {
    uint8_t button = 0;
    if (gpio_get_level(LCD1602_TOGGLE_BACKLIGHT_PIN) == 0) {
      button = 1;
    }

    bool button_pressed = false;
    if (prev_button != button) {
      prev_button = button;
      if (button == 1) {
        button_pressed = true;
        item.item_type = LCD1602_QUEUE_ITEM_TYPE_TOGGLE_BACKLIGHT;
      }
    }

    if (button_pressed) {
      xQueueSendToBack(lcd1602_queue, &item, portMAX_DELAY);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

esp_err_t lcd1602_buttons_setup() {
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << LCD1602_TOGGLE_BACKLIGHT_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    return ret;
  }

  if (xTaskCreate(lcd1602_buttons_task, "LCD1602-buttons",
                  configMINIMAL_STACK_SIZE, NULL, 1, NULL) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

QueueHandle_t dht11_queue = NULL;

void dht11_task(void* params) {
  vTaskDelay(pdMS_TO_TICKS(1000));

  uint16_t humidity;
  uint16_t temperature;
  for (;;) {
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(
            dht11_read(DHT11_PIN, &humidity, &temperature)) == ESP_OK) {
      uint32_t packed = humidity << 16 | temperature;
      xQueueSendToBack(dht11_queue, &packed, portMAX_DELAY);
    }

    vTaskDelay(pdMS_TO_TICKS(DHT11_SENSOR_READ_INTERVAL_MS));
  }
}

esp_err_t dht11_setup() {
  dht11_queue = xQueueCreate(QUEUE_LENGTH, sizeof(uint32_t));
  if (!dht11_queue) {
    return ESP_ERR_NO_MEM;
  }

  if (xTaskCreate(dht11_task, "DHT11", configMINIMAL_STACK_SIZE, NULL, 1,
                  NULL) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

void app_main(void) {
  time_t now = time(NULL);
  if (now == -1) {
    ESP_LOGE(TAG, "could not get time of day");
    return;
  }

  struct tm* local_time = localtime(&now);
  printf("week day: %d\n", local_time->tm_wday);

  ESP_ERROR_CHECK(lcd1602_setup());
  ESP_ERROR_CHECK(lcd1602_buttons_setup());
  ESP_ERROR_CHECK(dht11_setup());

  uint32_t dht11_data;
  for (;;) {
    xQueueReceive(dht11_queue, &dht11_data, portMAX_DELAY);
    ESP_LOGI(TAG, "dht11 data: %" PRIu32, dht11_data);
  }
}
