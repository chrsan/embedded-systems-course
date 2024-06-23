#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dht11.h"
#include "driver/gpio.h"
#include "driver/i2c_types.h"  // IWYU pragma: keep.
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep.
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lcd1602.h"

#define TAG "project"

#define LCD1602_ADDRESS 0x27
#define LCD1602_I2C_PORT I2C_NUM_0
#define LCD1602_SDA_PIN GPIO_NUM_13
#define LCD1602_SCL_PIN GPIO_NUM_14
#define LCD1602_TOGGLE_BACKLIGHT_BUTTON_PIN GPIO_NUM_15
#define LCD1602_TOGGLE_PAGE_BUTTON_PIN GPIO_NUM_33

#define LCD1602_BUFFER_SIZE 16

#define DHT11_PIN GPIO_NUM_2
#define DHT11_SENSOR_READ_INTERVAL_MS 4000

#define QUEUE_LENGTH 8

#define ONE_DAY_US INT64_C(86400000000)

struct Measurement {
  float last_value;

  float avg;
  float min;
  float max;
};

void update_measurement(struct Measurement* measurement, uint32_t count,
                        float value) {
  measurement->last_value = value;
  if (count == 1) {
    measurement->avg = value;
  } else {
    measurement->avg += (value - measurement->avg) / count;
  }

  if (value < measurement->min || count == 1) {
    measurement->min = value;
  }

  if (value > measurement->max) {
    measurement->max = value;
  }
}

struct MeasurementState {
  uint8_t current_day;
  uint32_t current_measurement_count;
  uint64_t total_measurement_count;
  struct Measurement humidity[8];
  struct Measurement temperature[8];
};

struct MeasurementState measurement_state;

void update_state(uint16_t raw_humidity, uint16_t raw_temperature) {
  float humidity = dht11_humidity(raw_humidity);
  float temperature = dht11_temperature(raw_temperature);

  uint8_t day = (esp_timer_get_time() / ONE_DAY_US) % 7;
  if (day != measurement_state.current_day) {
    measurement_state.current_day = day;

    memset(&measurement_state.humidity[day], 0, sizeof(struct Measurement));
    memset(&measurement_state.temperature[day], 0, sizeof(struct Measurement));
  }

  measurement_state.current_measurement_count += 1;
  update_measurement(&measurement_state.humidity[day],
                     measurement_state.current_measurement_count, humidity);
  update_measurement(&measurement_state.temperature[day],
                     measurement_state.current_measurement_count, temperature);

  measurement_state.total_measurement_count += 1;
  update_measurement(&measurement_state.humidity[7],
                     measurement_state.total_measurement_count, humidity);
  update_measurement(&measurement_state.temperature[7],
                     measurement_state.total_measurement_count, temperature);
}

char lcd1602_header[LCD1602_BUFFER_SIZE + 1];
char lcd1602_value[LCD1602_BUFFER_SIZE + 1];

uint8_t lcd1602_page = 0;

void lcd1602_format_rows() {
  uint8_t day = measurement_state.current_day;

  const char* header = "";
  float value = 0;
  switch (lcd1602_page) {
    case 0:
      header = "Humidity:";
      value = measurement_state.humidity[day].last_value;
      break;
    case 1:
      header = "Avg humidity:";
      value = measurement_state.humidity[day].avg;
      break;
    case 2:
      header = "Min humidity:";
      value = measurement_state.humidity[day].min;
      break;
    case 3:
      header = "Max humidity:";
      value = measurement_state.humidity[day].max;
      break;
    case 4:
      header = "Temperature:";
      value = measurement_state.temperature[day].last_value;
      break;
    case 5:
      header = "Avg temperature:";
      value = measurement_state.temperature[day].avg;
      break;
    case 6:
      header = "Min temperature:";
      value = measurement_state.temperature[day].min;
      break;
    case 7:
      header = "Max temperature:";
      value = measurement_state.temperature[day].max;
      break;
  }

  snprintf(lcd1602_header, sizeof(lcd1602_header), "%16s", header);
  snprintf(lcd1602_value, sizeof(lcd1602_value), "%15.2f", value);
}

QueueHandle_t lcd1602_queue = NULL;

enum Lcd1602QueueMessage {
  LCD1602_QUEUE_MESSAGE_UPDATE_DISPLAY,
  LCD1602_QUEUE_MESSAGE_TOGGLE_BACKLIGHT,
};

void lcd1602_task(void* params) {
  enum Lcd1602QueueMessage message;
  for (;;) {
    xQueueReceive(lcd1602_queue, &message, portMAX_DELAY);

    bool update = false;
    switch (message) {
      case LCD1602_QUEUE_MESSAGE_UPDATE_DISPLAY:
        update = true;
        break;
      case LCD1602_QUEUE_MESSAGE_TOGGLE_BACKLIGHT:
        lcd1602_toggle_backlight();
        break;
    }

    if (update) {
      if (ESP_ERROR_CHECK_WITHOUT_ABORT(lcd1602_set_cursor(LCD1602_ROW_1, 0)) !=
          ESP_OK) {
        continue;
      }

      for (const char* p = lcd1602_header; *p; ++p) {
        if (ESP_ERROR_CHECK_WITHOUT_ABORT(lcd1602_write_data(*p)) != ESP_OK) {
          continue;
        }
      }

      if (ESP_ERROR_CHECK_WITHOUT_ABORT(lcd1602_set_cursor(LCD1602_ROW_2, 0)) !=
          ESP_OK) {
        continue;
      }

      for (const char* p = lcd1602_value; *p; ++p) {
        if (ESP_ERROR_CHECK_WITHOUT_ABORT(lcd1602_write_data(*p)) != ESP_OK) {
          continue;
        }
      }
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

QueueHandle_t main_queue = NULL;

enum MainQueueMessageType {
  MAIN_QUEUE_MESSAGE_TYPE_MEASUREMENT,
  MAIN_QUEUE_MESSAGE_TYPE_TOGGLE_PAGE,
};

struct MainQueueMessage {
  enum MainQueueMessageType type;
  uint16_t raw_humidity;
  uint16_t raw_temperature;
};

void buttons_task(void* params) {
  uint8_t prev_button = 0;
  for (;;) {
    uint8_t button = 0;
    if (gpio_get_level(LCD1602_TOGGLE_BACKLIGHT_BUTTON_PIN) == 0) {
      button = 1;
    } else if (gpio_get_level(LCD1602_TOGGLE_PAGE_BUTTON_PIN) == 0) {
      button = 2;
    }

    if (prev_button != button) {
      prev_button = button;
      if (button == 1) {
        enum Lcd1602QueueMessage message =
            LCD1602_QUEUE_MESSAGE_TOGGLE_BACKLIGHT;
        xQueueSendToBack(lcd1602_queue, &message, portMAX_DELAY);
      } else if (button == 2) {
        struct MainQueueMessage message = {
            .type = MAIN_QUEUE_MESSAGE_TYPE_TOGGLE_PAGE,
        };

        xQueueSendToBack(main_queue, &message, portMAX_DELAY);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

esp_err_t buttons_setup() {
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << LCD1602_TOGGLE_BACKLIGHT_BUTTON_PIN) |
                      (1ULL << LCD1602_TOGGLE_PAGE_BUTTON_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    return ret;
  }

  if (xTaskCreate(buttons_task, "buttons", configMINIMAL_STACK_SIZE, NULL, 1,
                  NULL) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

void dht11_task(void* params) {
  vTaskDelay(pdMS_TO_TICKS(1000));

  uint16_t humidity;
  uint16_t temperature;
  for (;;) {
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(
            dht11_read(DHT11_PIN, &humidity, &temperature)) == ESP_OK) {
      struct MainQueueMessage message = {
          .type = MAIN_QUEUE_MESSAGE_TYPE_MEASUREMENT,
          .raw_humidity = humidity,
          .raw_temperature = temperature,
      };

      xQueueSendToBack(main_queue, &message, portMAX_DELAY);
    }

    vTaskDelay(pdMS_TO_TICKS(DHT11_SENSOR_READ_INTERVAL_MS));
  }
}

esp_err_t main_setup() {
  main_queue = xQueueCreate(QUEUE_LENGTH, sizeof(struct MainQueueMessage));
  if (!main_queue) {
    return ESP_ERR_NO_MEM;
  }

  if (xTaskCreate(dht11_task, "DHT11", configMINIMAL_STACK_SIZE, NULL, 1,
                  NULL) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

void app_main(void) {
  memset(&measurement_state, 0, sizeof(struct MeasurementState));

  ESP_ERROR_CHECK(lcd1602_setup());
  ESP_ERROR_CHECK(buttons_setup());
  ESP_ERROR_CHECK(main_setup());

  struct MainQueueMessage main_queue_message;
  for (;;) {
    xQueueReceive(main_queue, &main_queue_message, portMAX_DELAY);
    switch (main_queue_message.type) {
      case MAIN_QUEUE_MESSAGE_TYPE_MEASUREMENT:
        update_state(main_queue_message.raw_humidity,
                     main_queue_message.raw_temperature);
        break;
      case MAIN_QUEUE_MESSAGE_TYPE_TOGGLE_PAGE:
        lcd1602_page = lcd1602_page == 7 ? 0 : lcd1602_page + 1;
        break;
    }

    lcd1602_format_rows();
    enum Lcd1602QueueMessage lcd1602_queue_message =
        LCD1602_QUEUE_MESSAGE_UPDATE_DISPLAY;
    xQueueSendToBack(lcd1602_queue, &lcd1602_queue_message, portMAX_DELAY);
  }
}
