#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dht11.h"
#include "driver/gpio.h"
#include "driver/i2c_types.h"  // IWYU pragma: keep.
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep.
#include "freertos/queue.h"
#include "freertos/task.h"
#include "http.h"
#include "lcd1602.h"
#include "measurement.h"
#include "message.h"
#include "nvs_flash.h"

#define LCD1602_ADDRESS 0x27
#define LCD1602_I2C_PORT I2C_NUM_0
#define LCD1602_SDA_PIN GPIO_NUM_13
#define LCD1602_SCL_PIN GPIO_NUM_14
#define LCD1602_TOGGLE_LIGHT_BUTTON_PIN GPIO_NUM_32
#define LCD1602_TOGGLE_PAGE_BUTTON_PIN GPIO_NUM_33

#define LCD1602_BUFFER_SIZE 16

#define DHT11_PIN GPIO_NUM_4
#define DHT11_SENSOR_READ_INTERVAL_MS 4000
// #define DHT11_SENSOR_READ_INTERVAL_MS 1000 * 60

#define LED_PIN GPIO_NUM_15

#define QUEUE_LENGTH 8

struct MeasurementState measurement_state;

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

void lcd1602_write() {
  esp_err_t ret =
      ESP_ERROR_CHECK_WITHOUT_ABORT(lcd1602_set_cursor(LCD1602_ROW_1, 0));
  for (const char* p = lcd1602_header; ret == ESP_OK && *p; ++p) {
    ret = ESP_ERROR_CHECK_WITHOUT_ABORT(lcd1602_write_data(*p));
  }

  if (ret == ESP_OK) {
    ret = ESP_ERROR_CHECK_WITHOUT_ABORT(lcd1602_set_cursor(LCD1602_ROW_2, 0));
  }

  for (const char* p = lcd1602_value; ret == ESP_OK && *p; ++p) {
    ret = ESP_ERROR_CHECK_WITHOUT_ABORT(lcd1602_write_data(*p));
  }
}

void lcd1602_task(void* params) {
  enum Lcd1602QueueMessage message;
  for (;;) {
    xQueueReceive(lcd1602_queue, &message, portMAX_DELAY);
    switch (message) {
      case LCD1602_QUEUE_MESSAGE_UPDATE_DISPLAY:
        lcd1602_write();
        break;
      case LCD1602_QUEUE_MESSAGE_TOGGLE_BACKLIGHT:
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

QueueHandle_t main_queue = NULL;

void buttons_task(void* params) {
  uint8_t prev_button = 0;
  for (;;) {
    uint8_t button = 0;
    if (gpio_get_level(LCD1602_TOGGLE_LIGHT_BUTTON_PIN) == 0) {
      button = 1;
    } else if (gpio_get_level(LCD1602_TOGGLE_PAGE_BUTTON_PIN) == 0) {
      button = 2;
    }

    if (prev_button != button) {
      prev_button = button;
      if (button == 1) {
        if (lcd1602_backlight_on) {
          struct MainQueueMessage main_queue_message = {
              .type = MAIN_QUEUE_MESSAGE_TYPE_TURN_OFF_LED,
          };

          xQueueSendToBack(main_queue, &main_queue_message, portMAX_DELAY);
        }

        enum Lcd1602QueueMessage lcd1602_message =
            LCD1602_QUEUE_MESSAGE_TOGGLE_BACKLIGHT;
        xQueueSendToBack(lcd1602_queue, &lcd1602_message, portMAX_DELAY);
      } else if (button == 2) {
        struct MainQueueMessage main_queue_message = {
            .type = MAIN_QUEUE_MESSAGE_TYPE_TOGGLE_PAGE,
        };

        xQueueSendToBack(main_queue, &main_queue_message, portMAX_DELAY);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

esp_err_t buttons_setup() {
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << LCD1602_TOGGLE_LIGHT_BUTTON_PIN) |
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

esp_err_t led_setup() {
  gpio_config_t io_conf = {
      .pin_bit_mask = 1ULL << LED_PIN,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  return gpio_config(&io_conf);
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
          .u =
              {
                  .measurement =
                      {
                          .raw_humidity = humidity,
                          .raw_temperature = temperature,
                      },
              },
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

  if (xTaskCreate(dht11_task, "main", configMINIMAL_STACK_SIZE, NULL, 1,
                  NULL) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

esp_err_t wifi_setup() {
  esp_err_t ret;

  if ((ret = esp_netif_init()) != ESP_OK) {
    return ret;
  }

  if ((ret = esp_event_loop_create_default()) != ESP_OK) {
    return ret;
  }

  esp_netif_create_default_wifi_ap();
  wifi_init_config_t wifi_init_conf = WIFI_INIT_CONFIG_DEFAULT();
  if ((ret = esp_wifi_init(&wifi_init_conf)) != ESP_OK) {
    return ret;
  }

  if ((ret = esp_wifi_set_mode(WIFI_MODE_AP)) != ESP_OK) {
    return ret;
  }

  wifi_config_t wifi_conf = {
      .ap =
          {
              .ssid = "ESP32-WROVER",
              .password = "",
              .channel = 1,
              .authmode = WIFI_AUTH_OPEN,
              .max_connection = 2,
              .pmf_cfg =
                  {
                      .required = true,
                  },
          },
  };

  if ((ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_conf)) != ESP_OK) {
    return ret;
  }

  return esp_wifi_start();
}

void app_main(void) {
  // N.B. This is required for Wi-Fi.
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);

  memset(&measurement_state, 0, sizeof(struct MeasurementState));

  ESP_ERROR_CHECK(lcd1602_setup());
  ESP_ERROR_CHECK(buttons_setup());
  ESP_ERROR_CHECK(led_setup());
  ESP_ERROR_CHECK(main_setup());
  ESP_ERROR_CHECK(wifi_setup());
  ESP_ERROR_CHECK(http_server_setup(main_queue));

  struct MainQueueMessage main_queue_message;

  float humidity;
  float temperature;
  for (;;) {
    xQueueReceive(main_queue, &main_queue_message, portMAX_DELAY);

    bool update_lcd = false;
    switch (main_queue_message.type) {
      case MAIN_QUEUE_MESSAGE_TYPE_MEASUREMENT:
        update_lcd = true;
        humidity =
            dht11_humidity(main_queue_message.u.measurement.raw_humidity);
        temperature =
            dht11_temperature(main_queue_message.u.measurement.raw_temperature);
        if (lcd1602_backlight_on && (humidity < 30 || humidity > 60)) {
          gpio_set_level(LED_PIN, 1);
        }

        measurement_state_update(&measurement_state, humidity, temperature);
        break;
      case MAIN_QUEUE_MESSAGE_TYPE_TOGGLE_PAGE:
        update_lcd = true;
        lcd1602_page = lcd1602_page == 7 ? 0 : lcd1602_page + 1;
        break;
      case MAIN_QUEUE_MESSAGE_TYPE_TURN_OFF_LED:
        gpio_set_level(LED_PIN, 0);
        break;
      case MAIN_QUEUE_MESSAGE_TYPE_HTTP_REQUEST:
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            http_handle_req(main_queue_message.u.http_req, &measurement_state));
        break;
    }

    if (update_lcd) {
      lcd1602_format_rows();
      enum Lcd1602QueueMessage lcd1602_queue_message =
          LCD1602_QUEUE_MESSAGE_UPDATE_DISPLAY;
      xQueueSendToBack(lcd1602_queue, &lcd1602_queue_message, portMAX_DELAY);
    }
  }
}
