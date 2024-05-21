#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define PIN_LED GPIO_NUM_2
#define TAG "MAIN"

static void sleep_one_second() { vTaskDelay(pdMS_TO_TICKS(1000)); }

static void update_led(bool on) { gpio_set_level(PIN_LED, on); }

void app_main(void) {
  esp_rom_gpio_pad_select_gpio(PIN_LED);
  gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);

  for (int i = 0; i < 10; ++i) {
    ESP_LOGI(TAG, "Turning LED with pin %d on", PIN_LED);
    update_led(true);
    sleep_one_second();
    ESP_LOGI(TAG, "Turning LED with pin %d off", PIN_LED);
    update_led(false);
    sleep_one_second();
  }
}
