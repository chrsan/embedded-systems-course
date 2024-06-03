#include <stddef.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "keypad.h"

#define PIN1 GPIO_NUM_23
#define PIN2 GPIO_NUM_22
#define PIN3 GPIO_NUM_21
#define PIN4 GPIO_NUM_13
#define PIN5 GPIO_NUM_25
#define PIN6 GPIO_NUM_26
#define PIN7 GPIO_NUM_27
#define PIN8 GPIO_NUM_14

#define TAG "task1"

static const char keys[4][4] = {{'1', '2', '3', 'A'},
                                {'4', '5', '6', 'B'},
                                {'7', '8', '9', 'C'},
                                {'*', '0', '#', 'D'}};

void app_main(void) {
  gpio_num_t row_pins[] = {PIN8, PIN7, PIN6, PIN5};
  gpio_num_t col_pins[] = {PIN4, PIN3, PIN2, PIN1};
  struct KeypadConfig keypad_config = {
      .row_pins = row_pins,
      .col_pins = col_pins,
      .num_row_pins = 4,
      .num_col_pins = 4,
  };

  struct Keypad* keypad = NULL;
  ESP_ERROR_CHECK(keypad_init(&keypad_config, &keypad));

  struct KeypadEvent event;
  for (;;) {
    keypad_scan(keypad);
    if (keypad_get_event(keypad, &event)) {
      char key = keys[event.row_index][event.col_index];
      ESP_LOGI(TAG, "%c", key);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
