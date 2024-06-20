#pragma once

#include <stdint.h>

#include "driver/gpio.h"  // IWYU pragma: keep.
#include "esp_err.h"

float dht11_humidity(uint16_t humidity);

float dht11_temperature(uint16_t temperature);

esp_err_t dht11_read(gpio_num_t pin, uint16_t* humidity, uint16_t* temperature);
