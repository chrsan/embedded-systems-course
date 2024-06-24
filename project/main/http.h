#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep.
#include "freertos/queue.h"
#include "measurement.h"

esp_err_t http_server_setup(QueueHandle_t main_queue);

esp_err_t http_handle_req(httpd_req_t* req,
                          const struct MeasurementState* state);
