#pragma once

#include <stdint.h>

#include "esp_http_server.h"

enum Lcd1602QueueMessage {
  LCD1602_QUEUE_MESSAGE_UPDATE_DISPLAY,
  LCD1602_QUEUE_MESSAGE_TOGGLE_BACKLIGHT,
};

enum MainQueueMessageType {
  MAIN_QUEUE_MESSAGE_TYPE_MEASUREMENT,
  MAIN_QUEUE_MESSAGE_TYPE_TOGGLE_PAGE,
  MAIN_QUEUE_MESSAGE_TYPE_TURN_OFF_LED,
  MAIN_QUEUE_MESSAGE_TYPE_HTTP_REQUEST,
};

struct MeasurementData {
  uint16_t raw_humidity;
  uint16_t raw_temperature;
};

struct MainQueueMessage {
  enum MainQueueMessageType type;
  union {
    struct MeasurementData measurement_data;
    httpd_req_t* http_req;
  } data;
};
