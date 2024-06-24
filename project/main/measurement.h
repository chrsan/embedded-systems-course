#pragma once

#include <stdint.h>

#define MEASUREMENT_NUM_DAYS 6
#define MEASUREMENT_TOTAL_INDEX 7
#define MEASUREMENT_ARRAY_LEN 8

struct Measurement {
  float last_value;

  float avg;
  float min;
  float max;
};

struct MeasurementState {
  uint8_t current_day;
  uint32_t current_measurement_count;
  uint64_t total_measurement_count;
  struct Measurement humidity[MEASUREMENT_ARRAY_LEN];
  struct Measurement temperature[MEASUREMENT_ARRAY_LEN];
};

void measurement_state_update(struct MeasurementState* state, float humidity,
                              float temperature);
