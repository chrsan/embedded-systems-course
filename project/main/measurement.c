#include "measurement.h"

#include <string.h>

#include "esp_timer.h"
#include "util.h"

static void update_measurement(struct Measurement* measurement, uint32_t count,
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

void measurement_state_update(struct MeasurementState* state, float humidity,
                              float temperature) {
  uint8_t day =
      (esp_timer_get_time() / ONE_DAY_US) % (MEASUREMENT_NUM_DAYS + 1);
  if (day != state->current_day) {
    state->current_day = day;
    state->current_measurement_count = 0;

    memset(&state->humidity[day], 0, sizeof(struct Measurement));
    memset(&state->temperature[day], 0, sizeof(struct Measurement));
  }

  state->current_measurement_count += 1;
  update_measurement(&state->humidity[day], state->current_measurement_count,
                     humidity);
  update_measurement(&state->temperature[day], state->current_measurement_count,
                     temperature);

  state->total_measurement_count += 1;
  update_measurement(&state->humidity[7], state->total_measurement_count,
                     humidity);
  update_measurement(&state->temperature[7], state->total_measurement_count,
                     temperature);
}
