#include "json.h"

#include "esp_timer.h"
#include "json_generator.h"
#include "measurement.h"
#include "util.h"

static void json_gen_measurement(json_gen_str_t* jstr,
                                 const struct Measurement* measurement) {
  json_gen_obj_set_float(jstr, "avg", measurement->avg);
  json_gen_obj_set_float(jstr, "min", measurement->min);
  json_gen_obj_set_float(jstr, "max", measurement->max);
}

void json_gen(json_gen_str_t* jstr, const struct MeasurementState* state) {
  json_gen_start_object(jstr);

  json_gen_obj_set_int(jstr, "uptime", (esp_timer_get_time() / ONE_DAY_US));
  json_gen_obj_set_int(jstr, "current_day", state->current_day);
  json_gen_obj_set_float(jstr, "current_humidity_value",
                         state->humidity[state->current_day].last_value);
  json_gen_obj_set_float(jstr, "current_temperarure_value",
                         state->temperature[state->current_day].last_value);
  json_gen_obj_set_int(jstr, "current_measurement_count",
                       state->current_measurement_count);
  json_gen_obj_set_int(jstr, "total_measurement_count",
                       state->total_measurement_count);

  json_gen_push_array(jstr, "days");
  for (int i = 0; i <= MEASUREMENT_NUM_DAYS; ++i) {
    json_gen_start_object(jstr);
    json_gen_push_object(jstr, "humidity");
    json_gen_measurement(jstr, &state->humidity[i]);
    json_gen_pop_object(jstr);
    json_gen_push_object(jstr, "temperature");
    json_gen_measurement(jstr, &state->temperature[i]);
    json_gen_pop_object(jstr);
    json_gen_end_object(jstr);
  }

  json_gen_pop_array(jstr);

  json_gen_push_object(jstr, "total");
  json_gen_push_object(jstr, "humidity");
  json_gen_measurement(jstr, &state->humidity[MEASUREMENT_TOTAL_INDEX]);
  json_gen_pop_object(jstr);
  json_gen_push_object(jstr, "temperature");
  json_gen_measurement(jstr, &state->temperature[MEASUREMENT_TOTAL_INDEX]);
  json_gen_pop_object(jstr);
  json_gen_pop_object(jstr);

  json_gen_end_object(jstr);
}
