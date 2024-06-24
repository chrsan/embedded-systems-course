#pragma once

#include "json_generator.h"
#include "measurement.h"

void json_gen(json_gen_str_t* jstr, const struct MeasurementState* state);
