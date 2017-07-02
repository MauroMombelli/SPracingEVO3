#ifndef MAGNETOMETER_H_
#define MAGNETOMETER_H_

#include "TrigonomeC/my_math.h"
#include "TrigonomeC/vector3f.h"
#include "shared.h"

void get_estimated_error_magne(struct Vector3f * restrict const raw_data, struct Vector3f * restrict const halfe, struct tmp_data * restrict tmp_shared);

#endif
