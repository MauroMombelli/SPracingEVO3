#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include "TrigonomeC/my_math.h"
#include "TrigonomeC/vector3f.h"
#include "shared.h"

void get_estimated_error_acce(struct Vector3f * restrict const raw_data, struct Vector3f * restrict const halfe, struct tmp_data * restrict tmp_shared);

#endif
