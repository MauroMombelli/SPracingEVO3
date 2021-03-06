/*
 * accelerometer.c
 *
 *  Created on: 09/mag/2015
 *      Author: mauro
 */

#include "accelerometer.h"

void get_estimated_error_acce(struct Vector3f * restrict const raw_data, struct Vector3f * restrict const halfe, struct tmp_data * restrict tmp_shared) {

	if ( !vector3f_helper.is_zero(raw_data) ){
		return;
	}

    /* Prevent math error, also if we are in PERFECT free-fall we can't predict nothing*/
    float halfvx, halfvy, halfvz;

    /* Normalize accelerometer measurement */
    float recipNorm = invSqrt(raw_data->x * raw_data->x + raw_data->y * raw_data->y + raw_data->z * raw_data->z);
    raw_data->x *= recipNorm;
    raw_data->y *= recipNorm;
    raw_data->z *= recipNorm;

    /* Estimated direction of gravity */
    halfvx = tmp_shared->q1q3 - tmp_shared->q0q2;
    halfvy = tmp_shared->q0q1 + tmp_shared->q2q3;
    halfvz = tmp_shared->q0q0 - 0.5f + tmp_shared->q3q3;

    /* Error is sum of cross product between estimated direction and measured direction of field vectors */
    halfe->x += (raw_data->y * halfvz - raw_data->z * halfvy);
    halfe->y += (raw_data->z * halfvx - raw_data->x * halfvz);
    halfe->z += (raw_data->x * halfvy - raw_data->y * halfvx);
}
