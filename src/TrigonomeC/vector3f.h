/*
 * vector3f.h
 *
 *  Created on: 19/mar/2015
 *      Author: mauro
 */

#ifndef MY_MATH_INCLUDE_VECTOR3F_H_
#define MY_MATH_INCLUDE_VECTOR3F_H_

#include "my_math.h"


struct Vector3f{
	float x, y, z;
};

static uint8_t is_zero (const struct Vector3f * restrict const input){
	return fequals(input->x, 0.0f, 0.0000001f) && fequals(input->y, 0.0f, 0.0000001f) && fequals(input->z, 0.0f, 0.0000001f);
}

static void copy (const struct Vector3f * restrict const toClone, struct Vector3f * restrict clone){
	clone->x = toClone->x;
	clone->y = toClone->y;
	clone->z = toClone->z;
}

static void mult (const struct Vector3f * restrict const v, const float value, struct Vector3f * restrict const result){
	result->x = v->x * value;
	result->y = v->y * value;
	result->z = v->z * value;
}

static void sub (const struct Vector3f * restrict const sx, const struct Vector3f * restrict const dx, struct Vector3f * restrict const ris){
	ris->x = sx->x - dx->x;
	ris->y = sx->y - dx->y;
	ris->z = sx->z - dx->z;
}

static float dot (const struct Vector3f * restrict const sx, const struct Vector3f * restrict const dx){
	return sx->x * dx->x + sx->y * dx->y + sx->z * sx->z;
}

static void normalize (const struct Vector3f * restrict const v, struct Vector3f * restrict const result){
	float abs_sum = fabsf(v->x + v->y + v->z);
	result->x = v->x / abs_sum;
	result->y = v->y / abs_sum;
	result->z = v->z / abs_sum;
}

static const struct {
	void    (* copy)      (const struct Vector3f* restrict const, struct Vector3f* crestrictonst); /* origin, copy */
	void    (* mult)      (const struct Vector3f* restrict const, const float value, struct Vector3f* restrict const); /* sx, value, result*/
	void    (* sub)       (const struct Vector3f* restrict const, const struct Vector3f* restrict const, struct Vector3f* restrict const); /* sx, dx, result */
	float   (* dot)       (const struct Vector3f* restrict const, const struct Vector3f* restrict const); /* result is returned! */
	void    (* normalize) (const struct Vector3f* restrict const, struct Vector3f* restrict const);
	uint8_t (* is_zero)   (const struct Vector3f* restrict const);
} vector3f_helper = {
	copy, mult, sub, dot, normalize, is_zero
};

#endif /* MY_MATH_INCLUDE_VECTOR3F_H_ */
