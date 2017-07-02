/*
 * my_math.h
 *
 *  Created on: 10/dic/2014
 *      Author: mauro
 */

#ifndef MY_MATH_H_
#define MY_MATH_H_

#include <stdint.h>
#include <stddef.h>
#include <math.h>


float invSqrt(float x);
uint8_t fequals( float a, float b, float epsilon );

#endif /* MY_MATH_H_ */
