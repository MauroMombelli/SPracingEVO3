/*
 * gyro.h
 *
 *  Created on: 24 set 2016
 *      Author: mauro
 */

#ifndef GYRO_H_
#define GYRO_H_

#include "vector3f.h"

uint8_t gyro_init(void);
uint8_t gyro_update(const uint32_t time);
uint8_t gyro_get_data(struct vector3f * const ris);


#endif /* GYRO_H_ */
