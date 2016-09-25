/*
 * gyro.h
 *
 *  Created on: 24 set 2016
 *      Author: mauro
 */

#ifndef GYRO_H_
#define GYRO_H_


struct gyro_data{
	float x; // radians/s
	float y; // radians/s
	float z; // radians/s
};

bool gyro_init(void);
bool gyro_get_data(struct gyro_data * const ris);


#endif /* GYRO_H_ */
