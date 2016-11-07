/*
 * acce.h
 *
 *  Created on: 24 set 2016
 *      Author: mauro
 */

#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

uint8_t temp_init(void);
uint8_t temp_update(const uint16_t time);
uint8_t temp_get_data(float * ris);

#endif /* TEMPERATURE_H_ */
