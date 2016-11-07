/*
 * acce.h
 *
 *  Created on: 24 set 2016
 *      Author: mauro
 */

#ifndef ACCE_H_
#define ACCE_H_

uint8_t acce_init(void);
uint8_t acce_update(const uint16_t time);
uint8_t acce_get_data(struct vector3f * const ris);

#endif /* ACCE_H_ */
