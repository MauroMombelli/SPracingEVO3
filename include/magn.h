/*
 * magn.h
 *
 *  Created on: 24 set 2016
 *      Author: mauro
 */

#ifndef MAGN_H_
#define MAGN_H_

#include "vector3f.h"

uint8_t magne_init(void);
uint8_t magne_update(const uint32_t time);
uint8_t magne_get_data(struct vector3f * const ris);


#endif /* MAGN_H_ */
