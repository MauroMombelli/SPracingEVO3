/*
 * error.c
 *
 *  Created on: Jul 2, 2017
 *      Author: mauro
 */

#include "error.h"
#include "cmsis_device.h"
#include "serial/usart.h"
#include "Timer.h"

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

void systemReset(void) {
	// Generate system reset
	SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t) 0x04;
}

void systemResetToBootloader(void) {
	// 1FFFF000 -> 20000200 -> SP
	// 1FFFF004 -> 1FFFF021 -> PC

	*((uint32_t *) 0x20009FFC) = 0xDEADBEEF; // 40KB SRAM STM32F30X

	systemReset();
	//NVIC_SystemReset();
}

void check_for_reset(void){
	uint8_t  b;
	uint8_t len;
	do{
		len = USART1_Read(&b, 1);

		if (len && b == 'R') {
			USART1_Write("reboot\n", 7);
			timer_sleep(10);
			systemResetToBootloader();
		}
	}while(len != 0);
}

void wait_for_reset(void){
	uint8_t  b;
	while(1){
		uint8_t len = USART1_Read(&b, 1);

		if (len && b == 'R') {
			USART1_Write("reboot\n", 7);
			timer_sleep(10);
			systemResetToBootloader();
		}
		if (len == 0){
			timer_sleep(100);
		}
	}
}

