//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "spi.h"
// ----------------------------------------------------------------------------
//
// Standalone STM32F3 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8 MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f30x.c
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 1 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

#include "string.h"

#include "gyro.h"
#include "acce.h"
#include "temperature.h"
#include "serial/usart.h"

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

void systemReset(void){
    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}

void systemResetToBootloader(void) {
	// 1FFFF000 -> 20000200 -> SP
	// 1FFFF004 -> 1FFFF021 -> PC

	*((uint32_t *) 0x20009FFC) = 0xDEADBEEF; // 40KB SRAM STM32F30X

	systemReset();
	//NVIC_SystemReset();
}

int main(int argc, char* argv[]) {
	(void) argc;
	(void) argv;

	// Send a greeting to the trace device (skipped on Release).
	trace_puts("Hello ARM World!");

	// At this stage the system clock should have already been configured
	// at high speed.
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	timer_start();

	USART1_Init(115200);

	USART1_Write("START1\n", 7);

	blink_led_init();

	USART1_Write("START2\n", 7);

	uint8_t ris = gyro_init();
	if (ris) {
		while (1) {
			USART1_Write(&ris, 1);
			USART1_Write(" WRONG\n", 7);
		}
	}
	USART1_Write(&ris, 1);
	USART1_Write(" OK\n", 4);

	USART1_Write("GYRO OK\n", 8);

	// Infinite loop
	while (1) {

		uint32_t time = micros();

		//blink_led_on();
		//timer_sleep(1000);

		//blink_led_off();
		//timer_sleep(1000);

		uint8_t b[100];

		uint8_t len = USART1_Read(b, sizeof(b));

		for (uint8_t i = 0; i < len; i++) {
			if (b[i] == 'R') {
				systemResetToBootloader();
			}
		}

		USART1_Write(b, len);

		static uint32_t last = 0;
		static uint32_t readSec = 0, readErr = 0, readWait = 0;

		if (time - last >= 1000000) {
			last = time;
			char buffer [33];

			itoa((int)readSec, buffer, 10);
			USART1_Write(buffer, (uint8_t)strlen(buffer) );

			USART1_Write(" ", 1);

			itoa((int)readErr, buffer, 10);
			USART1_Write(buffer, (uint8_t)strlen(buffer) );

			USART1_Write(" ", 1);

			itoa((int)readWait, buffer, 10);
			USART1_Write(buffer, (uint8_t)strlen(buffer) );

			USART1_Write(" ", 1);

			itoa((int)time, buffer, 10 );
			USART1_Write(buffer, (uint8_t)strlen(buffer));

			USART1_Write(" alive\n", 7);
			readSec = readErr = readWait = 0;
		}


/*
		struct vector3f gyro, acce;
		float temp;
*/
		ris = gyro_update( time );

		if (!ris) {
			readSec++;
/*
			gyro_get_data(&gyro);

			acce_get_data(&acce);

			temp_get_data(&temp);

			int t;

			USART1_Write("GG", 2);
			t = abs(gyro.x);
			USART1_Write(&t, 2);

			t = abs(gyro.y);
			USART1_Write(&t, 2);

			t = abs(gyro.z);
			USART1_Write(&t, 2);

			USART1_Write("AA", 2);
			t = abs(acce.x);
			USART1_Write(&t, 2);

			t = abs(acce.y);
			USART1_Write(&t, 2);

			t = abs(acce.z);
			USART1_Write(&t, 2);
*/
			//USART1_Write("r\n", 2);
		} else {
			if (ris != 1){
				ris = (uint8_t) (ris + '0');//readable ascii
				USART1_Write(&ris, 1);
				USART1_Write(" ERROR\n", 7);
				readErr++;
			}else{
				//USART1_Write("NR\n", 3);
				readWait++;
			}

		}

	}

// Infinite loop, never return.
	return -1;
}

// ----------------------------------------------------------------------------
