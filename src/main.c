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

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#include "gyro.h"
#include "serial/usart.h"

int main(int argc, char* argv[]) {
	// Send a greeting to the trace device (skipped on Release).
	trace_puts("Hello ARM World!");

	// At this stage the system clock should have already been configured
	// at high speed.
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	timer_start();

	USART1_Init(115200);

	blink_led_init();

	if (gyro_init()) {
		while (1) {
			USART_SendData(USART1, 'F');
			/* Check the Transfer Complete Flag */
			while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
			}
		}
	}

	uint32_t seconds = 0;

	USART_SendData(USART1, (uint8_t) 'a');

	// Infinite loop
	while (1) {
		blink_led_on();
		timer_sleep(1000);

		blink_led_off();
		timer_sleep(1000);

		++seconds;

		// Count seconds on the trace device.
		//printf("Second %ld\n", seconds);

		struct gyro_data gyro;
		if (gyro_get_data(&gyro)) {

			USART_SendData(USART1, 'a');
			/* Check the Transfer Complete Flag */
			while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
			}
			USART_SendData(USART1, 'a');
			/* Check the Transfer Complete Flag */
			while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
			}

			union {
				float f;
				uint8_t raw[4];
			} tmp;

			tmp.f = gyro.x;
			for (uint8_t i = 0; i < 4; i++) {
				USART_SendData(USART1, tmp.raw[i]);
				/* Check the Transfer Complete Flag */
				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
				}
			}

			tmp.f = gyro.y;
			for (uint8_t i = 0; i < 4; i++) {
				USART_SendData(USART1, tmp.raw[i]);
				/* Check the Transfer Complete Flag */
				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
				}
			}

			tmp.f = gyro.z;
			for (uint8_t i = 0; i < 4; i++) {
				USART_SendData(USART1, tmp.raw[i]);
				/* Check the Transfer Complete Flag */
				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
				}
			}
			/* Check the Transfer Complete Flag */
			while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
			}
			USART_SendData(USART1, '\n');
		}

	}
	// Infinite loop, never return.
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
