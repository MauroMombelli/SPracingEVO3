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

volatile int data_is_int = 0;

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

void tmp(void) {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	USART1_Write("T1\n", 3);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	USART1_Write("TB\n", 3);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	USART1_Write("T2\n", 3);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);
	USART1_Write("T3\n", 3);

	EXTI_InitTypeDef EXTIInit;
	EXTI_StructInit(&EXTIInit);
	EXTIInit.EXTI_Line = EXTI_Line1;
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTIInit);
	USART1_Write("T4\n", 3);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART1_Write("T4\n", 3);
}

int main(int argc, char* argv[]) {
	(void) argc;
	(void) argv;

	// Send a greeting to the trace device (skipped on Release).
	trace_puts("Hello ARM World!");

	// At this stage the system clock should have already been configured
	// at high speed.
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	timer_start();

	USART1_Init(256000);

	USART1_Write("START1\n", 7);

	blink_led_init();

	USART1_Write("START2\n", 7);

	//tmp();

	USART1_Write("T5\n", 3);

	uint8_t error_number = gyro_init();
	if (error_number) {
		error_number = (uint8_t) (error_number + '0');
		while (1) {
			USART1_Write(&error_number, 1);
			USART1_Write(" WRONG\n", 7);
		}
	}
	USART1_Write(&error_number, 1);
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

		last = time;
		if (time - last >= 1000000) {
			last = time;
			char buffer[33];

			extern volatile int data_read;
			itoa((int) data_read, buffer, 10);
			data_read = 0;
			USART1_Write(buffer, (uint8_t) strlen(buffer));

			USART1_Write(" ", 1);

			itoa((int) readSec, buffer, 10);
			USART1_Write(buffer, (uint8_t) strlen(buffer));

			USART1_Write(" ", 1);

			itoa((int) readErr, buffer, 10);
			USART1_Write(buffer, (uint8_t) strlen(buffer));

			USART1_Write(" ", 1);

			itoa((int) readWait, buffer, 10);
			USART1_Write(buffer, (uint8_t) strlen(buffer));

			USART1_Write(" ", 1);

			itoa((int) time, buffer, 10);
			USART1_Write(buffer, (uint8_t) strlen(buffer));

			USART1_Write(" alive\n", 7);
			readSec = readErr = readWait = 0;
			/*
			 static int a = 0;
			 if (a) {
			 GPIO_SetBits(GPIOB, GPIO_Pin_1);
			 a = 0;
			 } else {
			 GPIO_ResetBits(GPIOB, GPIO_Pin_1);
			 a = 1;
			 }
			 */
		}
		/*
		 struct vector3f gyro, acce;
		 float temp;
		 */
		error_number = gyro_update(time);

		if (error_number == 0) {
			readSec++;

			struct vector3f gyro, acce;

			gyro_get_data(&gyro);

			acce_get_data(&acce);

//			temp_get_data(&temp);

			static uint8_t count = 0;

			count++;

			if (count == 16) {
				count = 0;

				int16_t t;

				USART1_Write("GG", 2);
				t = gyro.x;
				USART1_Write(&t, 2);

				t = gyro.y;
				USART1_Write(&t, 2);

				t = gyro.z;
				USART1_Write(&t, 2);

				USART1_Write("\n", 1);

				USART1_Write("AA", 2);
				t = acce.x;
				USART1_Write(&t, 2);

				t = acce.y;
				USART1_Write(&t, 2);

				t = acce.z;
				USART1_Write(&t, 2);
			}
			//USART1_Write("r\n", 2);
		} else {
			if (error_number != 1) {
				//ris = (uint8_t) (ris + '0');//readable ascii
				USART1_Write(&error_number, 1);
				USART1_Write(" ERROR\n", 7);
				readErr++;
			} else {
				//USART1_Write("NR\n", 3);
				readWait++;
			}

		}

	}

// Infinite loop, never return.
	return -1;
}

// ----------------------------------------------------------------------------
