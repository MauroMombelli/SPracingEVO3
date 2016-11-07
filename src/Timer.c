//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#include "Timer.h"
#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------

#if defined(USE_HAL_DRIVER)
void HAL_IncTick(void);
#endif

// Forward declarations.

void
timer_tick(void);

// ----------------------------------------------------------------------------

volatile timer_ticks_t timer_delayCount;

// ----------------------------------------------------------------------------

void timer_start(void) {
	// Use SysTick as reference for the delay loops.
	SysTick_Config(SystemCoreClock / TIMER_FREQUENCY_HZ / 100); //every 10us
}

void timer_sleep(timer_ticks_t ticks) {
	timer_ticks_t start = timer_delayCount / 100;

	// Busy wait until the SysTick decrements the counter to zero.
	while (timer_delayCount / 100 - start < ticks){
		;
	}
}

uint32_t millis(void){
	return timer_delayCount / 100;
}

uint32_t micros(void){
	return timer_delayCount * 10;
}

void timer_tick(void) {
	timer_delayCount++;
}

// ----- SysTick_Handler() ----------------------------------------------------

void SysTick_Handler(void) {
#if defined(USE_HAL_DRIVER)
	HAL_IncTick();
#endif
	timer_tick();
}

// ----------------------------------------------------------------------------
