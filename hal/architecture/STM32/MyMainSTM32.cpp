/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2020 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * Arduino core for STM32: https://github.com/stm32duino/Arduino_Core_STM32
 *
 * MySensors Arduino Core STM32 implementation, Copyright (C) 2019-2020 Olivier Mauti <olivier@mysensors.org>
 */

#define ARDUINO_MAIN
#include "Arduino.h"

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need HAL may fail.
__attribute__((constructor(101))) void premain()
{

	// Required by FreeRTOS, see http://www.freertos.org/RTOS-Cortex-M3-M4.html
#ifdef NVIC_PRIORITYGROUP_4
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
#endif
#if (__CORTEX_M == 0x07U)
	// Defined in CMSIS core_cm7.h
#ifndef I_CACHE_DISABLED
	SCB_EnableICache();
#endif
#ifndef D_CACHE_DISABLED
	SCB_EnableDCache();
#endif
#endif

	init();
}

/*
 * \brief Main entry point of Arduino application
 */
int main(void)
{
	initVariant();

	_begin(); // Startup MySensors library

	for (;;) {
#if defined(CORE_CALLBACK)
		CoreCallback();
#endif
		_process();	// Process incoming data
		loop();
		if (serialEventRun) {
			serialEventRun();
		}
	}

	return 0;
}
