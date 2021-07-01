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
 */

#include "MyHwSTM32.h"

/*
* Pinout STM32F103C8 dev board:
* http://wiki.stm32duino.com/images/a/ae/Bluepillpinout.gif
*
* Wiring
* --------------------------------------------------
RFM69 	CLK		MISO		MOSI	CSN		CE		IRQ
SPI1		PA5		PA6			PA7		PA4		NA		PA3 (default)

RF24 		CLK		MISO		MOSI	CSN		CE		IRQ
SPI1		PA5		PA6			PA7		PA4		PB0		NA

*/

bool hwInit(void)
{
#if !defined(MY_DISABLED_SERIAL)
	MY_SERIALDEVICE.begin(MY_BAUD_RATE);
#if defined(MY_GATEWAY_SERIAL)
	while (!MY_SERIALDEVICE) {}
#endif
#endif

	return true;
}

void hwReadConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *dst = static_cast<uint8_t *>(buf);
	int offs = reinterpret_cast<int>(addr);
	eeprom_buffer_fill();
	while (length-- > 0) {
		*dst++ = eeprom_buffered_read_byte(offs++);
	}
}

void hwWriteConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *src = static_cast<uint8_t *>(buf);
	int offs = reinterpret_cast<int>(addr);
	while (length-- > 0) {
		eeprom_buffered_write_byte(offs++, *src++);
	}
	eeprom_buffer_flush();
}

uint8_t hwReadConfig(const int addr)
{
	uint8_t value;
	hwReadConfigBlock(&value, reinterpret_cast<void *>(addr), 1);
	return value;
}

void hwWriteConfig(const int addr, uint8_t value)
{
	if (hwReadConfig(addr) != value) {
		hwWriteConfigBlock(&value, reinterpret_cast<void *>(addr), 1);
	}
}

int8_t hwSleep(uint32_t ms)
{
	// TODO: Not supported!
	(void)ms;
	return MY_SLEEP_NOT_POSSIBLE;
}

int8_t hwSleep(const uint8_t interrupt, const uint8_t mode, uint32_t ms)
{
	// TODO: Not supported!
	(void)interrupt;
	(void)mode;
	(void)ms;
	return MY_SLEEP_NOT_POSSIBLE;
}

int8_t hwSleep(const uint8_t interrupt1, const uint8_t mode1, const uint8_t interrupt2,
               const uint8_t mode2,
               uint32_t ms)
{
	// TODO: Not supported!
	(void)interrupt1;
	(void)mode1;
	(void)interrupt2;
	(void)mode2;
	(void)ms;
	return MY_SLEEP_NOT_POSSIBLE;
}



bool hwUniqueID(unique_id_t *uniqueID)
{
	// Fill ID with FF
	(void)memset((uint8_t *)uniqueID,  0xFF, 16);
	// Read device ID
	(void)memcpy((uint8_t *)uniqueID, (uint32_t *)UID_BASE, 12);

	return true;
}

uint16_t hwCPUVoltage(void)
{
	//Not yet implemented
	return FUNCTION_NOT_SUPPORTED;
}

uint16_t hwCPUFrequency(void)
{
	return HAL_RCC_GetSysClockFreq()/1000000UL;
}

int8_t hwCPUTemperature(void)
{
	return FUNCTION_NOT_SUPPORTED;
}

uint16_t hwFreeMem(void)
{
	//Not yet implemented
	return FUNCTION_NOT_SUPPORTED;
}


void hwWatchdogReset(void)
{
	IWatchdog.reload();
}

void hwReboot(void)
{
	NVIC_SystemReset();
	while (true)
		;
}
