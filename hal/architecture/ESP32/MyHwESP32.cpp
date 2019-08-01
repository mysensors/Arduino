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
 * Arduino core for ESP32: https://github.com/espressif/arduino-esp32
 *
 * MySensors ESP32 implementation, Copyright (C) 2017-2019 Olivier Mauti <olivier@mysensors.org>
 *
 */

#include "MyHwESP32.h"

bool hwInit(void)
{
#if !defined(MY_DISABLED_SERIAL)
	MY_SERIALDEVICE.begin(MY_BAUD_RATE, SERIAL_8N1);
#if defined(MY_GATEWAY_SERIAL)
	while (!MY_SERIALDEVICE) {}
#endif
#endif
	return EEPROM.begin(MY_EEPROM_SIZE);
}

void hwReadConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *dst = static_cast<uint8_t *>(buf);
	int offs = reinterpret_cast<int>(addr);
	while (length-- > 0) {
		*dst++ = EEPROM.read(offs++);
	}
}

void hwWriteConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *src = static_cast<uint8_t *>(buf);
	int offs = reinterpret_cast<int>(addr);
	while (length-- > 0) {
		EEPROM.write(offs++, *src++);
	}
	EEPROM.commit();
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

bool hwUniqueID(unique_id_t *uniqueID)
{
	uint64_t val = ESP.getEfuseMac();
	(void)memcpy((void *)uniqueID, (void *)&val, 8);
	(void)memset((void *)(uniqueID + 8), MY_HWID_PADDING_BYTE, 8); // padding
	return true;
}

ssize_t hwGetentropy(void *__buffer, size_t __length)
{
	// cut length if > 256
	if (__length > 256) {
		__length = 256;
	}
	uint8_t *dst = (uint8_t *)__buffer;
	// get random numbers
	for (size_t i = 0; i < __length; i++) {
		dst[i] = (uint8_t)esp_random();
	}
	return __length;
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

uint16_t hwCPUVoltage(void)
{
	// experimental, not documented feature and inaccurate?
	uint16_t internalBatReading;
	if (WiFi.status() == 255) {
		btStart();
		internalBatReading = rom_phy_get_vdd33();
		btStop();
	} else {
		internalBatReading = rom_phy_get_vdd33();
	}
	return internalBatReading;
}

uint16_t hwCPUFrequency(void)
{
	// in 1/10Mhz
	return static_cast<uint16_t>(ESP.getCpuFreqMHz() * 10);
}

int8_t hwCPUTemperature(void)
{
	// CPU temperature in °C
	return static_cast<int8_t>((temperatureRead() - MY_ESP32_TEMPERATURE_OFFSET) /
	                           MY_ESP32_TEMPERATURE_GAIN);
}

uint16_t hwFreeMem(void)
{
	return static_cast<uint16_t>(ESP.getFreeHeap());
}
