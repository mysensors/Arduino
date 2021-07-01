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

#include "MyHwLinuxGeneric.h"

static SoftEeprom eeprom;
static FILE *randomFp = NULL;

bool hwInit(void)
{
	MY_SERIALDEVICE.begin(MY_BAUD_RATE);
#ifdef MY_GATEWAY_SERIAL
#ifdef MY_LINUX_SERIAL_GROUPNAME
	if (!MY_SERIALDEVICE.setGroupPerm(MY_LINUX_SERIAL_GROUPNAME)) {
		logError("Unable to change permission for serial port device.\n");
		exit(1);
	}
#endif
#endif

	if (eeprom.init(conf.eeprom_file, conf.eeprom_size) != 0) {
		exit(1);
	}

	logInfo("Using eeprom file %s\n", conf.eeprom_file);

	return true;
}

void hwReadConfigBlock(void *buf, void *addr, size_t length)
{
	eeprom.readBlock(buf, addr, length);
}

void hwWriteConfigBlock(void *buf, void *addr, size_t length)
{
	eeprom.writeBlock(buf, addr, length);
}

uint8_t hwReadConfig(const int addr)
{
	return eeprom.readByte(addr);
}

void hwWriteConfig(const int addr, uint8_t value)
{
	eeprom.writeByte(addr, value);
}

void hwRandomNumberInit(void)
{
	uint32_t seed=0;

	if (randomFp != NULL) {
		fclose(randomFp);
	}
	if (!(randomFp = fopen("/dev/urandom", "r"))) {
		logError("Cannot open '/dev/urandom'.\n");
		exit(2);
	}

	while (hwGetentropy(&seed, sizeof(seed)) != sizeof(seed));
	randomSeed(seed);
}

ssize_t hwGetentropy(void *__buffer, size_t __length)
{
	return(fread(__buffer, 1, __length, randomFp));
}

uint32_t hwMillis(void)
{
	return millis();
}

bool hwUniqueID(unique_id_t *uniqueID)
{
	// not implemented yet
	(void)uniqueID;
	return false;
}

// Not supported!
int8_t hwSleep(uint32_t ms)
{
	(void)ms;

	return MY_SLEEP_NOT_POSSIBLE;
}

// Not supported!
int8_t hwSleep(const uint8_t interrupt, const uint8_t mode, uint32_t ms)
{
	(void)interrupt;
	(void)mode;
	(void)ms;

	return MY_SLEEP_NOT_POSSIBLE;
}

// Not supported!
int8_t hwSleep(const uint8_t interrupt1, const uint8_t mode1, const uint8_t interrupt2,
               const uint8_t mode2,
               uint32_t ms)
{
	(void)interrupt1;
	(void)mode1;
	(void)interrupt2;
	(void)mode2;
	(void)ms;

	return MY_SLEEP_NOT_POSSIBLE;
}

uint16_t hwCPUVoltage(void)
{
	// TODO: Not supported!
	return FUNCTION_NOT_SUPPORTED;
}

uint16_t hwCPUFrequency(void)
{
	// TODO: Not supported!
	return FUNCTION_NOT_SUPPORTED;
}

int8_t hwCPUTemperature(void)
{
	return -127;  // not implemented yet
}

uint16_t hwFreeMem(void)
{
	// TODO: Not supported!
	return FUNCTION_NOT_SUPPORTED;
}

void hwDigitalWrite(uint8_t pin, uint8_t value)
{
	digitalWrite(pin, value);
}

int hwDigitalRead(uint8_t pin)
{
	return digitalRead(pin);
}

void hwPinMode(uint8_t pin, uint8_t mode)
{
	pinMode(pin, mode);
}
