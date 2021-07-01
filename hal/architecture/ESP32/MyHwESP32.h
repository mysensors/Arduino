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
 * MySensors ESP32 implementation, Copyright (C) 2017-2018 Olivier Mauti <olivier@mysensors.org>
 *
 * Radio wiring ESP32(Node32s): RF24, RFM69, RFM95:
 *
 * | IO   | RF24 | RFM69 | RFM95 |
 * |------|------|-------|-------|
 * | MOSI | 23   | 23    | 23    |
 * | MISO | 19   | 19    | 19    |
 * | SCK  | 18   | 18    | 18    |
 * | CSN  | 5    | 5     | 5     |
 * | CE   | 17   | -     | -     |
 * | RST  | -    | 17    | 17    |
 * | IRQ  | 16*  | 16    | 16    |
 * * = optional
 *
 */

#ifndef MyHwESP32_h
#define MyHwESP32_h

#include <WiFi.h>
#include "EEPROM.h"
#include <SPI.h>

#ifdef __cplusplus
#include <Arduino.h>
#endif

#define CRYPTO_LITTLE_ENDIAN

#ifndef MY_SERIALDEVICE
#define MY_SERIALDEVICE Serial
#endif

#ifndef MY_DEBUGDEVICE
#define MY_DEBUGDEVICE MY_SERIALDEVICE
#endif

#ifndef MY_ESP32_TEMPERATURE_OFFSET
#define MY_ESP32_TEMPERATURE_OFFSET (0.0f)
#endif

#ifndef MY_ESP32_TEMPERATURE_GAIN
#define MY_ESP32_TEMPERATURE_GAIN (1.0f)
#endif

#define MY_EEPROM_SIZE 1024

#define hwDigitalWrite(__pin, __value) digitalWrite(__pin, __value)
#define hwDigitalRead(__pin) digitalRead(__pin)
#define hwPinMode(__pin, __value) pinMode(__pin, __value)
#define hwWatchdogReset()
#define hwReboot() ESP.restart()
#define hwMillis() millis()
#define hwMicros() micros()
#define hwRandomNumberInit() randomSeed(esp_random())
#define hwGetSleepRemaining() (0ul)

// experimental, not documented feature
extern "C" int rom_phy_get_vdd33();

bool hwInit(void);
void hwReadConfigBlock(void *buf, void *addr, size_t length);
void hwWriteConfigBlock(void *buf, void *addr, size_t length);
void hwWriteConfig(const int addr, uint8_t value);
uint8_t hwReadConfig(const int addr);
ssize_t hwGetentropy(void *__buffer, size_t __length);
#define MY_HW_HAS_GETENTROPY

// SOFTSPI
#ifdef MY_SOFTSPI
#error Soft SPI is not available on this architecture!
#endif
#define hwSPI SPI //!< hwSPI

/**
* Restore interrupt state.
* Helper function for MY_CRITICAL_SECTION.
*/
static __inline__ void __psRestore(const uint32_t *__s)
{
	XTOS_RESTORE_INTLEVEL(*__s);
}

#ifndef DOXYGEN
#define MY_CRITICAL_SECTION    for ( uint32_t __psSaved __attribute__((__cleanup__(__psRestore))) = XTOS_DISABLE_ALL_INTERRUPTS, __ToDo = 1; __ToDo ; __ToDo = 0 )
#endif  /* DOXYGEN */


#endif
