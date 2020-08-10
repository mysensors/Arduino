/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#ifndef MyHwLGT8F328P_h
#define MyHwLGT8F328P_h

// #include <avr/eeprom.h>
#include <EEPROM.h>
// #include <avr/pgmspace.h>
// #include <avr/sleep.h>
// #include <avr/power.h>
// #include <avr/interrupt.h>
// #include <avr/wdt.h>
#include <WDT.h>
// #include <avr/boot.h>
// #include <util/atomic.h>
#include <SPI.h>

// // Fast IO driver
// #include "drivers/DigitalWriteFast/digitalWriteFast.h"

// // SOFTSPI
// #ifdef MY_SOFTSPI
// #include "hal/architecture/AVR/drivers/DigitalIO/DigitalIO.h"
// #endif

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

// AVR temperature calibration reference: http://ww1.microchip.com/downloads/en/AppNotes/Atmel-8108-Calibration-of-the-AVRs-Internal-Temperature-Reference_ApplicationNote_AVR122.pdf
// #ifndef MY_AVR_TEMPERATURE_OFFSET
// #define MY_AVR_TEMPERATURE_OFFSET (324.31f)
// #endif

// #ifndef MY_AVR_TEMPERATURE_GAIN
// #define MY_AVR_TEMPERATURE_GAIN (1.22f)
// #endif

// Define these as macros to save valuable space
// #define hwDigitalWrite(__pin, __value) fastioWrite(__pin, __value)
// #define hwDigitalRead(__pin) fastioRead(__pin)
// #define hwPinMode(__pin, __value) fastioMode(__pin, __value)
#define hwDigitalWrite(__pin, __value) digitalWrite(__pin, __value)
#define hwDigitalRead(__pin) digitalRead(__pin)
#define hwPinMode(__pin, __value) pinMode(__pin, __value)

bool hwInit(void);

#define hwWatchdogReset() wdt_reset()
#define hwMillis() millis()
#define hwReboot() wdt_enable(WDTO_15MS); while (1)
#define hwReadConfig(__pos) EEPROM.read((const uint8_t *)__pos)
#define hwWriteConfig(__pos, __val) EEPROM.write((uint8_t *)__pos, (uint8_t)__val)
#define hwReadConfigBlock(__buf, __pos, __length) EEPROM.read_block((void *)__buf, (const void *)__pos, (uint32_t)__length)
#define hwWriteConfigBlock(__buf, __pos, __length) EEPROM.write_block((const void *)__buf, (void *)__pos, (uint32_t)__length)

inline void hwRandomNumberInit(void);
uint32_t hwInternalSleep(uint32_t ms);

#define sleep_disable() (SMCR &= ~_BV(SE))

// SOFTSPI
#if defined(MY_SOFTSPI)
// SoftSPI<MY_SOFT_SPI_MISO_PIN, MY_SOFT_SPI_MOSI_PIN, MY_SOFT_SPI_SCK_PIN, 0> hwSPI; //!< hwSPI
#error Soft SPI is not available on this architecture!
#else
#define hwSPI SPI //!< hwSPI
#endif

#ifndef DOXYGEN
#define MY_CRITICAL_SECTION     ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif  /* DOXYGEN */

#endif
