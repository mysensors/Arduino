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

#ifndef MyHwSTM32_h
#define MyHwSTM32_h

// #include <libmaple/iwdg.h>
#include <IWatchdog.h>
// #include <stm32yyxx_ll_iwdg.h>
#include <itoa.h>
#include <EEPROM.h>
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

#ifndef MY_STM32F1_TEMPERATURE_OFFSET
#define MY_STM32F1_TEMPERATURE_OFFSET (0.0f)
#endif

#ifndef MY_STM32F1_TEMPERATURE_GAIN
#define MY_STM32F1_TEMPERATURE_GAIN (1.0f)
#endif

// SS default
#ifndef SS
#define SS PA4
#endif

// mapping
#define snprintf_P snprintf
#define vsnprintf_P vsnprintf
#define strncpy_P strncpy
#define printf_P printf
#define yield()				  // not defined

#ifndef digitalPinToInterrupt
#define digitalPinToInterrupt(__pin) (__pin)
#endif

#define hwDigitalWrite(__pin, __value) digitalWrite(__pin, __value)
#define hwDigitalRead(__pin) digitalRead(__pin)
#define hwPinMode(__pin, __value) pinMode(__pin, __value)
#define hwWatchdogReset() IWatchdog.reload() // iwdg_feed()
// #define hwReboot() nvic_sys_reset()
#define hwReboot() HAL_NVIC_SystemReset()
#define hwMillis() millis()
#define hwGetSleepRemaining() (0ul)

extern void serialEventRun(void) __attribute__((weak));
bool hwInit(void);
void hwRandomNumberInit(void);
void hwReadConfigBlock(void *buf, void *addr, size_t length);
void hwWriteConfigBlock(void *buf, void *addr, size_t length);
// void hwWriteConfig(const int addr, uint8_t value);
// uint8_t hwReadConfig(const int addr);
#define hwReadConfig(__pos) eeprom_read_byte((const uint32_t)__pos)
#define hwWriteConfig(__pos, __val) eeprom_write_byte((const uint32_t)__pos, (uint8_t)__val)

// SOFTSPI
#ifdef MY_SOFTSPI
#error Soft SPI is not available on this architecture!
#endif
#define hwSPI SPI //!< hwSPI


#ifndef DOXYGEN
#define MY_CRITICAL_SECTION
#endif  /* DOXYGEN */

#endif
