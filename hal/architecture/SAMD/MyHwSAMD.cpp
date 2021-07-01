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

#include "MyHwSAMD.h"

/*
int8_t pinIntTrigger = 0;
void wakeUp()	 //place to send the interrupts
{
	pinIntTrigger = 1;
}
void wakeUp2()	 //place to send the second interrupts
{
	pinIntTrigger = 2;
}

// Watchdog Timer interrupt service routine. This routine is required
// to allow automatic WDIF and WDIE bit clearance in hardware.
ISR (WDT_vect)
{
	// WDIE & WDIF is cleared in hardware upon entering this ISR
	wdt_disable();
}
*/

// dtostrf is deprecated, adding missing function here
char* dtostrf(double val, signed char width, unsigned char prec, char* sout)
{
	asm(".global _printf_float");

	char fmt[20];
	sprintf(fmt, "%%%d.%df", width, prec);
	sprintf(sout, fmt, val);
	return sout;
}

void hwReadConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *dst = static_cast<uint8_t *>(buf);
	const int offs = reinterpret_cast<int>(addr);
	(void)eep.read(offs, dst, length);
}

void hwWriteConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *src = static_cast<uint8_t *>(buf);
	const int offs = reinterpret_cast<int>(addr);
	// use update() instead of write() to reduce e2p wear off
	(void)eep.update(offs, src, length);
}

uint8_t hwReadConfig(const int addr)
{
	return eep.read(addr);
}

void hwWriteConfig(const int addr, uint8_t value)
{
	(void)eep.update(addr, value);
}

bool hwInit(void)
{
#if !defined(MY_DISABLED_SERIAL)
	MY_SERIALDEVICE.begin(MY_BAUD_RATE);
#if defined(MY_GATEWAY_SERIAL)
	while (!MY_SERIALDEVICE) {}
#endif
#endif

	SYSCTRL->VREF.reg |= SYSCTRL_VREF_TSEN; // Enable the temperature sensor
	while (ADC->STATUS.bit.SYNCBUSY ==
	        1); // Wait for synchronization of registers between the clock domains

	const uint8_t eepInit = eep.begin(MY_EXT_EEPROM_TWI_CLOCK, &Wire);
#if defined(SENSEBENDER_GW_SAMD_V1)
	// check connection to external EEPROM - only sensebender GW
	return eepInit==0;
#else
	(void)eepInit;
	return true;
#endif
}

void hwWatchdogReset(void)
{
	// TODO: Not supported!
}

void hwReboot(void)
{
	NVIC_SystemReset();
	while (true);
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
	(void)memcpy((uint8_t *)uniqueID, (uint32_t *)0x0080A00C, 4);
	(void)memcpy((uint8_t *)uniqueID + 4, (uint32_t *)0x0080A040, 12);
	return true;
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC()
{
	while (ADC->STATUS.bit.SYNCBUSY);
}

uint16_t hwCPUVoltage(void)
{
	// Set ADC reference to internal 1v
	ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;
	ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;
	syncADC();
	// Set to 10 bits reading resolution
	ADC->CTRLB.reg = ADC_CTRLB_RESSEL_10BIT | ADC_CTRLB_PRESCALER_DIV256;
	syncADC();
	// Select MUXPOS as SCALEDIOVCC/4 channel, and MUXNEG as internal ground
	ADC->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC_Val;
	ADC->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;
	syncADC();
	// enable ADC
	ADC->CTRLA.bit.ENABLE = 1;
	syncADC();
	// start conversion
	ADC->SWTRIG.bit.START = 1;
	// clear the Data Ready flag
	ADC->INTFLAG.bit.RESRDY = 1;
	syncADC();
	// start conversion again, since The first conversion after the reference is changed must not be used.
	ADC->SWTRIG.bit.START = 1;
	// waiting for conversion to complete
	while (!ADC->INTFLAG.bit.RESRDY);
	syncADC();
	const uint32_t valueRead = ADC->RESULT.reg;
	// disable ADC
	ADC->CTRLA.bit.ENABLE = 0;
	syncADC();
	// value is 1/4 scaled, multiply by 4
	return valueRead * 4;
}

uint16_t hwCPUFrequency(void)
{
	// TODO: currently reporting compile time frequency (in 1/10MHz)
	return F_CPU / 100000UL;
}

int8_t hwCPUTemperature(void)
{
	// taken from https://github.com/arduino/ArduinoCore-samd/pull/277
	// Set to 12 bits resolution
	ADC->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT | ADC_CTRLB_PRESCALER_DIV256;
	syncADC();
	// Ensure we are sampling slowly
	ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(0x3f);
	syncADC();
	// Set ADC reference to internal 1v
	ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;
	ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;
	syncADC();
	// Select MUXPOS as temperature channel, and MUXNEG as internal ground
	ADC->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_TEMP_Val;
	ADC->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;
	syncADC();
	// Enable ADC
	ADC->CTRLA.bit.ENABLE = 1;
	syncADC();
	// Start ADC conversion
	ADC->SWTRIG.bit.START = 1;
	// Clear the Data Ready flag
	ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
	syncADC();
	// Start conversion again, since The first conversion after the reference is changed must not be used.
	ADC->SWTRIG.bit.START = 1;
	// Wait until ADC conversion is done
	while (!(ADC->INTFLAG.bit.RESRDY));
	syncADC();
	// Get result
	// This is signed so that the math later is done signed
	const int32_t adcReading = ADC->RESULT.reg;
	// Clear result ready flag
	ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
	syncADC();
	// Disable ADC
	ADC->CTRLA.bit.ENABLE = 0;
	syncADC();
	// Factory room temperature readings
	const uint8_t roomInteger = (*(uint32_t *)FUSES_ROOM_TEMP_VAL_INT_ADDR &
	                             FUSES_ROOM_TEMP_VAL_INT_Msk)
	                            >> FUSES_ROOM_TEMP_VAL_INT_Pos;
	const uint8_t roomDecimal = (*(uint32_t *)FUSES_ROOM_TEMP_VAL_DEC_ADDR &
	                             FUSES_ROOM_TEMP_VAL_DEC_Msk)
	                            >> FUSES_ROOM_TEMP_VAL_DEC_Pos;
	const int32_t roomReading = ((*(uint32_t *)FUSES_ROOM_ADC_VAL_ADDR & FUSES_ROOM_ADC_VAL_Msk) >>
	                             FUSES_ROOM_ADC_VAL_Pos);
	const int32_t roomTemperature = 1000 * roomInteger + 100 * roomDecimal;
	// Factory hot temperature readings
	const uint8_t hotInteger = (*(uint32_t *)FUSES_HOT_TEMP_VAL_INT_ADDR & FUSES_HOT_TEMP_VAL_INT_Msk)
	                           >>
	                           FUSES_HOT_TEMP_VAL_INT_Pos;
	const uint8_t hotDecimal = (*(uint32_t *)FUSES_HOT_TEMP_VAL_DEC_ADDR & FUSES_HOT_TEMP_VAL_DEC_Msk)
	                           >>
	                           FUSES_HOT_TEMP_VAL_DEC_Pos;
	const int32_t hotReading = ((*(uint32_t *)FUSES_HOT_ADC_VAL_ADDR & FUSES_HOT_ADC_VAL_Msk) >>
	                            FUSES_HOT_ADC_VAL_Pos);
	const int32_t hotTemperature = 1000 * hotInteger + 100 * hotDecimal;
	// Linear interpolation of temperature using factory room temperature and hot temperature
	const int32_t temperature = roomTemperature + ((hotTemperature - roomTemperature) *
	                            (adcReading - roomReading)) / (hotReading - roomReading);
	return static_cast<int8_t>(((temperature / 1000) - MY_SAMD_TEMPERATURE_OFFSET) /
	                           MY_SAMD_TEMPERATURE_GAIN);
}


uint16_t hwFreeMem(void)
{
	// TODO: Not supported!
	return FUNCTION_NOT_SUPPORTED;
}
