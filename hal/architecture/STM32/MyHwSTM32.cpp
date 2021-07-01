/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of
 * the network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list:
 * https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include "MyHwSTM32.h"
#include "drivers/hal_conf_custom.h"
#include "drivers/stm32_ADC_internal_channels.h"

/*
 * Pinout STM32F103C8 dev board:
 * http://wiki.stm32duino.com/images/a/ae/Bluepillpinout.gif
 *
 * Wiring RFM69 radio / SPI1
 * --------------------------------------------------
 * CLK	PA5
 * MISO	PA6
 * MOSI	PA7
 * CSN	PA4
 * CE	NA
 * IRQ	PA3 (default)
 *
 * Wiring RF24 radio / SPI1
 * --------------------------------------------------
 * CLK	PA5
 * MISO	PA6
 * MOSI	PA7
 * CSN	PA4
 * CE	PB0 (default)
 * IRQ	NA
 *
 */
bool hwInit(void) {
#if !defined(MY_DISABLED_SERIAL)
  MY_SERIALDEVICE.begin(MY_BAUD_RATE);
#if defined(MY_GATEWAY_SERIAL)
  while (!MY_SERIALDEVICE) {
  }
#endif
#endif
  return true;
  // return EEPROM.begin();

  // if (EEPROM.init() == EEPROM_OK) {
  // 	uint16 cnt;
  // 	EEPROM.count(&cnt);
  // 	if(cnt>=EEPROM.maxcount()) {
  // 		// tmp, WIP: format eeprom if full
  // 		EEPROM.format();
  // 	}
  // 	return true;
  // }
  // return false;
}

void hwReadConfigBlock(void *buf, void *addr, size_t length) {
  uint8_t *dst = static_cast<uint8_t *>(buf);
  int pos = reinterpret_cast<int>(addr);
  eeprom_buffer_fill();
  // return eeprom_buffered_read_byte(pos);

  // while (length-- > 0) {
  //   *dst++ = EEPROM.read(pos++);
  // }
  while (length-- > 0) {
    *dst++ = eeprom_buffered_read_byte(pos++);
  }
}

void hwWriteConfigBlock(void *buf, void *addr, size_t length) {
  uint8_t *src = static_cast<uint8_t *>(buf);
  int pos = reinterpret_cast<int>(addr);
  // while (length-- > 0) {
  //   EEPROM.write(pos++, *src++);
  // }
  while (length-- > 0) {
    // EEPROM.write(pos++, *src++);
    eeprom_buffered_write_byte(pos++, *src++);
  }
  eeprom_buffer_flush();
}

// void hwReadConfigBlock(void *buf, void *addr, size_t length) {
//   uint8_t *dst = static_cast<uint8_t *>(buf);
//   int pos = reinterpret_cast<int>(addr);
//   while (length-- > 0) {
//     *dst++ = EEPROM.read(pos++);
//   }
// }

// void hwWriteConfigBlock(void *buf, void *addr, size_t length) {
//   uint8_t *src = static_cast<uint8_t *>(buf);
//   int pos = reinterpret_cast<int>(addr);
//   while (length-- > 0) {
//     EEPROM.write(pos++, *src++);
//   }
// }

// uint8_t hwReadConfig(const int addr) {
//   uint8_t value;
//   hwReadConfigBlock(&value, reinterpret_cast<void *>(addr), 1);
//   return value;
// }

// void hwWriteConfig(const int addr, uint8_t value) {
//   hwWriteConfigBlock(&value, reinterpret_cast<void *>(addr), 1);
// }

int8_t hwSleep(uint32_t ms) {
  // TODO: Not supported!
  (void)ms;
  return MY_SLEEP_NOT_POSSIBLE;
}

int8_t hwSleep(const uint8_t interrupt, const uint8_t mode, uint32_t ms) {
  // TODO: Not supported!
  (void)interrupt;
  (void)mode;
  (void)ms;
  return MY_SLEEP_NOT_POSSIBLE;
}

int8_t hwSleep(const uint8_t interrupt1, const uint8_t mode1,
               const uint8_t interrupt2, const uint8_t mode2, uint32_t ms) {
  // TODO: Not supported!
  (void)interrupt1;
  (void)mode1;
  (void)interrupt2;
  (void)mode2;
  (void)ms;
  return MY_SLEEP_NOT_POSSIBLE;
}

void hwRandomNumberInit(void) {
  // use internal temperature sensor as noise source
  uint32_t seed = 0;
  uint16_t currentValue = 0;
  uint16_t newValue = 0;

  for (uint8_t i = 0; i < 32; i++) {
    const uint32_t timeout = hwMillis() + 20;
    while (timeout >= hwMillis()) {
      newValue = analogRead(ATEMP);
      if (newValue != currentValue) {
        currentValue = newValue;
        break;
      }
    }
    seed ^= ((newValue + hwMillis()) & 7) << i;
  }
  randomSeed(seed);
}

bool hwUniqueID(unique_id_t *uniqueID) {
  (void)memcpy((uint8_t *)uniqueID, (uint32_t *)0x1FFFF7E0,
               16); // FlashID + ChipID
  return true;
}

uint16_t hwCPUVoltage(void) { return (uint16_t)readVref(); }

uint16_t hwCPUFrequency(void) { return F_CPU / 100000UL; }

int8_t hwCPUTemperature(void) { return (int8_t)readTempSensor(readVref()); }

uint16_t hwFreeMem(void) {
  // Not yet implemented
  return FUNCTION_NOT_SUPPORTED;
}
