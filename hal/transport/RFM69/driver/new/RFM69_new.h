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
 * RFM69 driver refactored for MySensors
 *
 * Based on :
 * - LowPowerLab RFM69 Lib Copyright Felix Rusu (2014), felix@lowpowerlab.com
 * - Automatic Transmit Power Control class derived from RFM69 library.
 *	  Discussion and details in this forum post: https://lowpowerlab.com/forum/index.php/topic,688.0.html
 *	  Copyright Thomas Studwell (2014,2015)
 * - MySensors generic radio driver implementation Copyright (C) 2017, 2018 Olivier Mauti <olivier@mysensors.org>
 *
 * Changes by : @tekka, @scalz, @marceloagno
 *
 * Definitions for Semtech SX1231/H radios:
 * https://www.semtech.com/uploads/documents/sx1231.pdf
 * https://www.semtech.com/uploads/documents/sx1231h.pdf
 */


/**
* @file RFM69_new.h
*
* @defgroup RFM69Newgrp RFM69New
* @ingroup internals
* @{
*
* RFM69 driver-related log messages, format: [!]SYSTEM:[SUB SYSTEM:]MESSAGE
* - [!] Exclamation mark is prepended in case of error
*
* |E| SYS   | SUB  | Message                              | Comment
* |-|-------|------|--------------------------------------|-----------------------------------------------------------------------------------
* | | RFM69 | INIT |                                      | Initialise RFM69 radio
* | | RFM69 | INIT | PIN,CS=%%d,IQP=%%d,IQN=%%d[,RST=%%d] | Pin configuration: chip select (CS), IRQ pin (IQP), IRQ number (IQN), Reset (RST)
* | | RFM69 | INIT | HWV=%%d                              | HW version, see datasheet chapter 9
* |!| RFM69 | INIT | SANCHK FAIL                          | Sanity check failed, check wiring or replace module
* | | RFM69 | PTX  | NO ADJ                               | TX power level, no adjustment
* | | RFM69 | PTX  | LEVEL=%%d dbM                        | TX power level, set to (LEVEL) dBm
* | | RFM69 | SAC  | SEND ACK,TO=%%d,RSSI=%%d             | ACK sent to (TO), RSSI of incoming message (RSSI)
* | | RFM69 | ATC  | ADJ TXL,cR=%%d,tR=%%d..%%d,TXL=%%d   | Adjust TX level, current RSSI (cR), target RSSI range (tR), TX level (TXL)
* | | RFM69 | SWR  | SEND,TO=%%d,SEQ=%%d,RETRY=%%d        | Send to (TO), sequence number (SWQ), retry if no ACK received (RETRY)
* | | RFM69 | SWR  | ACK,FROM=%%d,SEQ=%%d,RSSI=%%d        | ACK received from (FROM), sequence nr (SEQ), ACK RSSI (RSSI)
* |!| RFM69 | SWR  | NACK                                 | Message sent, no ACK received
* | | RFM69 | SPP  | PCT=%%d,TX LEVEL=%%d                 | Set TX level, input TX percent (PCT)
* | | RFM69 | RSL  |                                      | Radio in sleep mode
* | | RFM69 | RSB  |                                      | Radio in standby mode
* | | RFM69 | PWD  |                                      | Power down radio
* | | RFM69 | PWU  |                                      | Power up radio
*
* @brief API declaration for RFM69
*
*/

#ifndef _RFM69_h
#define _RFM69_h

#include "RFM69registers_new.h"

#if !defined(RFM69_SPI)
#define RFM69_SPI hwSPI //!< default SPI
#endif

#if defined(ARDUINO_ARCH_AVR)
#if defined(__AVR_ATmega32U4__)
#define DEFAULT_RFM69_IRQ_PIN			(3)												//!< DEFAULT_RFM69_IRQ_PIN
#else
#define DEFAULT_RFM69_IRQ_PIN			(2)												//!< DEFAULT_RFM69_IRQ_PIN
#endif
#elif defined(ARDUINO_ARCH_ESP8266)
#define DEFAULT_RFM69_IRQ_PIN			(5)												//!< DEFAULT_RFM69_IRQ_PIN
#elif defined(ARDUINO_ARCH_ESP32)
#define DEFAULT_RFM69_IRQ_PIN			(16)											//!< DEFAULT_RFM69_IRQ_PIN
#define DEFAULT_RFM69_IRQ_NUM			digitalPinToInterrupt(DEFAULT_RFM69_IRQ_PIN)	//!< DEFAULT_RFM69_IRQ_NUM
#elif defined(ARDUINO_ARCH_SAMD)
#define DEFAULT_RFM69_IRQ_PIN			(2)												//!< DEFAULT_RFM69_IRQ_PIN
#elif defined(LINUX_ARCH_RASPBERRYPI)
#define DEFAULT_RFM69_IRQ_PIN			(22)											//!< DEFAULT_RFM69_IRQ_PIN
#elif defined(ARDUINO_ARCH_STM32)
#define DEFAULT_RFM69_IRQ_PIN			(PA3)											//!< DEFAULT_RFM69_IRQ_PIN
#elif defined(ARDUINO_ARCH_STM32F1)
#define DEFAULT_RFM69_IRQ_PIN			(PA3)											//!< DEFAULT_RFM69_IRQ_PIN
#elif defined(TEENSYDUINO)
#define DEFAULT_RFM69_IRQ_PIN			(8)												//!< DEFAULT_RFM69_IRQ_PIN
#else
#define DEFAULT_RFM69_IRQ_PIN			(2)												//!< DEFAULT_RFM69_IRQ_PIN
#endif

#define DEFAULT_RFM69_CS_PIN			(SS)											//!< DEFAULT_RFM69_CS_PIN

// SPI settings
#define RFM69_SPI_DATA_ORDER			MSBFIRST		//!< SPI data order
#define RFM69_SPI_DATA_MODE				SPI_MODE0		//!< SPI mode

// Additional radio settings
#define RFM69_SYNCVALUE1		(0x2D)					//!< Make this compatible with sync1 byte of RFM12B lib

#if (MY_RFM69HW==true)
// RFM69H(C)W
#define RFM69_VERSION_HW	//!< HW version
#define RFM69_MIN_POWER_LEVEL_DBM		((rfm69_powerlevel_t)-2)	//!< min. power level, -2dBm
#if defined(MY_RFM69_MAX_POWER_LEVEL_DBM)
#define RFM69_MAX_POWER_LEVEL_DBM		MY_RFM69_MAX_POWER_LEVEL_DBM //!< MY_RFM69_MAX_POWER_LEVEL_DBM
#else
#define RFM69_MAX_POWER_LEVEL_DBM		((rfm69_powerlevel_t)20)	//!< max. power level, +20dBm
#endif
#else
// RFM69(C)W
#define RFM69_MIN_POWER_LEVEL_DBM		((rfm69_powerlevel_t)-18)	//!< min. power level, -18dBm
#if defined(MY_RFM69_MAX_POWER_LEVEL_DBM)
#define RFM69_MAX_POWER_LEVEL_DBM		MY_RFM69_MAX_POWER_LEVEL_DBM //!< MY_RFM69_MAX_POWER_LEVEL_DBM
#else
#define RFM69_MAX_POWER_LEVEL_DBM		((rfm69_powerlevel_t)13)	//!< max. power level, +13dBm
#endif
#endif

#define RFM69_FIFO_SIZE                  (0xFFu)		//!< Max number of bytes the Rx/Tx FIFO can hold
#define RFM69_MAX_PACKET_LEN             (0x40u)		//!< This is the maximum number of bytes that can be carried 
#define RFM69_ATC_TARGET_RANGE_DBM       (2u)				//!< ATC target range +/- dBm
#define RFM69_PACKET_HEADER_VERSION      (1u)				//!< RFM69 packet header version
#define RFM69_MIN_PACKET_HEADER_VERSION  (1u)				//!< Minimal RFM69 packet header version

#define RFM69_RETRIES                    (5u)				//!< Retries in case of failed transmission
#define RFM69_RETRY_TIMEOUT_MS           (200ul)		//!< Timeout for ACK, adjustments needed if modem configuration changed (air time different)
#define RFM69_MODE_READY_TIMEOUT_MS      (50ul)			//!< Timeout for mode ready

#define RFM69_ACK_REQUESTED              (7u)				//!< RFM69 header, controlFlag, bit 7
#define RFM69_ACK_RECEIVED               (6u)				//!< RFM69 header, controlFlag, bit 6
#define RFM69_ACK_RSSI_REPORT            (5u)				//!< RFM69 header, controlFlag, bit 5

#define RFM69_BROADCAST_ADDRESS          (255u)			//!< Broadcasting address 
#define RFM69_TARGET_RSSI_DBM            (-75)			//!< RSSI target
#define RFM69_HIGH_POWER_DBM             (18u)			//!< High power threshold, dBm

#if !defined(MY_RFM69_TX_TIMEOUT_MS)
#define MY_RFM69_TX_TIMEOUT_MS           (2*1000ul)	//!< Timeout for packet sent
#endif

// CSMA settings
#if !defined(MY_RFM69_CSMA_LIMIT_DBM)
#define MY_RFM69_CSMA_LIMIT_DBM             (-95)			//!< upper RX signal sensitivity threshold in dBm for carrier sense access
#endif
#if !defined(MY_RFM69_CSMA_TIMEOUT_MS)
#define MY_RFM69_CSMA_TIMEOUT_MS            (500ul)		//!< CSMA timeout
#endif
// powerup delay
#define RFM69_POWERUP_DELAY_MS           (100ul)		//!< Power up delay, allow VCC to settle, transport to become fully operational

// available frequency bands, non trivial values to avoid misconfiguration
#define RFM69_315MHZ                     (315000000ul)	//!< RFM69_315MHZ
#define RFM69_433MHZ                     (433920000ul)	//!< RFM69_433MHZ, center frequency 433.92 MHz
#define RFM69_865MHZ                     (865500000ul)  //!< RFM69_865MHZ, center frequency 865.5 MHz
#define RFM69_868MHZ                     (868000000ul)	//!< RFM69_868MHZ
#define RFM69_915MHZ                     (915000000ul)	//!< RFM69_915MHZ

#define RFM69_COURSE_TEMP_COEF           (-90)			//!< puts the temperature reading in the ballpark, user can fine tune the returned value
#define RFM69_FXOSC                      (32*1000000ul)	//!< OSC freq, 32MHz
#define RFM69_FSTEP                      (RFM69_FXOSC / 524288.0f)	//!< FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

// helper macros
#define RFM69_getACKRequested(__value) ((bool)bitRead(__value,RFM69_ACK_REQUESTED))						//!< getACKRequested
#define RFM69_setACKRequested(__value, __flag) bitWrite(__value,RFM69_ACK_REQUESTED,__flag)		//!< setACKRequested
#define RFM69_getACKReceived(__value) ((bool)bitRead(__value,RFM69_ACK_RECEIVED))							//!< getACKReceived
#define RFM69_setACKReceived(__value, __flag) bitWrite(__value,RFM69_ACK_RECEIVED,__flag)			//!< setACKReceived
#define RFM69_setACKRSSIReport(__value, __flag) bitWrite(__value,RFM69_ACK_RSSI_REPORT,__flag)//!< setACKRSSIReport
#define RFM69_getACKRSSIReport(__value) ((bool)bitRead(__value,RFM69_ACK_RSSI_REPORT))				//!< getACKRSSIReport

// Register access
#define RFM69_READ_REGISTER		(0x7Fu)			//!< reading register
#define RFM69_WRITE_REGISTER	(0x80u)			//!< writing register

// Modem configuration section
#define RFM69_CONFIG_FSK (RFM69_DATAMODUL_DATAMODE_PACKET | RFM69_DATAMODUL_MODULATIONTYPE_FSK | RFM69_DATAMODUL_MODULATIONSHAPING_00) //!< RFM69_CONFIG_FSK
#define RFM69_CONFIG_GFSK (RFM69_DATAMODUL_DATAMODE_PACKET | RFM69_DATAMODUL_MODULATIONTYPE_FSK | RFM69_DATAMODUL_MODULATIONSHAPING_10) //!< RFM69_CONFIG_GFSK
#define RFM69_CONFIG_OOK (RFM69_DATAMODUL_DATAMODE_PACKET | RFM69_DATAMODUL_MODULATIONTYPE_OOK | RFM69_DATAMODUL_MODULATIONSHAPING_00) //!< RFM69_CONFIG_OOK

#define RFM69_CONFIG_NOWHITE		(RFM69_PACKET1_FORMAT_VARIABLE | RFM69_PACKET1_DCFREE_OFF | RFM69_PACKET1_CRC_ON | RFM69_PACKET1_CRCAUTOCLEAR_ON | RFM69_PACKET1_ADRSFILTERING_NODEBROADCAST) //!< RFM69_CONFIG_NOWHITE
#define RFM69_CONFIG_WHITE			(RFM69_PACKET1_FORMAT_VARIABLE | RFM69_PACKET1_DCFREE_WHITENING | RFM69_PACKET1_CRC_ON | RFM69_PACKET1_CRCAUTOCLEAR_ON | RFM69_PACKET1_ADRSFILTERING_NODEBROADCAST) //!< RFM69_CONFIG_WHITE
#define RFM69_CONFIG_MANCHESTER	(RFM69_PACKET1_FORMAT_VARIABLE | RFM69_PACKET1_DCFREE_MANCHESTER | RFM69_PACKET1_CRC_ON | RFM69_PACKET1_CRCAUTOCLEAR_ON | RFM69_PACKET1_ADRSFILTERING_NODEBROADCAST) //!< RFM69_CONFIG_MANCHESTER

#define RFM69_RXBW_111_24_4 (RFM69_RXBW_DCCFREQ_111 | RFM69_RXBW_MANT_24 | RFM69_RXBW_EXP_4) //!< RFM69_RXBW_111_24_4
#define RFM69_RXBW_111_24_3 (RFM69_RXBW_DCCFREQ_111 | RFM69_RXBW_MANT_24 | RFM69_RXBW_EXP_3) //!< RFM69_RXBW_111_24_3
#define RFM69_RXBW_111_24_2 (RFM69_RXBW_DCCFREQ_111 | RFM69_RXBW_MANT_24 | RFM69_RXBW_EXP_2) //!< RFM69_RXBW_111_24_2
#define RFM69_RXBW_111_16_2 (RFM69_RXBW_DCCFREQ_111 | RFM69_RXBW_MANT_16 | RFM69_RXBW_EXP_2) //!< RFM69_RXBW_111_16_2
#define RFM69_RXBW_111_16_1 (RFM69_RXBW_DCCFREQ_111 | RFM69_RXBW_MANT_16 | RFM69_RXBW_EXP_1) //!< RFM69_RXBW_111_16_1
#define RFM69_RXBW_111_16_0 (RFM69_RXBW_DCCFREQ_111 | RFM69_RXBW_MANT_16 | RFM69_RXBW_EXP_0) //!< RFM69_RXBW_111_16_0
#define RFM69_RXBW_010_16_2 (RFM69_RXBW_DCCFREQ_010 | RFM69_RXBW_MANT_16 | RFM69_RXBW_EXP_2) //!< RFM69_RXBW_010_16_2

#define RFM69_FSK_BR2_FD5				RFM69_CONFIG_FSK, RFM69_BITRATEMSB_2000, RFM69_BITRATELSB_2000, RFM69_FDEVMSB_5000, RFM69_FDEVLSB_5000, RFM69_RXBW_111_24_4, RFM69_CONFIG_WHITE //!< RFM69_FSK_BR2_FD5
#define RFM69_FSK_BR2_4_FD4_8		RFM69_CONFIG_FSK, RFM69_BITRATEMSB_2400, RFM69_BITRATELSB_2400, RFM69_FDEVMSB_4800, RFM69_FDEVLSB_4800, RFM69_RXBW_111_24_4, RFM69_CONFIG_WHITE //!< RFM69_FSK_BR2_4_FD4_8
#define RFM69_FSK_BR4_8_FD9_6		RFM69_CONFIG_FSK, RFM69_BITRATEMSB_4800, RFM69_BITRATELSB_4800, RFM69_FDEVMSB_9600, RFM69_FDEVLSB_9600, RFM69_RXBW_111_24_4, RFM69_CONFIG_WHITE //!< RFM69_FSK_BR4_8_FD9_6
#define RFM69_FSK_BR9_6_FD19_2	RFM69_CONFIG_FSK, RFM69_BITRATEMSB_9600, RFM69_BITRATELSB_9600, RFM69_FDEVMSB_19200, RFM69_FDEVLSB_19200, RFM69_RXBW_111_24_4, RFM69_CONFIG_WHITE //!< RFM69_FSK_BR9_6_FD19_2
#define RFM69_FSK_BR19_2_FD38_4 RFM69_CONFIG_FSK, RFM69_BITRATEMSB_19200, RFM69_BITRATELSB_19200, RFM69_FDEVMSB_38400, RFM69_FDEVLSB_38400, RFM69_RXBW_111_24_3, RFM69_CONFIG_WHITE //!< RFM69_FSK_BR19_2_FD38_4
#define RFM69_FSK_BR38_4_FD76_8 RFM69_CONFIG_FSK, RFM69_BITRATEMSB_38400, RFM69_BITRATELSB_38400, RFM69_FDEVMSB_76800, RFM69_FDEVLSB_76800, RFM69_RXBW_111_24_2, RFM69_CONFIG_WHITE //!< RFM69_FSK_BR38_4_FD76_8
#define RFM69_FSK_BR55_5_FD50		RFM69_CONFIG_FSK, RFM69_BITRATEMSB_55555, RFM69_BITRATELSB_55555, RFM69_FDEVMSB_50000, RFM69_FDEVLSB_50000, RFM69_RXBW_111_16_2, RFM69_CONFIG_WHITE //!< RFM69_FSK_BR55_5_FD50
#define RFM69_FSK_BR57_6_FD120	RFM69_CONFIG_FSK, RFM69_BITRATEMSB_57600, RFM69_BITRATELSB_57600, RFM69_FDEVMSB_120000, RFM69_FDEVLSB_120000, RFM69_RXBW_111_16_1, RFM69_CONFIG_WHITE //!< RFM69_FSK_BR57_6_FD120
#define RFM69_FSK_BR125_FD125		RFM69_CONFIG_FSK, RFM69_BITRATEMSB_125000, RFM69_BITRATELSB_125000, RFM69_FDEVMSB_125000, RFM69_FDEVLSB_125000, RFM69_RXBW_010_16_2, RFM69_CONFIG_WHITE //!< RFM69_FSK_BR125_FD125
#define RFM69_FSK_BR250_FD250		RFM69_CONFIG_FSK, RFM69_BITRATEMSB_250000, RFM69_BITRATELSB_250000, RFM69_FDEVMSB_250000, RFM69_FDEVLSB_250000, RFM69_RXBW_111_16_0, RFM69_CONFIG_WHITE //!< RFM69_FSK_BR250_FD250

#define RFM69_GFSK_BR2_FD5				RFM69_CONFIG_GFSK, RFM69_BITRATEMSB_2000, RFM69_BITRATELSB_2000, RFM69_FDEVMSB_5000, RFM69_FDEVLSB_5000, RFM69_RXBW_111_24_4, RFM69_CONFIG_WHITE //!< RFM69_GFSK_BR2_FD5
#define RFM69_GFSK_BR2_4_FD4_8		RFM69_CONFIG_GFSK, RFM69_BITRATEMSB_2400, RFM69_BITRATELSB_2400, RFM69_FDEVMSB_4800, RFM69_FDEVLSB_4800, RFM69_RXBW_111_24_4, RFM69_CONFIG_WHITE //!< RFM69_GFSK_BR2_4_FD4_8
#define RFM69_GFSK_BR4_8_FD9_6		RFM69_CONFIG_GFSK, RFM69_BITRATEMSB_4800, RFM69_BITRATELSB_4800, RFM69_FDEVMSB_9600, RFM69_FDEVLSB_9600, RFM69_RXBW_111_24_4, RFM69_CONFIG_WHITE //!< RFM69_GFSK_BR4_8_FD9_6
#define RFM69_GFSK_BR9_6_FD19_2		RFM69_CONFIG_GFSK, RFM69_BITRATEMSB_9600, RFM69_BITRATELSB_9600, RFM69_FDEVMSB_19200, RFM69_FDEVLSB_19200, RFM69_RXBW_111_24_4, RFM69_CONFIG_WHITE //!< RFM69_GFSK_BR9_6_FD19_2
#define RFM69_GFSK_BR19_2_FD38_4	RFM69_CONFIG_GFSK, RFM69_BITRATEMSB_19200, RFM69_BITRATELSB_19200, RFM69_FDEVMSB_38400, RFM69_FDEVLSB_38400, RFM69_RXBW_111_24_3, RFM69_CONFIG_WHITE //!< RFM69_GFSK_BR19_2_FD38_4
#define RFM69_GFSK_BR38_4_FD76_8	RFM69_CONFIG_GFSK, RFM69_BITRATEMSB_38400, RFM69_BITRATELSB_38400, RFM69_FDEVMSB_76800, RFM69_FDEVLSB_76800, RFM69_RXBW_111_24_2, RFM69_CONFIG_WHITE //!< RFM69_GFSK_BR38_4_FD76_8
#define RFM69_GFSK_BR55_5_FD50		RFM69_CONFIG_GFSK, RFM69_BITRATEMSB_55555, RFM69_BITRATELSB_55555, RFM69_FDEVMSB_50000, RFM69_FDEVLSB_50000, RFM69_RXBW_111_16_2, RFM69_CONFIG_WHITE //!< RFM69_GFSK_BR55_5_FD50
#define RFM69_GFSK_BR57_6_FD120		RFM69_CONFIG_GFSK, RFM69_BITRATEMSB_57600, RFM69_BITRATELSB_57600, RFM69_FDEVMSB_120000, RFM69_FDEVLSB_120000, RFM69_RXBW_111_16_1, RFM69_CONFIG_WHITE //!< RFM69_GFSK_BR57_6_FD120
#define RFM69_GFSK_BR125_FD125		RFM69_CONFIG_GFSK, RFM69_BITRATEMSB_125000, RFM69_BITRATELSB_125000, RFM69_FDEVMSB_125000, RFM69_FDEVLSB_125000, RFM69_RXBW_010_16_2, RFM69_CONFIG_WHITE //!< RFM69_GFSK_BR125_FD125
#define RFM69_GFSK_BR250_FD250		RFM69_CONFIG_GFSK, RFM69_BITRATEMSB_250000, RFM69_BITRATELSB_250000, RFM69_FDEVMSB_250000, RFM69_FDEVLSB_250000, RFM69_RXBW_111_16_0, RFM69_CONFIG_WHITE //!< RFM69_GFSK_BR250_FD250

#define RFM69_OOK_BR2_FD5				RFM69_CONFIG_OOK, RFM69_BITRATEMSB_2000, RFM69_BITRATELSB_2000, RFM69_FDEVMSB_5000, RFM69_FDEVLSB_5000, RFM69_RXBW_111_24_4, RFM69_CONFIG_WHITE //!< RFM69_OOK_BR2_FD5
#define RFM69_OOK_BR2_4_FD4_8		RFM69_CONFIG_OOK, RFM69_BITRATEMSB_2400, RFM69_BITRATELSB_2400, RFM69_FDEVMSB_4800, RFM69_FDEVLSB_4800, RFM69_RXBW_111_24_4, RFM69_CONFIG_WHITE //!< RFM69_OOK_BR2_4_FD4_8
#define RFM69_OOK_BR4_8_FD9_6		RFM69_CONFIG_OOK, RFM69_BITRATEMSB_4800, RFM69_BITRATELSB_4800, RFM69_FDEVMSB_9600, RFM69_FDEVLSB_9600, RFM69_RXBW_111_24_4, RFM69_CONFIG_WHITE //!< RFM69_OOK_BR4_8_FD9_6
#define RFM69_OOK_BR9_6_FD19_2	RFM69_CONFIG_OOK, RFM69_BITRATEMSB_9600, RFM69_BITRATELSB_9600, RFM69_FDEVMSB_19200, RFM69_FDEVLSB_19200, RFM69_RXBW_111_24_4, RFM69_CONFIG_WHITE //!< RFM69_OOK_BR9_6_FD19_2
#define RFM69_OOK_BR19_2_FD38_4 RFM69_CONFIG_OOK, RFM69_BITRATEMSB_19200, RFM69_BITRATELSB_19200, RFM69_FDEVMSB_38400, RFM69_FDEVLSB_38400, RFM69_RXBW_111_24_3, RFM69_CONFIG_WHITE //!< RFM69_OOK_BR19_2_FD38_4
#define RFM69_OOK_BR38_4_FD76_8 RFM69_CONFIG_OOK, RFM69_BITRATEMSB_38400, RFM69_BITRATELSB_38400, RFM69_FDEVMSB_76800, RFM69_FDEVLSB_76800, RFM69_RXBW_111_24_2, RFM69_CONFIG_WHITE //!< RFM69_OOK_BR38_4_FD76_8
#define RFM69_OOK_BR55_5_FD50		RFM69_CONFIG_OOK, RFM69_BITRATEMSB_55555, RFM69_BITRATELSB_55555, RFM69_FDEVMSB_50000, RFM69_FDEVLSB_50000, RFM69_RXBW_111_16_2, RFM69_CONFIG_WHITE //!< RFM69_OOK_BR55_5_FD50
#define RFM69_OOK_BR57_6_FD120	RFM69_CONFIG_OOK, RFM69_BITRATEMSB_57600, RFM69_BITRATELSB_57600, RFM69_FDEVMSB_120000, RFM69_FDEVLSB_120000, RFM69_RXBW_111_16_1, RFM69_CONFIG_WHITE //!< RFM69_OOK_BR57_6_FD120
#define RFM69_OOK_BR125_FD125		RFM69_CONFIG_OOK, RFM69_BITRATEMSB_125000, RFM69_BITRATELSB_125000, RFM69_FDEVMSB_125000, RFM69_FDEVLSB_125000, RFM69_RXBW_010_16_2, RFM69_CONFIG_WHITE //!< RFM69_OOK_BR125_FD125
#define RFM69_OOK_BR250_FD250		RFM69_CONFIG_OOK, RFM69_BITRATEMSB_250000, RFM69_BITRATELSB_250000, RFM69_FDEVMSB_250000, RFM69_FDEVLSB_250000, RFM69_RXBW_111_16_0, RFM69_CONFIG_WHITE //!< RFM69_OOK_BR250_FD250

#if !defined(MY_RFM69_MODEM_CONFIGURATION)
#define MY_RFM69_MODEM_CONFIGURATION RFM69_FSK_BR55_5_FD50  //!< default setting, RFM69_FSK_BR55_5_FD50
#endif

/**
* @brief Radio modes
*/
typedef enum {
	RFM69_RADIO_MODE_RX = 0,						//!< RX mode
	RFM69_RADIO_MODE_TX = 1,						//!< TX mode
	RFM69_RADIO_MODE_CAD = 2,						//!< CAD mode
	RFM69_RADIO_MODE_SLEEP = 3,					//!< SLEEP mode
	RFM69_RADIO_MODE_STDBY = 4,					//!< STDBY mode
	RFM69_RADIO_MODE_SYNTH = 5,					//!< SYNTH mode
	RFM69_RADIO_MODE_LISTEN = 6					//!< LISTEN mode
} rfm69_radio_mode_t;

/**
* @brief Sequence number data type
*/
typedef uint8_t rfm69_sequenceNumber_t;
/**
* @brief RSSI data type
*/
typedef uint8_t rfm69_RSSI_t;
/**
* @brief SNR data type
*/
typedef int8_t rfm69_SNR_t;
/**
* @brief Control flag data type
*/
typedef uint8_t rfm69_controlFlags_t;
/**
* @brief Power level in dBm
*/
typedef int8_t rfm69_powerlevel_t;

/**
* @brief RFM69 header
* IMPORTANT: Do not change order (see datasheet for packet structure)
*/
typedef struct {
	uint8_t packetLen;                      //!< packet length
	uint8_t recipient;                      //!< payload recipient
	uint8_t version;                        //!< header version (20180128tk: >=3.0.0 fused with controlFlags)
	uint8_t sender;                         //!< payload sender
	rfm69_controlFlags_t controlFlags;      //!< control flags, used for ACK
	rfm69_sequenceNumber_t sequenceNumber;  //!< packet sequence number, used for ACK
} __attribute__((packed)) rfm69_header_t;

/**
* @brief RFM69  ACK packet structure
*/
typedef struct {
	rfm69_sequenceNumber_t sequenceNumber;  //!< sequence number
	rfm69_RSSI_t RSSI;                      //!< RSSI
} __attribute__((packed)) rfm69_ack_t;

#define RFM69_HEADER_LEN sizeof(rfm69_header_t)		//!< Size header inside  payload
#define RFM69_MAX_PAYLOAD_LEN (RFM69_MAX_PACKET_LEN - RFM69_HEADER_LEN)	//!< Max payload length

/**
* @brief Packet structure
* IMPORTANT: Do not change order
*/
typedef struct {
	union {
		struct {
			rfm69_header_t header;						//!< Packet header
			union {
				uint8_t payload[RFM69_MAX_PAYLOAD_LEN];	//!< Union: Data Payload, i.e. MySensors message
				rfm69_ack_t ACK;						//!< Union: ACK payload (internal)
			};
		};
		uint8_t data[RFM69_MAX_PACKET_LEN];				//!< RAW data access
	};
	uint8_t payloadLen;									//!< Length of payload (excluding header)
	rfm69_RSSI_t RSSI;									//!< RSSI of current packet, RSSI = value - 137
} __attribute__((packed)) rfm69_packet_t;

/**
* @brief RFM69 internal variables
*/
typedef struct {
	uint8_t address;                           //!< Node address
	rfm69_packet_t currentPacket;              //!< Buffer for current packet
	rfm69_sequenceNumber_t txSequenceNumber;   //!< RFM69_txSequenceNumber
	rfm69_powerlevel_t powerLevel;             //!< TX power level dBm
	uint8_t ATCtargetRSSI;                     //!< ATC: target RSSI
	// 8 bit
	rfm69_radio_mode_t radioMode : 3;          //!< current transceiver state
	bool dataReceived : 1;                     //!< data received
	bool ackReceived : 1;                      //!< ACK received
	bool ATCenabled : 1;                       //!< ATC enabled
	uint8_t reserved : 2;                      //!< Reserved
} rfm69_internal_t;

#define LOCAL static		//!< static

/**
* @brief RFM69_handler
*/
LOCAL void RFM69_handler(void);

/**
* @brief Clear flags and FIFO
*/
LOCAL void RFM69_clearFIFO(void);

/**
* @brief Check for channel activity
* @return True if channel activity under RFM69_CSMA_LIMIT_DBM
*/
LOCAL bool RFM69_channelFree(void);

/**
* @brief RFM69_interruptHandling
*/
LOCAL void RFM69_interruptHandling(void);

/**
* @brief Initialise the driver transport hardware and software
* @param frequencyHz Frequency in Hz
* @return True if initialisation succeeded
*/
LOCAL bool RFM69_initialise(const uint32_t frequencyHz);

/**
* @brief Set the driver/node address
* @param addr
*/
LOCAL void RFM69_setAddress(const uint8_t addr);

/**
* @brief Get driver/node address
* @return Node address
*/
LOCAL uint8_t RFM69_getAddress(void);

/**
* @brief Tests whether a new message is available
* @return True if a new, complete, error-free uncollected message is available to be retreived by @ref RFM69_receive()
*/
LOCAL bool RFM69_available(void);

/**
* @brief If a valid message is received, copy it to buf and return length. 0 byte messages are permitted.
* @param buf Location to copy the received message
* @param maxBufSize Max buffer size
* @return Number of bytes
*/
LOCAL uint8_t RFM69_receive(uint8_t *buf, const uint8_t maxBufSize);

/**
* @brief RFM69_sendFrame
* @param packet
* @param increaseSequenceCounter
* @return True if packet sent
*/
LOCAL bool RFM69_sendFrame(rfm69_packet_t *packet, const bool increaseSequenceCounter = true);

/**
* @brief RFM69_send
* @param recipient
* @param data
* @param len
* @param flags
* @param increaseSequenceCounter
* @return True if frame sent
*/
LOCAL bool RFM69_send(const uint8_t recipient, uint8_t *data, const uint8_t len,
                      const rfm69_controlFlags_t flags, const bool increaseSequenceCounter = true);

/**
* @brief Sets the transmitter and receiver center frequency
* @param frequencyHz Frequency in Hz
*/
LOCAL void RFM69_setFrequency(const uint32_t frequencyHz);

/**
* @brief Sets the transmitter power output level, and configures the transmitter pin
* @param newPowerLevel Transmitter power level in dBm (-18 to +20dBm)
* @return True power level adjusted
*/
LOCAL bool RFM69_setTxPowerLevel(rfm69_powerlevel_t newPowerLevel);

/**
* @brief Reports the transmitter power output level in dBm
* @return power level
*/
LOCAL rfm69_powerlevel_t RFM69_getTxPowerLevel(void);

/**
* @brief Reports the transmitter power output level in percents
* @return power level
*/
LOCAL uint8_t RFM69_getTxPowerPercent(void);

/**
* @brief Sets the radio into low-power sleep mode
* @return true if sleep mode was successfully entered
*/
LOCAL bool RFM69_sleep(void);

/**
* @brief Sets the radio to standby mode
* @return true if standby mode was successfully entered
*/
LOCAL bool RFM69_standBy(void);

/**
* @brief Power down radio (HW)
*/
LOCAL void RFM69_powerDown(void);

/**
* @brief Power up radio (HW)
*/
LOCAL void RFM69_powerUp(void);

/**
* @brief RFM69_sendACK
* @param recipient
* @param sequenceNumber
* @param RSSI (rfm95_RSSI_t)
*/
LOCAL void RFM69_sendACK(const uint8_t recipient, const rfm69_sequenceNumber_t sequenceNumber,
                         const rfm69_RSSI_t RSSI);

/**
* @brief RFM69_sendWithRetry
* @param recipient
* @param buffer
* @param bufferSize
* @param noACK
* @return True if packet successfully sent
*/
LOCAL bool RFM69_sendWithRetry(const uint8_t recipient, const void *buffer,
                               const uint8_t bufferSize,
                               const bool noACK);

/**
* @brief RFM69_setRadioMode
* @param newRadioMode
* @return True if mode changed
*/
LOCAL bool RFM69_setRadioMode(const rfm69_radio_mode_t newRadioMode);

/**
* @brief Low level interrupt handler
*/
LOCAL void RFM69_interruptHandler(void);

/**
* @brief RFM69_getSendingRSSI
* @return RSSI Own RSSI as measured at the receiving node of last sent packet (if ACK & ATC enabled)
*/
LOCAL int16_t RFM69_getSendingRSSI(void);

/**
* @brief RFM69_getReceivingRSSI
* @return RSSI Signal strength of last received packet
*/
LOCAL int16_t RFM69_getReceivingRSSI(void);

/**
* @brief RFM69_executeATC
* @param currentRSSI
* @param targetRSSI
* @return True if power level adjusted
*/
LOCAL bool RFM69_executeATC(const rfm69_RSSI_t currentRSSI, const rfm69_RSSI_t targetRSSI);

// TEMP ADDED
/**
* @brief RFM69_setConfiguration Set general radio register configuration TODO temp use setmodemregisters
*/
LOCAL void RFM69_setConfiguration(void);

/**
* @brief RFM69_isModeReady
* @return True if Mode Ready is ok, false is timeout
*/
LOCAL bool RFM69_isModeReady(void);

/**
* @brief RFM69_sanityCheck detect HW defect, configuration errors or interrupted SPI line
* @return True if radio sanity check passed
*/
LOCAL bool RFM69_sanityCheck(void);

/**
* @brief RFM69_encrypt Set encryption mode
* @param key if key is null, encryption is disabled. Key has to be 16 bytes!
*/
LOCAL void RFM69_encrypt(const char *key);

/**
* @brief RFM69_setHighPowerRegs
* @param onOff
*/
LOCAL void RFM69_setHighPowerRegs(const bool onOff);

/**
* @brief RFM69_readRSSI
* @param forceTrigger
* @return RSSI (internal format)
*/
LOCAL rfm69_RSSI_t RFM69_readRSSI(const bool forceTrigger = false);

/**
* @brief RFM69_ATCmode
* @param targetRSSI Target RSSI for transmitter (default -60)
* @param onOff True to enable ATC
*/
LOCAL void RFM69_ATCmode(const bool onOff, const int16_t targetRSSI = RFM69_TARGET_RSSI_DBM);

/**
* @brief RFM69_readAllRegs
* Read and display all RFM69 register contents.
* @note define RFM69_REGISTER_DETAIL for register content decoding.
*/
LOCAL void RFM69_readAllRegs(void);

#endif

/** @}*/
