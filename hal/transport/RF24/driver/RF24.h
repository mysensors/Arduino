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
* Based on maniacbug's RF24 library, copyright (C) 2011 J. Coliz <maniacbug@ymail.com>
* RF24 driver refactored and optimized for speed and size, copyright (C) 2017 Olivier Mauti <olivier@mysensors.org>
*
* Definitions for Nordic nRF24L01+ radios:
* https://www.nordicsemi.com/eng/Products/2.4GHz-RF/nRF24L01P
*
*/

/**
* @file RF24.h
*
* @defgroup RF24grp RF24
* @ingroup internals
* @{
*
* RF24 driver-related log messages, format: [!]SYSTEM:[SUB SYSTEM:]MESSAGE
* - [!] Exclamation mark is prepended in case of error
*
* |E| SYS  | SUB  | Message              | Comment
* |-|------|------|----------------------|---------------------------------------------------------------------
* | | RF24 | INIT | PIN,CE=%%d,CS=%%d    | Initialise RF24 radio, pin configuration: chip enable (CE), chip select (CS)
* |!| RF24 | INIT | SANCHK FAIL          | Sanity check failed, check wiring or replace module
* | | RF24 | SPP  | PCT=%%d,TX LEVEL=%%d | Set TX level, input TX percent (PCT)
* | | RF24 | RBR  | REG=%%d,VAL=%%d      | Read register (REG), value=(VAL)
* | | RF24 | WBR  | REG=%%d,VAL=%%d      | Write register (REG), value=(VAL)
* | | RF24 | FRX  |                      | Flush RX buffer
* | | RF24 | FTX  |                      | Flush TX buffer
* | | RF24 | OWP  | RCPT=%%d             | Open writing pipe, recipient=(RCPT)
* | | RF24 | STL  |                      | Start listening
* | | RF24 | SPL  |                      | Stop listening
* | | RF24 | SLP  |                      | Set radio to sleep
* | | RF24 | SBY  |                      | Set radio to standby
* | | RF24 | TXM  | TO=%%d,LEN=%%d       | Transmit message to=(TO), length=(LEN)
* |!| RF24 | TXM  | MAX_RT               | Max TX retries, no ACK received
* |!| RF24 | GDP  | PYL INV              | Invalid payload size
* | | RF24 | RXM  | LEN=%%d              | Read message, length=(LEN)
* | | RF24 | STX  | LEVEL=%%d            | Set TX level, level=(LEVEL)
*
*/

#ifndef __RF24_H__
#define __RF24_H__

#include "RF24registers.h"

#if !defined(RF24_SPI)
#define RF24_SPI hwSPI //!< default SPI
#endif

#if defined(ARDUINO_ARCH_AVR)
#define DEFAULT_RF24_CE_PIN				(9)		//!< DEFAULT_RF24_CE_PIN
#elif defined(ARDUINO_ARCH_ESP8266)
#define DEFAULT_RF24_CE_PIN				(4)		//!< DEFAULT_RF24_CE_PIN
#elif defined(ARDUINO_ARCH_ESP32)
#define DEFAULT_RF24_CE_PIN				(17)	//!< DEFAULT_RF24_CE_PIN
#elif defined(ARDUINO_ARCH_SAMD)
#define DEFAULT_RF24_CE_PIN				(27)	//!< DEFAULT_RF24_CE_PIN
#elif defined(LINUX_ARCH_RASPBERRYPI)
#define DEFAULT_RF24_CE_PIN				(22)	//!< DEFAULT_RF24_CE_PIN
//#define DEFAULT_RF24_CS_PIN			(24)	//!< DEFAULT_RF24_CS_PIN
#elif defined(ARDUINO_ARCH_STM32F1)
#define DEFAULT_RF24_CE_PIN				(PB0)	//!< DEFAULT_RF24_CE_PIN
#elif defined(TEENSYDUINO)
#define DEFAULT_RF24_CE_PIN				(9)		//!< DEFAULT_RF24_CE_PIN
#else
#define DEFAULT_RF24_CE_PIN				(9)		//!< DEFAULT_RF24_CE_PIN
#endif

#define DEFAULT_RF24_CS_PIN				(SS)	//!< DEFAULT_RF24_CS_PIN


#define LOCAL static		//!< static

// SPI settings
#define RF24_SPI_DATA_ORDER				MSBFIRST	//!< RF24_SPI_DATA_ORDER
#define RF24_SPI_DATA_MODE				SPI_MODE0	//!< RF24_SPI_DATA_MODE

#define RF24_BROADCAST_ADDRESS	(255u)	//!< RF24_BROADCAST_ADDRESS

// verify RF24 IRQ defs
#if defined(MY_RX_MESSAGE_BUFFER_FEATURE)
#if !defined(MY_RF24_IRQ_PIN)
#error Message buffering feature requires MY_RF24_IRQ_PIN to be defined!
#endif
// SoftSPI does not support usingInterrupt()
#ifdef MY_SOFTSPI
#error RF24 IRQ usage cannot be used with Soft SPI
#endif
// ESP8266 does not support usingInterrupt()
#ifdef ARDUINO_ARCH_ESP8266
#error RF24 IRQ usage cannot be used with ESP8266
#endif
#ifndef SPI_HAS_TRANSACTION
#error RF24 IRQ usage requires transactional SPI support
#endif
#else
#ifdef MY_RX_MESSAGE_BUFFER_SIZE
#error Receive message buffering requires RF24 IRQ usage
#endif
#endif


// RF24 settings
#if defined(MY_RX_MESSAGE_BUFFER_FEATURE)
#define RF24_CONFIGURATION (uint8_t) ((RF24_CRC_16 << 2) | (1 << RF24_MASK_TX_DS) | (1 << RF24_MASK_MAX_RT))		//!< MY_RF24_CONFIGURATION
#else
#define RF24_CONFIGURATION (uint8_t) (RF24_CRC_16 << 2)		//!< RF24_CONFIGURATION
#endif
#define RF24_FEATURE (uint8_t)( _BV(RF24_EN_DPL))	//!<  RF24_FEATURE
#define RF24_RF_SETUP (uint8_t)(( ((MY_RF24_DATARATE & 0b10 ) << 4) | ((MY_RF24_DATARATE & 0b01 ) << 3) | (MY_RF24_PA_LEVEL << 1) ) + 1) 		//!< RF24_RF_SETUP, +1 for Si24R1 and LNA

// powerup delay
#define RF24_POWERUP_DELAY_MS	(100u)		//!< Power up delay, allow VCC to settle, transport to become fully operational

// pipes
#define RF24_BROADCAST_PIPE		(1u)		//!< RF24_BROADCAST_PIPE
#define RF24_NODE_PIPE			(0u)		//!< RF24_NODE_PIPE

// functions
/**
* @brief RF24_csn
* @param level
*/
LOCAL void RF24_csn(const bool level);
/**
* @brief RF24_ce
* @param level
*/
LOCAL void RF24_ce(const bool level);
/**
* @brief RF24_spiMultiByteTransfer
* @param cmd
* @param buf
* @param len
* @param readMode
* @return
*/
LOCAL uint8_t RF24_spiMultiByteTransfer(const uint8_t cmd, uint8_t *buf, const uint8_t len,
                                        const bool readMode);
/**
* @brief RF24_spiByteTransfer
* @param cmd
* @return
*/
LOCAL uint8_t RF24_spiByteTransfer(const uint8_t cmd);
/**
* @brief RF24_RAW_readByteRegister
* @param cmd
* @return
*/
LOCAL uint8_t RF24_RAW_readByteRegister(const uint8_t cmd);
/**
* @brief RF24_RAW_writeByteRegister
* @param cmd
* @param value
* @return
*/
LOCAL uint8_t RF24_RAW_writeByteRegister(const uint8_t cmd, const uint8_t value);

// helper macros
#define RF24_readByteRegister(__reg) RF24_RAW_readByteRegister(RF24_CMD_READ_REGISTER | (RF24_REGISTER_MASK & (__reg)))		//!< RF24_readByteRegister
#define RF24_readMultiByteRegister(__reg,__buf,__len) RF24_spiMultiByteTransfer(RF24_CMD_READ_REGISTER | (RF24_REGISTER_MASK & (__reg)), (uint8_t *)__buf, __len, true)	//!< RF24_readMultiByteRegister
#define RF24_writeByteRegister(__reg,__value) RF24_RAW_writeByteRegister(RF24_CMD_WRITE_REGISTER | (RF24_REGISTER_MASK & (__reg)), __value)	//!< RF24_writeByteRegister
#define RF24_writeMultiByteRegister(__reg,__buf,__len) RF24_spiMultiByteTransfer(RF24_CMD_WRITE_REGISTER | (RF24_REGISTER_MASK & (__reg)), (uint8_t *)__buf, __len, false)	//!< RF24_writeMultiByteRegister

/**
* @brief RF24_flushRX
*/
LOCAL void RF24_flushRX(void);
/**
* @brief RF24_flushTX
*/
LOCAL void RF24_flushTX(void);
/**
* @brief RF24_getStatus
* @return
*/
LOCAL uint8_t RF24_getStatus(void);
/**
* @brief RF24_getFIFOStatus
* @return
*/
LOCAL uint8_t RF24_getFIFOStatus(void) __attribute__((unused));
/**
* @brief RF24_openWritingPipe
* @param recipient
*/
LOCAL void RF24_openWritingPipe(const uint8_t recipient);
/**
* @brief RF24_startListening
*/
LOCAL void RF24_startListening(void);
/**
* @brief RF24_stopListening
*/
LOCAL void RF24_stopListening(void);
/**
* @brief RF24_sleep
*/
LOCAL void RF24_sleep(void);
/**
* @brief RF24_standBy
*/
LOCAL void RF24_standBy(void);
/**
* @brief RF24_powerDown
*/
LOCAL void RF24_powerDown(void);
/**
* @brief RF24_powerUp
*/
LOCAL void RF24_powerUp(void);
/**
* @brief RF24_sendMessage
* @param recipient
* @param buf
* @param len
* @param noACK set True if no ACK is required
* @return
*/
LOCAL bool RF24_sendMessage(const uint8_t recipient, const void *buf, const uint8_t len,
                            const bool noACK = false);
/**
* @brief RF24_getDynamicPayloadSize
* @return
*/
LOCAL uint8_t RF24_getDynamicPayloadSize(void);
/**
* @brief RF24_isDataAvailable
* @return
*/
LOCAL bool RF24_isDataAvailable(void);
/**
* @brief RF24_readMessage
* @return
*/
LOCAL uint8_t RF24_readMessage(void *buf);
/**
* @brief RF24_setNodeAddress
* @param address
*/
LOCAL void RF24_setNodeAddress(const uint8_t address);
/**
* @brief RF24_getNodeID
* @return
*/
LOCAL uint8_t RF24_getNodeID(void);
/**
* @brief RF24_sanityCheck
* @return
*/
LOCAL bool RF24_sanityCheck(void);
/**
* @brief RF24_initialize
* @return
*/
LOCAL bool RF24_initialize(void);
/**
* @brief RF24_setChannel
* @param channel
*/
LOCAL void RF24_setChannel(const uint8_t channel);
/**
* @brief RF24_getChannel
* @return channel
*/
LOCAL uint8_t RF24_getChannel(void) __attribute__((unused));
/**
* @brief RF24_setRetries
* @param retransmitDelay
* @param retransmitCount
*/
LOCAL void RF24_setRetries(const uint8_t retransmitDelay, const uint8_t retransmitCount);
/**
* @brief RF24_setAddressWidth
* @param addressWidth
*/
LOCAL void RF24_setAddressWidth(const uint8_t addressWidth);
/**
* @brief RF24_setRFSetup
* @param RFsetup
*/
LOCAL void RF24_setRFSetup(const uint8_t RFsetup);
/**
* @brief RF24_setFeature
* @param feature
*/
LOCAL void RF24_setFeature(const uint8_t feature);
/**
* @brief RF24_getFeature
* @return
*/
LOCAL uint8_t RF24_getFeature(void);
/**
* @brief RF24_setPipe
* @param pipe
*/
LOCAL void RF24_setPipe(const uint8_t pipe);
/**
* @brief RF24_setAutoACK
* @param pipe
*/
LOCAL void RF24_setAutoACK(const uint8_t pipe);
/**
* @brief RF24_setDynamicPayload
* @param pipe
*/
LOCAL void RF24_setDynamicPayload(const uint8_t pipe);
/**
* @brief RF24_getRFConfiguration
* @return RF configuration
*/
LOCAL uint8_t RF24_getRFConfiguration(void) __attribute__((unused));
/**
* @brief RF24_setRFConfiguration
* @param configuration
*/
LOCAL void RF24_setRFConfiguration(const uint8_t configuration);
/**
* @brief RF24_setPipeAddress
* @param pipe
* @param address
* @param addressWidth
*/
LOCAL void RF24_setPipeAddress(const uint8_t pipe, uint8_t *address, const uint8_t addressWidth);
/**
* @brief RF24_setPipeLSB
* @param pipe
* @param LSB
*/
LOCAL void RF24_setPipeLSB(const uint8_t pipe, const uint8_t LSB);
/**
* @brief RF24_getObserveTX
* @return
*/
LOCAL uint8_t RF24_getObserveTX(void);
/**
* @brief RF24_setStatus
* @param status
* @return status byte before setting new status
*/
LOCAL uint8_t RF24_setStatus(const uint8_t status);
/**
* @brief RF24_enableFeatures
*/
LOCAL void RF24_enableFeatures(void);
/**
* @brief RF24_getTxPowerPercent
* @return
*/
LOCAL uint8_t RF24_getTxPowerPercent(void);
/**
* @brief RF24_getRawTxPowerLevel
* @return power level
*/
LOCAL uint8_t RF24_getRawTxPowerLevel(void);
/**
* @brief RF24_getTxPowerLevel
* @return power level in pseudo-dBm
*/
LOCAL int16_t RF24_getTxPowerLevel(void);
/**
* @brief RF24_setTxPowerPercent
* @param newPowerPercent
* @return
*/
LOCAL bool RF24_setTxPowerPercent(const uint8_t newPowerPercent);
/**
* @brief RF24_getSendingRSSI
* @return Pseudo-RSSI based on ARC register
*/
LOCAL int16_t RF24_getSendingRSSI(void);
/**
* @brief Generate a constant carrier wave at active channel & transmit power (for testing only).
*/
LOCAL void RF24_enableConstantCarrierWave(void) __attribute__((unused));
/**
* @brief Stop generating a constant carrier wave (for testing only).
*/
LOCAL void RF24_disableConstantCarrierWave(void) __attribute__((unused));
/**
* @brief Retrieve latched RPD power level, in receive mode (for testing, nRF24L01+ only).
* @return True when power level >-64dBm for more than 40us.
*/
LOCAL bool RF24_getReceivedPowerDetector(void) __attribute__((unused));

#if defined(MY_RX_MESSAGE_BUFFER_FEATURE)
/**
* @brief Callback type
*/
typedef void (*RF24_receiveCallbackType)(void);
/**
* @brief RF24_registerReceiveCallback
* Register a callback, which will be called (from interrupt context) for every message received.
* @note When a callback is registered, it _must_ retrieve the message from the nRF24
* by calling RF24_readMessage(). Otherwise the interrupt will not get deasserted
* and message reception will stop.
* @param cb
*/
LOCAL void RF24_registerReceiveCallback(RF24_receiveCallbackType cb);
#endif

#endif // __RF24_H__

/** @}*/
