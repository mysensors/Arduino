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
 * Based on Mike McCauley's RFM95 library, Copyright (C) 2014 Mike McCauley <mikem@airspayce.com>
 * Radiohead http://www.airspayce.com/mikem/arduino/RadioHead/index.html
 *
 * RFM95 driver refactored and optimized for MySensors, Copyright (C) 2017-2018 Olivier Mauti <olivier@mysensors.org>
 *
 * Changelog:
 * - ACK with sequenceNumber
 * - ATC control
 *
 * Definitions for HopeRF LoRa radios:
 * http://www.hoperf.com/upload/rf/RFM95_96_97_98W.pdf
 *
 */

/**
* @file RFM95.h
*
* @defgroup RFM95grp RFM95
* @ingroup internals
* @{
*
* RFM95 driver-related log messages, format: [!]SYSTEM:[SUB SYSTEM:]MESSAGE
* - [!] Exclamation mark is prepended in case of error
*
* |E| SYS   | SUB  | Message                                | Comment
* |-|-------|------|----------------------------------------|-----------------------------------------------------------------------------------
* | | RFM95 | INIT |                                        | Initialise RFM95 radio
* | | RFM95 | INIT | PIN,CS=%%d,IQP=%%d,IQN=%%d[,RST=%%d]   | Pin configuration: chip select (CS), IRQ pin (IQP), IRQ number (IQN), Reset (RST)
* |!| RFM95 | INIT | SANCHK FAIL                            | Sanity check failed, check wiring or replace module
* |!| RFM95 | IRH  | CRC FAIL                               | Incoming packet has CRC error, skip
* | | RFM95 | RCV  | SEND ACK                               | ACK request received, sending ACK back
* | | RFM95 | PTC  | LEVEL=%%d                              | Set TX power level
* | | RFM95 | SAC  | SEND ACK,TO=%%d,RSSI=%%d,SNR=%%d       | Send ACK to node (TO), RSSI of received message (RSSI), SNR of message (SNR)
* | | RFM95 | ATC  | ADJ TXL,cR=%%d,tR=%%d..%%d,TXL=%%d     | Adjust TX level, current RSSI (cR), target RSSI range (tR), TX level (TXL)
* | | RFM95 | SWR  | SEND,TO=%%d,RETRY=%%d                  | Send message to (TO), NACK retry counter (RETRY)
* | | RFM95 | SWR  | ACK FROM=%%d,SEQ=%%d,RSSI=%%d,SNR=%%d  | ACK received from node (FROM), seq ID (SEQ), (RSSI), (SNR)
* |!| RFM95 | SWR  | NACK                                   | No ACK received
* | | RFM95 | SPP  | PCT=%%d,TX LEVEL=%%d                   | Set TX level percent (PCT), TX level (LEVEL)
* | | RFM95 | PWD  |                                        | Power down radio
* | | RFM95 | PWU  |                                        | Power up radio
*
* RFM95 modem configuration
*
* BW = Bandwidth in kHz
* CR = Error correction code
* SF = Spreading factor, chips / symbol
*
* | CONFIG           | BW    | CR  | SF   | Comment               | air-time (15 bytes)
* |------------------|-------|-----|------|-----------------------|------------------------
* | BW125CR45SF128   | 125   | 4/5 | 128  | Default, medium range | 50ms
* | BW500CR45SF128   | 500   | 4/5 | 128  | Fast, short range     | 15ms
* | BW31_25CR48SF512 | 31.25 | 4/8 | 512  | Slow, long range      | 900ms
* | BW125CR48SF4096  | 125   | 4/8 | 4096 | Slow, long range      | 1500ms
*
* See here for air-time calculation: https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc
*
* @brief API declaration for RFM95
*
*/

#ifndef _RFM95_h
#define _RFM95_h

#include "RFM95registers.h"

#if !defined(RFM95_SPI)
#define RFM95_SPI hwSPI //!< default SPI
#endif

// default PIN assignments, can be overridden
#if defined(ARDUINO_ARCH_AVR)
#if defined(__AVR_ATmega32U4__)
#define DEFAULT_RFM95_IRQ_PIN			(3)				//!< DEFAULT_RFM95_IRQ_PIN
#else
#define DEFAULT_RFM95_IRQ_PIN			(2)				//!< DEFAULT_RFM95_IRQ_PIN
#endif
#elif defined(ARDUINO_ARCH_ESP8266)
#define DEFAULT_RFM95_IRQ_PIN			(5)				//!< DEFAULT_RFM95_IRQ_PIN
#elif defined(ARDUINO_ARCH_ESP32)
#define DEFAULT_RFM95_IRQ_PIN			(16)												//!< DEFAULT_RFM95_IRQ_PIN
#define DEFAULT_RFM95_IRQ_NUM			digitalPinToInterrupt(DEFAULT_RFM95_IRQ_PIN)		//!< DEFAULT_RFM95_IRQ_NUM
#elif defined(ARDUINO_ARCH_SAMD)
#define DEFAULT_RFM95_IRQ_PIN			(2)				//!< DEFAULT_RFM95_IRQ_PIN
#elif defined(LINUX_ARCH_RASPBERRYPI)
#define DEFAULT_RFM95_IRQ_PIN			(22)			//!< DEFAULT_RFM95_IRQ_PIN
#elif defined(ARDUINO_ARCH_STM32F1)
#define DEFAULT_RFM95_IRQ_PIN			(PA3)			//!< DEFAULT_RFM95_IRQ_PIN
#elif defined(ARDUINO_ARCH_STM32)
#define DEFAULT_RFM95_IRQ_PIN			(PA3)			//!< DEFAULT_RFM95_IRQ_PIN
#elif defined(TEENSYDUINO)
#define DEFAULT_RFM95_IRQ_PIN			(8)				//!< DEFAULT_RFM95_IRQ_PIN
#else
#define DEFAULT_RFM95_IRQ_PIN			(2)				//!< DEFAULT_RFM95_IRQ_PIN
#endif

#define DEFAULT_RFM95_CS_PIN			(SS)			//!< DEFAULT_RFM95_CS_PIN

// SPI settings
#define RFM95_SPI_DATA_ORDER		MSBFIRST		//!< SPI data order
#define RFM95_SPI_DATA_MODE			SPI_MODE0		//!< SPI mode

// RFM95 radio configurations: reg_1d, reg_1e, reg_26 (see datasheet)
#define RFM95_BW125CR45SF128 RFM95_BW_125KHZ | RFM95_CODING_RATE_4_5, RFM95_SPREADING_FACTOR_128CPS | RFM95_RX_PAYLOAD_CRC_ON, RFM95_AGC_AUTO_ON //!< 0x72,0x74,0x04
#define RFM95_BW500CR45SF128 RFM95_BW_500KHZ | RFM95_CODING_RATE_4_5, RFM95_SPREADING_FACTOR_128CPS | RFM95_RX_PAYLOAD_CRC_ON, RFM95_AGC_AUTO_ON //!< 0x92,0x74,0x04
#define RFM95_BW31_25CR48SF512 RFM95_BW_31_25KHZ | RFM95_CODING_RATE_4_8, RFM95_SPREADING_FACTOR_512CPS | RFM95_RX_PAYLOAD_CRC_ON, RFM95_AGC_AUTO_ON //!< 0x48,0x94,0x04
#define RFM95_BW125CR48SF4096	RFM95_BW_125KHZ | RFM95_CODING_RATE_4_8, RFM95_SPREADING_FACTOR_4096CPS | RFM95_RX_PAYLOAD_CRC_ON, RFM95_AGC_AUTO_ON | RFM95_LOW_DATA_RATE_OPTIMIZE	//!< 0x78,0xc4,0x0C

#if !defined(RFM95_RETRY_TIMEOUT_MS)
// air-time approximation for timeout, 1 hop ~15 bytes payload - adjust if needed
// BW125/SF128: 50ms
// BW500/SF128: 15ms
// BW31.25/SF512: 900ms
// BW125/SF4096: 1500ms
#define RFM95_RETRY_TIMEOUT_MS			(500ul)			//!< Timeout for ACK, adjustments needed if modem configuration changed (air time different)
#endif

#if !defined(MY_RFM95_TX_TIMEOUT_MS)
#define MY_RFM95_TX_TIMEOUT_MS                 (5*1000ul)		//!< TX timeout
#endif

// Frequency definitions
#define RFM95_169MHZ                           (169000000ul)	//!< 169 Mhz
#define RFM95_315MHZ                           (315000000ul)	//!< 315 Mhz
#define RFM95_434MHZ                           (433920000ul)	//!< 433.92 Mhz
#define RFM95_868MHZ                           (868100000ul)	//!< 868.1 Mhz
#define RFM95_915MHZ                           (915000000ul)	//!< 915 Mhz

#define RFM95_RETRIES                          (5u)					//!< Retries in case of failed transmission
#define RFM95_FIFO_SIZE                        (0xFFu)			//!< Max number of bytes the LORA Rx/Tx FIFO can hold
#define RFM95_RX_FIFO_ADDR                     (0x00u)			//!< RX FIFO addr pointer
#define RFM95_TX_FIFO_ADDR                     (0x80u)			//!< TX FIFO addr pointer
#define RFM95_MAX_PACKET_LEN                   (0x40u)			//!< This is the maximum number of bytes that can be carried by the LORA
#define RFM95_PREAMBLE_LENGTH                  (8u)					//!< Preamble length, default=8
#define RFM95_CAD_TIMEOUT_MS                   (2*1000ul)		//!< channel activity detection timeout
#define RFM95_POWERUP_DELAY_MS                 (100u)				//!< Power up delay, allow VCC to settle, transport to become fully operational

#define RFM95_PACKET_HEADER_VERSION            (1u)			//!< RFM95 packet header version
#define RFM95_MIN_PACKET_HEADER_VERSION        (1u)			//!< Minimal RFM95 packet header version
#define RFM95_BIT_ACK_REQUESTED                (7u)			//!< RFM95 header, controlFlag, bit 7
#define RFM95_BIT_ACK_RECEIVED                 (6u)			//!< RFM95 header, controlFlag, bit 6
#define RFM95_BIT_ACK_RSSI_REPORT              (5u)			//!< RFM95 header, controlFlag, bit 5

#define RFM95_BROADCAST_ADDRESS                (255u)			//!< Broadcasting address 
#define RFM95_ATC_TARGET_RANGE_DBM             (2u)			//!< ATC target range +/- dBm
#define RFM95_RSSI_OFFSET                      (137u)			//!< RSSI offset
#define RFM95_TARGET_RSSI                      (-70)			//!< RSSI target
#define RFM95_PROMISCUOUS                      (false)			//!< RFM95 promiscuous mode

#define RFM95_FXOSC                            (32*1000000ul)				//!< The crystal oscillator frequency of the module
#define RFM95_FSTEP                            (RFM95_FXOSC / 524288.0f)	//!< The Frequency Synthesizer step

// helper macros
#define RFM95_getACKRequested(__value) ((bool)bitRead(__value, RFM95_BIT_ACK_REQUESTED))				//!< getACKRequested
#define RFM95_setACKRequested(__value, __flag) bitWrite(__value, RFM95_BIT_ACK_REQUESTED,__flag)		//!< setACKRequested
#define RFM95_getACKReceived(__value) ((bool)bitRead(__value, RFM95_BIT_ACK_RECEIVED))					//!< getACKReceived
#define RFM95_setACKReceived(__value, __flag) bitWrite(__value, RFM95_BIT_ACK_RECEIVED,__flag)			//!< setACKReceived
#define RFM95_setACKRSSIReport(__value, __flag) bitWrite(__value, RFM95_BIT_ACK_RSSI_REPORT,__flag)		//!< setACKRSSIReport
#define RFM95_getACKRSSIReport(__value) ((bool)bitRead(__value, RFM95_BIT_ACK_RSSI_REPORT))				//!< getACKRSSIReport
#define RFM95_internalToSNR(__value)	((int8_t)(__value / 4))						//!< Convert internal SNR to SNR

#define RFM95_MIN_POWER_LEVEL_DBM		((rfm95_powerLevel_t)5u)	//!< min. power level
#if defined(MY_RFM95_MAX_POWER_LEVEL_DBM)
#define RFM95_MAX_POWER_LEVEL_DBM		MY_RFM95_MAX_POWER_LEVEL_DBM	//!< MY_RFM95_MAX_POWER_LEVEL_DBM
#else
#define RFM95_MAX_POWER_LEVEL_DBM		((rfm95_powerLevel_t)23u)	//!< max. power level
#endif
/**
* @brief Radio modes
*/
typedef enum {
	RFM95_RADIO_MODE_RX = 0,					//!< RX mode
	RFM95_RADIO_MODE_TX = 1,					//!< TX mode
	RFM95_RADIO_MODE_CAD = 2,					//!< CAD mode
	RFM95_RADIO_MODE_SLEEP = 3,					//!< SLEEP mode
	RFM95_RADIO_MODE_STDBY = 4					//!< STDBY mode
} rfm95_radioMode_t;

/**
* @brief RFM95 modem config registers
*/
typedef struct {
	uint8_t	reg_1d; //!< Value for register REG_1D_MODEM_CONFIG1
	uint8_t	reg_1e;	//!< Value for register REG_1E_MODEM_CONFIG2
	uint8_t	reg_26;	//!< Value for register REG_26_MODEM_CONFIG3
} rfm95_modemConfig_t;

/**
* @brief Sequence number data type
*/
typedef uint16_t rfm95_sequenceNumber_t;		// will eventually change to uint8_t in 3.0
/**
* @brief RSSI data type
*/
typedef uint8_t rfm95_RSSI_t;
/**
* @brief SNR data type
*/
typedef int8_t rfm95_SNR_t;
/**
* @brief Control flag data type
*/
typedef uint8_t rfm95_controlFlags_t;
/**
* @brief Power level in dBm
*/
typedef int8_t rfm95_powerLevel_t;
/**
* @brief RFM95 LoRa header
*/
typedef struct {
	uint8_t version;								//!< Header version
	uint8_t recipient;								//!< Payload recipient
	uint8_t sender;									//!< Payload sender
	rfm95_controlFlags_t controlFlags;				//!< Control flags, used for ACK
	rfm95_sequenceNumber_t sequenceNumber;			//!< Packet sequence number, used for ACK
} __attribute__((packed)) rfm95_header_t;

/**
* @brief RFM95 LoRa ACK packet structure
*/
typedef struct {
	rfm95_sequenceNumber_t sequenceNumber;				//!< sequence number
	rfm95_RSSI_t RSSI;									//!< RSSI
	rfm95_SNR_t SNR;									//!< SNR
} __attribute__((packed)) rfm95_ack_t;


#define RFM95_HEADER_LEN sizeof(rfm95_header_t)		//!< Size header inside LoRa payload
#define RFM95_MAX_PAYLOAD_LEN (RFM95_MAX_PACKET_LEN - RFM95_HEADER_LEN)	//!< Max payload length

/**
* @brief LoRa packet structure
*/
typedef struct {
	union {
		struct {
			rfm95_header_t header;						//!< LoRa header
			union {
				uint8_t payload[RFM95_MAX_PAYLOAD_LEN];	//!< Payload, i.e. MySensors message
				rfm95_ack_t ACK;						//!< Union: ACK
			};
		};
		uint8_t data[RFM95_MAX_PACKET_LEN];				//!< RAW
	};
	uint8_t payloadLen;									//!< Length of payload (excluding header)
	rfm95_RSSI_t RSSI;									//!< RSSI of current packet, RSSI = value - 137
	rfm95_SNR_t SNR;									//!< SNR of current packet
} __attribute__((packed)) rfm95_packet_t;


/**
* @brief RFM95 internal variables
*/
typedef struct {
	uint8_t address;                          //!< Node address
	rfm95_packet_t currentPacket;             //!< Buffer for current packet
	rfm95_sequenceNumber_t txSequenceNumber;  //!< RFM95_txSequenceNumber
	rfm95_powerLevel_t powerLevel;            //!< TX power level dBm
	rfm95_RSSI_t ATCtargetRSSI;               //!< ATC: target RSSI
	// 8 bit
	rfm95_radioMode_t radioMode : 3;          //!< current transceiver state
	bool channelActive : 1;                   //!< RFM95_cad
	bool ATCenabled : 1;                      //!< ATC enabled
	bool ackReceived : 1;                     //!< ACK received
	bool dataReceived : 1;                    //!< Data received
	bool reserved : 1;                        //!< unused
} rfm95_internal_t;

#define LOCAL static		//!< static

/**
* @brief Initialise the driver transport hardware and software
* @param frequencyHz Transmitter frequency in Hz
* @return True if initialisation succeeded
*/
LOCAL bool RFM95_initialise(const uint32_t frequencyHz);
/**
* @brief Set the driver/node address
* @param addr
*/
LOCAL void RFM95_setAddress(const uint8_t addr);
/**
* @brief Get driver/node address
* @return Node address
*/
LOCAL uint8_t RFM95_getAddress(void);
/**
* @brief Sets all the registers required to configure the data modem in the RF95/96/97/98, including the
* bandwidth, spreading factor etc.
* @param config See modemConfig_t and references therein
*/
LOCAL void RFM95_setModemRegisters(const rfm95_modemConfig_t *config);
/**
* @brief Tests whether a new message is available
* @return True if a new, complete, error-free uncollected message is available to be retreived by @ref RFM95_receive()
*/
LOCAL bool RFM95_available(void);
/**
* @brief If a valid message is received, copy it to buf and return length. 0 byte messages are permitted.
* @param buf Location to copy the received message
* @param maxBufSize Max buffer size
* @return Number of bytes
*/
LOCAL uint8_t RFM95_receive(uint8_t *buf, const uint8_t maxBufSize);
/**
* @brief RFM95_send
* @param recipient
* @param data
* @param len
* @param flags
* @param increaseSequenceCounter
* @return True if packet sent
*/
LOCAL bool RFM95_send(const uint8_t recipient, uint8_t *data, const uint8_t len,
                      const rfm95_controlFlags_t flags, const bool increaseSequenceCounter = true);
/**
* @brief RFM95_sendFrame
* @param packet
* @param increaseSequenceCounter
* @return True if frame sent
*/
LOCAL bool RFM95_sendFrame(rfm95_packet_t *packet, const bool increaseSequenceCounter = true);
/**
* @brief RFM95_setPreambleLength
* @param preambleLength
*/
LOCAL void RFM95_setPreambleLength(const uint16_t preambleLength);
/**
* @brief Sets the transmitter and receiver centre frequency
* @param frequencyHz Frequency in Hz
*/
LOCAL void RFM95_setFrequency(const uint32_t frequencyHz);
/**
* @brief Sets the transmitter power output level, and configures the transmitter pin
* @param newPowerLevel Transmitter power level in dBm (+5 to +23)
* @return True power level adjusted
*/
LOCAL bool RFM95_setTxPowerLevel(rfm95_powerLevel_t newPowerLevel);

/**
* @brief Sets the transmitter power output percent.
* @param newPowerPercent Transmitter power level in percent
* @return True power level adjusted
*/
LOCAL bool RFM95_setTxPowerPercent(const uint8_t newPowerPercent);

/**
* @brief Enable TCXO mode
* Call this immediately after init(), to force your radio to use an external
* frequency source, such as a Temperature Compensated Crystal Oscillator (TCXO).
* See the comments in the main documentation about the sensitivity of this radio to
* clock frequency especially when using narrow bandwidths.
* @note Has to be called while radio is in sleep mode.
*/
LOCAL void RFM95_enableTCXO(void);

/**
* @brief Sets the radio into low-power sleep mode
* @return true if sleep mode was successfully entered
*/
LOCAL bool RFM95_sleep(void);
/**
* @brief Sets the radio into standby mode
* @return true if standby mode was successfully entered
*/
LOCAL bool RFM95_standBy(void);
/**
* @brief Powerdown radio, if RFM95_POWER_PIN defined
*/
LOCAL void RFM95_powerDown(void);
/**
* @brief Powerup radio, if RFM95_POWER_PIN defined
*/
LOCAL void RFM95_powerUp(void);
/**
* @brief RFM95_sendACK
* @param recipient
* @param sequenceNumber
* @param RSSI (rfm95_RSSI_t)
* @param SNR (rfm95_SNR_t)
*/
LOCAL void RFM95_sendACK(const uint8_t recipient, const rfm95_sequenceNumber_t sequenceNumber,
                         const rfm95_RSSI_t RSSI, const rfm95_SNR_t SNR);
/**
* @brief RFM95_sendWithRetry
* @param recipient
* @param buffer
* @param bufferSize
* @param noACK
* @return True if packet successfully sent
*/
LOCAL bool RFM95_sendWithRetry(const uint8_t recipient, const void *buffer,
                               const uint8_t bufferSize, const bool noACK);
/**
* @brief Wait until no channel activity detected
* @return True if no channel activity detected, False if timeout occured
*/
LOCAL bool RFM95_waitCAD(void);

/**
* @brief RFM95_setRadioMode
* @param newRadioMode
* @return True if mode changed
*/
LOCAL bool RFM95_setRadioMode(const rfm95_radioMode_t newRadioMode);
/**
* @brief Low level interrupt handler
*/
LOCAL void RFM95_interruptHandler(void);

/**
* @brief Packet engine
*/
LOCAL void RFM95_interruptHandling(void);

/**
* @brief RFM95_handler
*/
LOCAL void RFM95_handler(void);
/**
* @brief RFM95_getSendingRSSI
* @return RSSI Signal strength of last packet received
*/
LOCAL int16_t RFM95_getReceivingRSSI(void);
/**
* @brief RFM95_getSendingRSSI
* @return RSSI Signal strength of last packet sent (if ACK and ATC enabled)
*/
LOCAL int16_t RFM95_getSendingRSSI(void);
/**
* @brief RFM95_getReceivingSNR
* @return SNR
*/
LOCAL int16_t RFM95_getReceivingSNR(void);
/**
* @brief RFM95_getSendingSNR
* @return SNR of last packet sent (if ACK and ATC enabled)
*/
LOCAL int16_t RFM95_getSendingSNR(void);
/**
* @brief Get transmitter power level
* @return Transmitter power level in percents
*/
LOCAL uint8_t RFM95_getTxPowerPercent(void);
/**
* @brief Get transmitter power level
* @return Transmitter power level in dBm
*/
LOCAL uint8_t RFM95_getTxPowerLevel(void);
/**
* @brief RFM_executeATC
* @param currentRSSI
* @param targetRSSI
* @return True if power level adjusted
*/
LOCAL bool RFM95_executeATC(const rfm95_RSSI_t currentRSSI, const rfm95_RSSI_t targetRSSI);
/**
* @brief RFM95_ATCmode
* @param targetRSSI Target RSSI for transmitter (default -60)
* @param OnOff True to enable ATC
*/
LOCAL void RFM95_ATCmode(const bool OnOff, const int16_t targetRSSI = RFM95_TARGET_RSSI);
/**
* @brief RFM95_sanityCheck
* @return True if sanity check passed
*/
LOCAL bool RFM95_sanityCheck(void);

#endif

/** @}*/
