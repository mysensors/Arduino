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
 * TransportHAL debug log messages:
 *
 * |E| SYS | SUB   | Message										| Comment
 * |-|-----|-------|----------------------------|---------------------------------------------------------------------
 * | | THA | INIT  |														| Initialize transportHAL
 * | | THA | INIT  | PSK=%%s										| Load PSK for transport encryption (%AES)
 * | | THA | SAD   | ADDR=%%d										| Set transport address (ADDR)
 * | | THA | GAD   | ADDR=%%d										| Get trnasport address (ADDR)
 * | | THA | DATA  | AVAIL											| Message available
 * | | THA | SAN   | RES=%%d										| Transport sanity check, result (RES)
 * | | THA | RCV   | MSG=%%s										| Receive message (MSG)
 * | | THA | RCV   | DECRYPT										| Decrypt received message
 * | | THA | RCV   | PLAIN=%%s									| Decrypted message (PLAIN)
 * |!| THA | RCV   | PVER=%%d										| Message protocol version (PVER) mismatch
 * |!| THA | RCV   | LEN=%%d,EXP=%%d						| Invalid message length (LEN), exptected length (EXP)
 * | | THA | RCV   | MSG LEN=%%d								| Length of received message (LEN)
 * | | THA | SND   | MSG=%%s										| Send message (MSG)
 * | | THA | SND   | ENCRYPT										| Encrypt message to send (%AES)
 * | | THA | SND   | CIP=%%s										| Ciphertext of encypted message (CIP)
 * | | THA | SND   | MSG LEN=%%d,RES=%%d				| Sending message with length (LEN), result (RES)
 *
 *
 */

#ifndef MyTransportHAL_h
#define MyTransportHAL_h

#define INVALID_SNR         ((int16_t)-256)	//!< INVALID_SNR
#define INVALID_RSSI        ((int16_t)-256)	//!< INVALID_RSSI
#define INVALID_PERCENT     ((int16_t)-100)	//!< INVALID_PERCENT
#define INVALID_LEVEL       ((int16_t)-256)	//!< INVALID_LEVEL

#if defined(MY_RX_MESSAGE_BUFFER_FEATURE)
#if defined(MY_RADIO_NRF5_ESB)
#error Receive message buffering not supported for NRF5 radio! Please define MY_NRF5_RX_BUFFER_SIZE
#endif
#if defined(MY_RADIO_RFM69)
#error Receive message buffering not supported for RFM69!
#endif
#if defined(MY_RADIO_RFM95)
#error Receive message buffering not supported for RFM95!
#endif
#if defined(MY_RS485)
#error Receive message buffering not supported for RS485!
#endif
#if defined(MY_CAN)
#error Receive message buffering not supported for CAN!
#endif
#elif defined(MY_RX_MESSAGE_BUFFER_SIZE)
#error Receive message buffering requires message buffering feature enabled!
#endif

/**
* @brief Signal report selector
*/
typedef enum {
	SR_RX_RSSI,            //!< SR_RX_RSSI
	SR_TX_RSSI,            //!< SR_TX_RSSI
	SR_RX_SNR,             //!< SR_RX_SNR
	SR_TX_SNR,             //!< SR_TX_SNR
	SR_TX_POWER_LEVEL,     //!< SR_TX_POWER_LEVEL
	SR_TX_POWER_PERCENT,   //!< SR_TX_POWER_PERCENT
	SR_UPLINK_QUALITY,     //!< SR_UPLINK_QUALITY
	SR_NOT_DEFINED         //!< SR_NOT_DEFINED
} signalReport_t;


/**
* @brief Initialize transport HW
* @return true if initialization successful
*/
bool transportHALInit(void);
/**
* @brief Set node address
*/
void transportHALSetAddress(const uint8_t address);
/**
* @brief Retrieve node address
*/
uint8_t transportHALGetAddress(void);
/**
* @brief Send message
* @param to recipient
* @param data message to be sent
* @param len length of message (header + payload)
* @param noACK do not wait for ACK
* @return true if message sent successfully
*/
bool transportHALSend(const uint8_t nextRecipient, const MyMessage *outMsg, const uint8_t len,
                      const bool noACK);
/**
* @brief Verify if RX FIFO has pending messages
* @return true if message available in RX FIFO
*/
bool transportHALDataAvailable(void);
/**
* @brief Sanity check for transport: is transport HW still responsive?
* @return true if transport HW is ok
*/
bool transportHALSanityCheck(void);
/**
* @brief Receive message from FIFO
* @param inMsg
* @param msgLength length of received message (header + payload)
* @return True if valid message received
*/
bool transportHALReceive(MyMessage *inMsg, uint8_t *msgLength);
/**
* @brief Power down transport HW (if corresponding MY_XYZ_POWER_PIN defined)
*/
void transportHALPowerDown(void);
/**
* @brief Power up transport HW (if corresponding MY_XYZ_POWER_PIN defined)
*/
void transportHALPowerUp(void);
/**
* @brief Set transport HW to sleep (no power down)
*/
void transportHALSleep(void);
/**
* @brief Set transport HW to standby
*/
void transportHALStandBy(void);
/**
* @brief transportGetSendingRSSI
* @return RSSI of outgoing message (via ACK packet)
*/
int16_t transportHALGetSendingRSSI(void);
/**
* @brief transportGetReceivingRSSI
* @return RSSI of incoming message
*/
int16_t transportHALGetReceivingRSSI(void);
/**
* @brief transportGetSendingSNR
* @return SNR of outgoing message (via ACK packet)
*/
int16_t transportHALGetSendingSNR(void);
/**
* @brief transportGetReceivingSNR
* @return SNR of incoming message
*/
int16_t transportHALGetReceivingSNR(void);
/**
* @brief transportGetTxPowerPercent
* @return TX power level in percent
*/
int16_t transportHALGetTxPowerPercent(void);
/**
* @brief transportSetTxPowerPercent
* @param powerPercent power level in percent
* @return True if power level set
*/
bool transportHALSetTxPowerPercent(const uint8_t powerPercent);
/**
* @brief transportGetTxPowerLevel
* @return TX power in dBm
*/
int16_t transportHALGetTxPowerLevel(void);

#endif // MyTransportHAL_h
