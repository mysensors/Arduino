/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2020 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 */

#include "MyTransportHAL.h"

#if defined(MY_DEBUG_VERBOSE_TRANSPORT_HAL)
#define TRANSPORT_HAL_DEBUG(x,...) DEBUG_OUTPUT(x, ##__VA_ARGS__)	//!< debug
#else
#define TRANSPORT_HAL_DEBUG(x,...)	//!< debug NULL
#endif

bool transportHALInit(void)
{
	TRANSPORT_HAL_DEBUG(PSTR("THA:INIT\n"));
#if defined(MY_TRANSPORT_ENCRYPTION)
	uint8_t transportPSK[16];
#if defined(MY_ENCRYPTION_SIMPLE_PASSWD)
	(void)memset((void *)transportPSK, 0, sizeof(transportPSK));
	(void)memcpy((void *)transportPSK, MY_ENCRYPTION_SIMPLE_PASSWD,
	             strnlen(MY_ENCRYPTION_SIMPLE_PASSWD, sizeof(transportPSK)));
#else
	hwReadConfigBlock((void *)transportPSK, (void *)EEPROM_RF_ENCRYPTION_AES_KEY_ADDRESS,
	                  sizeof(transportPSK));
#endif
#if defined(MY_DEBUG_VERBOSE_TRANSPORT_HAL)
	hwDebugBuf2Str((const uint8_t *)transportPSK, sizeof(transportPSK));
	TRANSPORT_HAL_DEBUG(PSTR("THA:INIT:PSK=%s\n"),hwDebugPrintStr);
#endif
#endif
	bool result = transportInit();

#if defined(MY_TRANSPORT_ENCRYPTION)
#if defined(MY_RADIO_RFM69)
	transportEncrypt((const char *)transportPSK);
#else
	//set up AES-key
	AES128CBCInit(transportPSK);
#endif
	// Make sure it is purged from memory when set
	(void)memset((void *)transportPSK, 0,
	             sizeof(transportPSK));
#endif
	return result;
}

void transportHALSetAddress(const uint8_t address)
{
	TRANSPORT_HAL_DEBUG(PSTR("THA:SAD:ADDR=%" PRIu8 "\n"), address);
	transportSetAddress(address);
}

uint8_t transportHALGetAddress(void)
{
	uint8_t result = transportGetAddress();
	TRANSPORT_HAL_DEBUG(PSTR("THA:GAD:ADDR=%" PRIu8 "\n"), result);
	return result;
}

bool transportHALDataAvailable(void)
{
	bool result = transportDataAvailable();
#if defined(MY_DEBUG_VERBOSE_TRANSPORT_HAL)
	if (result) {
		TRANSPORT_HAL_DEBUG(PSTR("THA:DATA:AVAIL\n"));
	}
#endif
	return result;
}

bool transportHALSanityCheck(void)
{
	bool result = transportSanityCheck();
	TRANSPORT_HAL_DEBUG(PSTR("THA:SAN:RES=%" PRIu8 "\n"), result);
	return result;
}

bool transportHALReceive(MyMessage *inMsg, uint8_t *msgLength)
{
	// set pointer to first byte of data structure
	uint8_t *rx_data = &inMsg->last;
	uint8_t payloadLength = transportReceive((void *)rx_data);
#if defined(MY_DEBUG_VERBOSE_TRANSPORT_HAL)
	hwDebugBuf2Str((const uint8_t *)rx_data, payloadLength);
	TRANSPORT_HAL_DEBUG(PSTR("THA:RCV:MSG=%s\n"), hwDebugPrintStr);
#endif
#if defined(MY_TRANSPORT_ENCRYPTION) && !defined(MY_RADIO_RFM69)
	TRANSPORT_HAL_DEBUG(PSTR("THA:RCV:DECRYPT\n"));
	// has to be adjusted, WIP!
	uint8_t IV[16] = { 0 };
	// decrypt data
	AES128CBCDecrypt(IV, (uint8_t *)rx_data, payloadLength);
#if defined(MY_DEBUG_VERBOSE_TRANSPORT_HAL)
	hwDebugBuf2Str((const uint8_t *)rx_data, payloadLength);
	TRANSPORT_HAL_DEBUG(PSTR("THA:RCV:PLAIN=%s\n"), hwDebugPrintStr);
#endif
#endif
	// Reject messages with incorrect protocol version
	MyMessage tmp = *inMsg;
	if (!tmp.isProtocolVersionValid()) {
		setIndication(INDICATION_ERR_VERSION);
		TRANSPORT_HAL_DEBUG(PSTR("!THA:RCV:PVER=%" PRIu8 "\n"),
		                    tmp.getVersion());	// protocol version mismatch
		return false;
	}
	*msgLength = tmp.getLength();
#if defined(MY_TRANSPORT_ENCRYPTION) && !defined(MY_RADIO_RFM69)
	// payload length = a multiple of blocksize length for decrypted messages, i.e. cannot be used for payload length check
#else
	// Reject payloads with incorrect length
	const uint8_t expectedMessageLength = tmp.getExpectedMessageSize();
	if (payloadLength != expectedMessageLength) {
		setIndication(INDICATION_ERR_LENGTH);
		TRANSPORT_HAL_DEBUG(PSTR("!THA:RCV:LEN=%" PRIu8 ",EXP=%" PRIu8 "\n"), payloadLength,
		                    expectedMessageLength); // invalid payload length
		return false;
	}
#endif
	TRANSPORT_HAL_DEBUG(PSTR("THA:RCV:MSG LEN=%" PRIu8 "\n"), payloadLength);
	return true;
}

bool transportHALSend(const uint8_t nextRecipient, const MyMessage *outMsg, const uint8_t len,
                      const bool noACK)
{
	if (outMsg == NULL) {

		// nothing to send
		return false;
	}
#if defined(MY_DEBUG_VERBOSE_TRANSPORT_HAL)
	hwDebugBuf2Str((const uint8_t *)&outMsg->last, len);
	TRANSPORT_HAL_DEBUG(PSTR("THA:SND:MSG=%s\n"), hwDebugPrintStr);
#endif

#if defined(MY_TRANSPORT_ENCRYPTION) && !defined(MY_RADIO_RFM69)
	TRANSPORT_HAL_DEBUG(PSTR("THA:SND:ENCRYPT\n"));
	uint8_t *tx_data[MAX_MESSAGE_SIZE];
	// copy input data because it is read-only
	(void)memcpy((void *)tx_data, (void *)&outMsg->last, len);
	// We us IV vector filled with zeros but randomize unused bytes in encryption block
	uint8_t IV[16] = { 0 };
	const uint8_t finalLength = len > 16 ? 32 : 16;
	// fill block with random data
	for (uint8_t i = len; i < finalLength; i++) {
		*((uint8_t *)tx_data + i) = random(256);
	}
	//encrypt data
	AES128CBCEncrypt(IV, (uint8_t *)tx_data, finalLength);
#if defined(MY_DEBUG_VERBOSE_TRANSPORT_HAL)
	hwDebugBuf2Str((const uint8_t *)tx_data, finalLength);
	TRANSPORT_HAL_DEBUG(PSTR("THA:SND:CIP=%s\n"), hwDebugPrintStr);
#endif

#else
	const uint8_t *tx_data = &outMsg->last;
	const uint8_t finalLength = len;
#endif

#ifdef MY_TRANSPORT_SEND_RETRIES
	uint8_t retries = MY_TRANSPORT_SEND_RETRIES;
	bool result = false; 
	while (retries-- && (!result)) {
		result = transportSend(nextRecipient, (void *)tx_data, finalLength, noACK);
	}
#else
	bool result = transportSend(nextRecipient, (void *)tx_data, finalLength, noACK);
#endif
	TRANSPORT_HAL_DEBUG(PSTR("THA:SND:MSG LEN=%" PRIu8 ",RES=%" PRIu8 "\n"), finalLength, result);
	return result;
}

void transportHALPowerDown(void)
{
	transportPowerDown();
}

void transportHALPowerUp(void)
{
	transportPowerUp();
}

void transportHALSleep(void)
{
	transportSleep();
}

void transportHALStandBy(void)
{
	transportStandBy();
}

int16_t transportHALGetSendingRSSI(void)
{
	int16_t result = transportGetSendingRSSI();
	return result;
}

int16_t transportHALGetReceivingRSSI(void)
{
	int16_t result = transportGetReceivingRSSI();
	return result;
}

int16_t transportHALGetSendingSNR(void)
{
	int16_t result = transportGetSendingSNR();
	return result;
}

int16_t transportHALGetReceivingSNR(void)
{
	int16_t result = transportGetReceivingSNR();
	return result;
}

int16_t transportHALGetTxPowerPercent(void)
{
	int16_t result = transportGetTxPowerPercent();
	return result;
}

bool transportHALSetTxPowerPercent(const uint8_t powerPercent)
{
	bool result = transportSetTxPowerPercent(powerPercent);
	return result;
}

int16_t transportHALGetTxPowerLevel(void)
{
	int16_t result = transportGetTxPowerLevel();
	return result;
}
