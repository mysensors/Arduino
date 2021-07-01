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
*/

#include "RF24.h"

// debug output
#if defined(MY_DEBUG_VERBOSE_RF24)
#define RF24_DEBUG(x,...)	DEBUG_OUTPUT(x, ##__VA_ARGS__)	//!< DEBUG
#else
#define RF24_DEBUG(x,...)	//!< DEBUG null
#endif

LOCAL uint8_t RF24_BASE_ID[MY_RF24_ADDR_WIDTH] = { MY_RF24_BASE_RADIO_ID };
LOCAL uint8_t RF24_NODE_ADDRESS = RF24_BROADCAST_ADDRESS;

#if defined(MY_RX_MESSAGE_BUFFER_FEATURE)
LOCAL RF24_receiveCallbackType RF24_receiveCallback = NULL;
#endif

#if defined(__linux__)
uint8_t RF24_spi_rxbuff[32+1] ; //SPI receive buffer (payload max 32 bytes)
uint8_t RF24_spi_txbuff[32+1]
; //SPI transmit buffer (payload max 32 bytes + 1 byte for the command)
#endif

LOCAL void RF24_csn(const bool level)
{
#if defined(__linux__)
	(void)level;
#else
	hwDigitalWrite(MY_RF24_CS_PIN, level);
#endif
}

LOCAL void RF24_ce(const bool level)
{
	hwDigitalWrite(MY_RF24_CE_PIN, level);
}

LOCAL uint8_t RF24_spiMultiByteTransfer(const uint8_t cmd, uint8_t *buf, uint8_t len,
                                        const bool readMode)
{
	uint8_t status;
	uint8_t *current = buf;
#if !defined(MY_SOFTSPI) && defined(SPI_HAS_TRANSACTION)
	RF24_SPI.beginTransaction(SPISettings(MY_RF24_SPI_SPEED, RF24_SPI_DATA_ORDER,
	                                      RF24_SPI_DATA_MODE));
#endif

	RF24_csn(LOW);
	// timing
	delayMicroseconds(10);
#ifdef __linux__
	uint8_t *prx = RF24_spi_rxbuff;
	uint8_t *ptx = RF24_spi_txbuff;
	uint8_t size = len + 1; // Add register value to transmit buffer

	*ptx++ = cmd;
	while ( len-- ) {
		if (readMode) {
			*ptx++ = RF24_CMD_NOP;
		} else {
			*ptx++ = *current++;
		}
	}
	RF24_SPI.transfernb( (char *) RF24_spi_txbuff, (char *) RF24_spi_rxbuff, size);
	if (readMode) {
		if (size == 2) {
			status = *++prx;   // result is 2nd byte of receive buffer
		} else {
			status = *prx++; // status is 1st byte of receive buffer
			// decrement before to skip status byte
			while (--size && (buf != NULL)) {
				*buf++ = *prx++;
			}
		}
	} else {
		status = *prx; // status is 1st byte of receive buffer
	}
#else
	status = RF24_SPI.transfer(cmd);
	while ( len-- ) {
		if (readMode) {
			status = RF24_SPI.transfer(RF24_CMD_NOP);
			if (buf != NULL) {
				*current++ = status;
			}
		} else {
			(void)RF24_SPI.transfer(*current++);
		}
	}
#endif

	RF24_csn(HIGH);
#if !defined(MY_SOFTSPI) && defined(SPI_HAS_TRANSACTION)
	RF24_SPI.endTransaction();
#endif
	// timing
	delayMicroseconds(10);
	return status;
}

LOCAL uint8_t RF24_spiByteTransfer(const uint8_t cmd)
{
	return RF24_spiMultiByteTransfer(cmd, NULL, 0, false);
}

LOCAL uint8_t RF24_RAW_readByteRegister(const uint8_t cmd)
{
	const uint8_t value = RF24_spiMultiByteTransfer(cmd, NULL, 1, true);
	RF24_DEBUG(PSTR("RF24:RBR:REG=%" PRIu8 ",VAL=%" PRIu8 "\n"), cmd & RF24_REGISTER_MASK, value);
	return value;
}

LOCAL uint8_t RF24_RAW_writeByteRegister(const uint8_t cmd, uint8_t value)
{
	RF24_DEBUG(PSTR("RF24:WBR:REG=%" PRIu8 ",VAL=%" PRIu8 "\n"), cmd & RF24_REGISTER_MASK, value);
	return RF24_spiMultiByteTransfer( cmd, &value, 1, false);
}

LOCAL void RF24_flushRX(void)
{
	RF24_DEBUG(PSTR("RF24:FRX\n"));
	RF24_spiByteTransfer(RF24_CMD_FLUSH_RX);
}

LOCAL void RF24_flushTX(void)
{
	RF24_DEBUG(PSTR("RF24:FTX\n"));
	RF24_spiByteTransfer(RF24_CMD_FLUSH_TX);
}

LOCAL uint8_t RF24_getStatus(void)
{
	return RF24_spiByteTransfer(RF24_CMD_NOP);
}

LOCAL uint8_t RF24_getFIFOStatus(void)
{
	return RF24_readByteRegister(RF24_REG_FIFO_STATUS);
}

LOCAL void RF24_setChannel(const uint8_t channel)
{
	RF24_writeByteRegister(RF24_REG_RF_CH,channel);
}

LOCAL void RF24_setRetries(const uint8_t retransmitDelay, const uint8_t retransmitCount)
{
	RF24_writeByteRegister(RF24_REG_SETUP_RETR,
	                       retransmitDelay << RF24_ARD | retransmitCount << RF24_ARC);
}

LOCAL void RF24_setAddressWidth(const uint8_t addressWidth)
{
	RF24_writeByteRegister(RF24_REG_SETUP_AW, addressWidth - 2);
}

LOCAL void RF24_setRFSetup(const uint8_t RFsetup)
{
	RF24_writeByteRegister(RF24_REG_RF_SETUP, RFsetup);
}

LOCAL void RF24_setFeature(const uint8_t feature)
{
	RF24_writeByteRegister(RF24_REG_FEATURE, feature);

	if (RF24_getFeature() != feature) {
		// toggle features (necessary on some clones and non-P versions)
		RF24_enableFeatures();
		RF24_writeByteRegister(RF24_REG_FEATURE, feature);
	}
}

LOCAL uint8_t RF24_getFeature(void)
{
	return RF24_readByteRegister(RF24_REG_FEATURE);
}

LOCAL void RF24_setPipe(const uint8_t pipe)
{
	RF24_writeByteRegister(RF24_REG_EN_RXADDR, pipe);
}

LOCAL void RF24_setAutoACK(const uint8_t pipe)
{
	RF24_writeByteRegister(RF24_REG_EN_AA, pipe);
}

LOCAL void RF24_setDynamicPayload(const uint8_t pipe)
{
	RF24_writeByteRegister(RF24_REG_DYNPD, pipe);
}

LOCAL void RF24_setRFConfiguration(const uint8_t configuration)
{
	RF24_writeByteRegister(RF24_REG_NRF_CONFIG, configuration);
}

LOCAL void RF24_setPipeAddress(const uint8_t pipe, uint8_t *address, const uint8_t addressWidth)
{
	RF24_writeMultiByteRegister(pipe, address, addressWidth);
}

LOCAL void RF24_setPipeLSB(const uint8_t pipe, const uint8_t LSB)
{
	RF24_writeByteRegister(pipe, LSB);
}

LOCAL uint8_t RF24_getObserveTX(void)
{
	return RF24_readByteRegister(RF24_REG_OBSERVE_TX);
}

LOCAL uint8_t RF24_setStatus(const uint8_t status)
{
	return RF24_writeByteRegister(RF24_REG_STATUS, status);
}

LOCAL void RF24_enableFeatures(void)
{
	RF24_RAW_writeByteRegister(RF24_CMD_ACTIVATE, 0x73);
}

LOCAL void RF24_openWritingPipe(const uint8_t recipient)
{
	RF24_DEBUG(PSTR("RF24:OWP:RCPT=%" PRIu8 "\n"), recipient); // open writing pipe
	// only write LSB of RX0 and TX pipe
	RF24_setPipeLSB(RF24_REG_RX_ADDR_P0, recipient);
	RF24_setPipeLSB(RF24_REG_TX_ADDR, recipient);
}

LOCAL void RF24_startListening(void)
{
	RF24_DEBUG(PSTR("RF24:STL\n"));	// start listening
#if MY_RF24_CE_PIN == NOT_A_PIN
	RF24_sleep();
#endif
	// toggle PRX
	RF24_setRFConfiguration(RF24_CONFIGURATION | _BV(RF24_PWR_UP) | _BV(RF24_PRIM_RX) );
	// all RX pipe addresses must be unique, therefore skip if node ID is RF24_BROADCAST_ADDRESS
	if(RF24_NODE_ADDRESS!= RF24_BROADCAST_ADDRESS) {
		RF24_setPipeLSB(RF24_REG_RX_ADDR_P0, RF24_NODE_ADDRESS);
	}
#if MY_RF24_CE_PIN != NOT_A_PIN
	// start listening
	RF24_ce(HIGH);
#endif
}

LOCAL void RF24_stopListening(void)
{
	RF24_DEBUG(PSTR("RF24:SPL\n"));	// stop listening
#if MY_RF24_CE_PIN == NOT_A_PIN
	RF24_sleep();
#else
	RF24_ce(LOW);
#endif
	// timing
	delayMicroseconds(130);
	RF24_setRFConfiguration(RF24_CONFIGURATION | _BV(RF24_PWR_UP) );
	// timing
	delayMicroseconds(100);
}

LOCAL void RF24_powerDown(void)
{
#if defined(MY_RF24_POWER_PIN)
	hwDigitalWrite(MY_RF24_POWER_PIN, LOW);
#endif
}

LOCAL void RF24_powerUp(void)
{
#if defined(MY_RF24_POWER_PIN)
	hwDigitalWrite(MY_RF24_POWER_PIN, HIGH);
	delay(RF24_POWERUP_DELAY_MS);	// allow VCC to settle
#endif
}
LOCAL void RF24_sleep(void)
{
	RF24_DEBUG(PSTR("RF24:SLP\n")); // put radio to sleep
	RF24_ce(LOW);
	RF24_setRFConfiguration(RF24_CONFIGURATION);
}

LOCAL void RF24_standBy(void)
{
	RF24_DEBUG(PSTR("RF24:SBY\n")); // put radio to standby
	RF24_ce(LOW);
	RF24_setRFConfiguration(RF24_CONFIGURATION | _BV(RF24_PWR_UP));
	// There must be a delay of up to 4.5ms after the nRF24L01+ leaves power down mode before the CE is set high.
	delayMicroseconds(4500);
}


LOCAL bool RF24_sendMessage(const uint8_t recipient, const void *buf, const uint8_t len,
                            const bool noACK)
{
	RF24_stopListening();
	RF24_openWritingPipe(recipient);
	RF24_DEBUG(PSTR("RF24:TXM:TO=%" PRIu8 ",LEN=%" PRIu8 "\n"), recipient, len); // send message
	// flush TX FIFO
	RF24_flushTX();
	if (noACK) {
		// noACK messages are only sent once
		RF24_setRetries(RF24_SET_ARD, 0);
	}
	// this command is affected in clones (e.g. Si24R1):  flipped NoACK bit when using W_TX_PAYLOAD_NO_ACK / W_TX_PAYLOAD
	// AutoACK is disabled on the broadcasting pipe - NO_ACK prevents resending
	(void)RF24_spiMultiByteTransfer(RF24_CMD_WRITE_TX_PAYLOAD, (uint8_t *)buf, len, false);
	// go, TX starts after ~10us, CE high also enables PA+LNA on supported HW
	RF24_ce(HIGH);
	// timeout counter to detect HW issues
	uint16_t timeout = 0xFFFF;
	while (!(RF24_getStatus() & (_BV(RF24_MAX_RT) | _BV(RF24_TX_DS))) && timeout--) {
		doYield();
	}
	// timeout value after successful TX on 16Mhz AVR ~ 65500, i.e. msg is transmitted after ~36 loop cycles
	RF24_ce(LOW);
	// reset interrupts
	const uint8_t RF24_status = RF24_setStatus(_BV(RF24_RX_DR) | _BV(RF24_TX_DS) | _BV(RF24_MAX_RT));
	// Max retries exceeded
	if (RF24_status & _BV(RF24_MAX_RT)) {
		// flush packet
		RF24_DEBUG(PSTR("?RF24:TXM:MAX_RT\n"));	// max retries (normal messages) and noACK messages
		RF24_flushTX();
	}
	if (noACK) {
		RF24_setRetries(RF24_SET_ARD, RF24_SET_ARC);
	}
	RF24_startListening();
	// true if message sent
	return (RF24_status & _BV(RF24_TX_DS) || noACK);
}

LOCAL uint8_t RF24_getDynamicPayloadSize(void)
{
	uint8_t result = RF24_spiMultiByteTransfer(RF24_CMD_READ_RX_PL_WID, NULL, 1, true);
	// check if payload size invalid
	if(result > 32) {
		RF24_DEBUG(PSTR("!RF24:GDP:PYL INV\n")); // payload len invalid
		RF24_flushRX();
		result = 0;
	}
	return result;
}

LOCAL bool RF24_isDataAvailable(void)
{
	// prevent debug message flooding
#if defined(MY_DEBUG_VERBOSE_RF24)
	const uint8_t value = RF24_spiMultiByteTransfer(RF24_CMD_READ_REGISTER | (RF24_REGISTER_MASK &
	                      (RF24_REG_FIFO_STATUS)), NULL, 1, true);
	return (bool)(!(value & _BV(RF24_RX_EMPTY)));
#else
	return (bool)(!(RF24_getFIFOStatus() & _BV(RF24_RX_EMPTY)) );
#endif
}

LOCAL uint8_t RF24_readMessage(void *buf)
{
	const uint8_t len = RF24_getDynamicPayloadSize();
	RF24_DEBUG(PSTR("RF24:RXM:LEN=%" PRIu8 "\n"), len);	// read message
	RF24_spiMultiByteTransfer(RF24_CMD_READ_RX_PAYLOAD, (uint8_t *)buf, len, true);
	// clear RX interrupt
	(void)RF24_setStatus(_BV(RF24_RX_DR));
	return len;
}

LOCAL void RF24_setNodeAddress(const uint8_t address)
{
	if(address!= RF24_BROADCAST_ADDRESS) {
		RF24_NODE_ADDRESS = address;
		// enable node pipe
		RF24_setPipe(_BV(RF24_ERX_P0 + RF24_BROADCAST_PIPE) | _BV(RF24_ERX_P0));
		// enable autoACK on pipe 0
		RF24_setAutoACK(_BV(RF24_ENAA_P0));
	}
}

LOCAL uint8_t RF24_getNodeID(void)
{
	return RF24_NODE_ADDRESS;
}

LOCAL bool RF24_sanityCheck(void)
{
	// detect HW defect, configuration errors or interrupted SPI line, CE disconnect cannot be detected
	return (RF24_readByteRegister(RF24_REG_RF_SETUP) == RF24_RF_SETUP) && (RF24_readByteRegister(
	            RF24_REG_RF_CH) == MY_RF24_CHANNEL);
}
LOCAL int16_t RF24_getTxPowerLevel(void)
{
	// in dBm
	return (int16_t)((-6) * (3-((RF24_readByteRegister(RF24_REG_RF_SETUP) >> 1) & 3)));
}

LOCAL uint8_t RF24_getTxPowerPercent(void)
{
	// report TX level in %, 0 (LOW) = 25%, 3 (MAX) = 100
	const uint8_t result = 25 + (((RF24_readByteRegister(RF24_REG_RF_SETUP) >> 1) & 3) * 25);
	return result;
}
LOCAL bool RF24_setTxPowerLevel(const uint8_t newPowerLevel)
{
	const uint8_t registerContent = RF24_readByteRegister(RF24_REG_RF_SETUP);
	RF24_writeByteRegister(RF24_REG_RF_SETUP, (registerContent & 0xF9) | ((newPowerLevel & 3) << 1));
	RF24_DEBUG(PSTR("RF24:STX:LEVEL=%" PRIu8 "\n"), newPowerLevel);
	return true;
}

LOCAL bool RF24_setTxPowerPercent(const uint8_t newPowerPercent)
{
	const uint8_t newPowerLevel = static_cast<uint8_t>(RF24_MIN_POWER_LEVEL + (RF24_MAX_POWER_LEVEL
	                              - RF24_MIN_POWER_LEVEL) * (newPowerPercent / 100.0f));
	return RF24_setTxPowerLevel(newPowerLevel);
}
LOCAL int16_t RF24_getSendingRSSI(void)
{
	// calculate pseudo-RSSI based on retransmission counter (ARC)
	// min -104dBm at 250kBps
	// Arbitrary definition: ARC 0 == -29, ARC 15 = -104
	return static_cast<int16_t>(-29 - (8 * (RF24_getObserveTX() & 0xF)));
}

LOCAL void RF24_enableConstantCarrierWave(void)
{
	RF24_standBy();
	RF24_setRFSetup(RF24_RF_SETUP | _BV(RF24_CONT_WAVE) | _BV(RF24_PLL_LOCK) );
	RF24_ce(HIGH);
}

LOCAL void RF24_disableConstantCarrierWave(void)
{
	RF24_ce(LOW);
	RF24_setRFSetup(RF24_RF_SETUP);
}

LOCAL bool RF24_getReceivedPowerDetector(void)
{
	// nRF24L01+ only. nRF24L01 contains a carrier detect function (same register & bit) which works
	// slightly different and takes at least 128us to become active.
	return (RF24_readByteRegister(RF24_REG_RPD) & _BV(RF24_RPD)) != 0;
}

#if defined(MY_RX_MESSAGE_BUFFER_FEATURE)
LOCAL void IRQ_HANDLER_ATTR RF24_irqHandler(void)
{
	if (RF24_receiveCallback) {
#if defined(MY_GATEWAY_SERIAL) && !defined(__linux__)
		// Will stay for a while (several 100us) in this interrupt handler. Any interrupts from serial
		// rx coming in during our stay will not be handled and will cause characters to be lost.
		// As a workaround we re-enable interrupts to allow nested processing of other interrupts.
		// Our own handler is disconnected to prevent recursive calling of this handler.
		detachInterrupt(digitalPinToInterrupt(MY_RF24_IRQ_PIN));
		interrupts();
#endif
		// Read FIFO until empty.
		// Procedure acc. to datasheet (pg. 63):
		// 1.Read payload, 2.Clear RX_DR IRQ, 3.Read FIFO_status, 4.Repeat when more data available.
		// Datasheet (ch. 8.5) states, that the nRF de-asserts IRQ after reading STATUS.

#if defined(__linux__)
		// Start checking if RX-FIFO is not empty, as we might end up here from an interrupt
		// for a message we've already read.
		if (RF24_isDataAvailable()) {
			do {
				RF24_receiveCallback();		// Must call RF24_readMessage(), which will clear RX_DR IRQ !
			} while (RF24_isDataAvailable());
		} else {
			// Occasionally interrupt is triggered but no data is available - clear RX interrupt only
			RF24_setStatus(_BV(RF24_RX_DR));
			logNotice("RF24: Recovered from a bad interrupt trigger.\n");
		}
#else
		// Start checking if RX-FIFO is not empty, as we might end up here from an interrupt
		// for a message we've already read.
		while (RF24_isDataAvailable()) {
			RF24_receiveCallback();		// Must call RF24_readMessage(), which will clear RX_DR IRQ !
		}
#endif

#if defined(MY_GATEWAY_SERIAL) && !defined(__linux__)
		// Restore our interrupt handler.
		noInterrupts();
		attachInterrupt(digitalPinToInterrupt(MY_RF24_IRQ_PIN), RF24_irqHandler, FALLING);
#endif
	} else {
		// clear RX interrupt
		RF24_setStatus(_BV(RF24_RX_DR));
	}
}

LOCAL void RF24_registerReceiveCallback(RF24_receiveCallbackType cb)
{
	MY_CRITICAL_SECTION {
		RF24_receiveCallback = cb;
	}
}
#endif

LOCAL bool RF24_initialize(void)
{
	RF24_DEBUG(PSTR("RF24:INIT:PIN,CE=%" PRIu8 ",CS=%" PRIu8 "\n"), MY_RF24_CE_PIN, MY_RF24_CS_PIN);
	// Initialize pins & HW
#if defined(MY_RF24_POWER_PIN)
	hwPinMode(MY_RF24_POWER_PIN, OUTPUT);
#endif
	RF24_powerUp();
#if defined(MY_RX_MESSAGE_BUFFER_FEATURE)
	hwPinMode(MY_RF24_IRQ_PIN,INPUT);
#endif
	hwPinMode(MY_RF24_CE_PIN, OUTPUT);
#if !defined(__linux__)
	hwPinMode(MY_RF24_CS_PIN, OUTPUT);
#endif
	RF24_ce(LOW);
	RF24_csn(HIGH);

	// Initialize SPI
	RF24_SPI.begin();
#if defined(MY_RX_MESSAGE_BUFFER_FEATURE)
	// assure SPI can be used from interrupt context
	// Note: ESP8266 & SoftSPI currently do not support interrupt usage for SPI,
	// therefore it is unsafe to use MY_RF24_IRQ_PIN with ESP8266/SoftSPI!
	RF24_SPI.usingInterrupt(digitalPinToInterrupt(MY_RF24_IRQ_PIN));
	// attach interrupt
	attachInterrupt(digitalPinToInterrupt(MY_RF24_IRQ_PIN), RF24_irqHandler, FALLING);
#endif
	// power up and standby
	RF24_standBy();
	// set address width
	RF24_setAddressWidth(MY_RF24_ADDR_WIDTH);
	// auto retransmit delay 1500us, auto retransmit count 15
	RF24_setRetries(RF24_SET_ARD, RF24_SET_ARC);
	// set channel
	RF24_setChannel(MY_RF24_CHANNEL);
	// set data rate and pa level
	RF24_setRFSetup(RF24_RF_SETUP);
	// enable ACK payload and dynamic payload
	RF24_setFeature(RF24_FEATURE);
	// sanity check (this function is P/non-P independent)
	if (!RF24_sanityCheck()) {
		RF24_DEBUG(PSTR("!RF24:INIT:SANCHK FAIL\n")); // sanity check failed, check wiring or replace module
		return false;
	}
	// enable broadcasting pipe
	RF24_setPipe(_BV(RF24_ERX_P0 + RF24_BROADCAST_PIPE));
	// disable AA on all pipes, activate when node pipe set
	RF24_setAutoACK(0x00);
	// enable dynamic payloads on used pipes
	RF24_setDynamicPayload(_BV(RF24_DPL_P0 + RF24_BROADCAST_PIPE) | _BV(RF24_DPL_P0));
	// listen to broadcast pipe
	RF24_BASE_ID[0] = RF24_BROADCAST_ADDRESS;
	RF24_setPipeAddress(RF24_REG_RX_ADDR_P0 + RF24_BROADCAST_PIPE, (uint8_t *)&RF24_BASE_ID,
	                    RF24_BROADCAST_PIPE > 1 ? 1 : MY_RF24_ADDR_WIDTH);
	// pipe 0, set full address, later only LSB is updated
	RF24_setPipeAddress(RF24_REG_RX_ADDR_P0, (uint8_t *)&RF24_BASE_ID, MY_RF24_ADDR_WIDTH);
	RF24_setPipeAddress(RF24_REG_TX_ADDR, (uint8_t *)&RF24_BASE_ID, MY_RF24_ADDR_WIDTH);
	// reset FIFO
	RF24_flushRX();
	RF24_flushTX();
	// reset interrupts
	RF24_setStatus(_BV(RF24_TX_DS) | _BV(RF24_MAX_RT) | _BV(RF24_RX_DR));
	return true;
}
