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

#include "RFM69_new.h"

// debug
#if defined(MY_DEBUG_VERBOSE_RFM69)
#define RFM69_DEBUG(x,...)	DEBUG_OUTPUT(x, ##__VA_ARGS__)	//!< Debug print
#else
#define RFM69_DEBUG(x,...)	//!< DEBUG null
#endif

rfm69_internal_t RFM69;	//!< internal variables
volatile uint8_t RFM69_irq; //!< rfm69 irq flag

#if defined(__linux__)
// SPI RX and TX buffers (max packet len + 1 byte for the command)
uint8_t RFM69_spi_rxbuff[RFM69_MAX_PACKET_LEN + 1];
uint8_t RFM69_spi_txbuff[RFM69_MAX_PACKET_LEN + 1];
#endif

LOCAL void RFM69_csn(const bool level)
{
#if defined(__linux__)
	(void)level;
#else
	hwDigitalWrite(MY_RFM69_CS_PIN, level);
#endif
}

LOCAL void RFM69_prepareSPITransaction(void)
{
#if !defined(MY_SOFTSPI) && defined(SPI_HAS_TRANSACTION)
	RFM69_SPI.beginTransaction(SPISettings(MY_RFM69_SPI_SPEED, RFM69_SPI_DATA_ORDER,
	                                       RFM69_SPI_DATA_MODE));
#endif
}

LOCAL void RFM69_concludeSPITransaction(void)
{
#if !defined(MY_SOFTSPI) && defined(SPI_HAS_TRANSACTION)
	RFM69_SPI.endTransaction();
#endif
}

LOCAL uint8_t RFM69_spiMultiByteTransfer(const uint8_t cmd, uint8_t *buf, uint8_t len,
        const bool aReadMode)
{
	uint8_t status;
	uint8_t *current = buf;

	RFM69_prepareSPITransaction();
	RFM69_csn(LOW);

#if defined(__linux__)
	uint8_t *prx = RFM69_spi_rxbuff;
	uint8_t *ptx = RFM69_spi_txbuff;
	uint8_t size = len + 1; // Add register value to transmit buffer

	*ptx++ = cmd;
	while (len--) {
		if (aReadMode) {
			*ptx++ = (uint8_t)RFM69_NOP;
		} else {
			*ptx++ = *current++;
		}
	}
	RFM69_SPI.transfernb((char *)RFM69_spi_txbuff, (char *)RFM69_spi_rxbuff, size);
	if (aReadMode) {
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
	status = RFM69_SPI.transfer(cmd);
	while (len--) {
		if (aReadMode) {
			status = RFM69_SPI.transfer((uint8_t)RFM69_NOP);
			if (buf != NULL) {
				*current++ = status;
			}
		} else {
			status = RFM69_SPI.transfer(*current++);
		}
	}
#endif
	RFM69_csn(HIGH);
	RFM69_concludeSPITransaction();
	return status;
}

// low level register access
LOCAL inline uint8_t RFM69_RAW_readByteRegister(const uint8_t address)
{
	return RFM69_spiMultiByteTransfer(address, NULL, 1, true);
}

LOCAL inline uint8_t RFM69_RAW_writeByteRegister(const uint8_t address, uint8_t value)
{
	return RFM69_spiMultiByteTransfer(address, &value, 1, false);
}

// helper functions
LOCAL inline uint8_t RFM69_readReg(const uint8_t reg)
{
	return RFM69_RAW_readByteRegister(reg & RFM69_READ_REGISTER);
}

LOCAL inline uint8_t RFM69_writeReg(const uint8_t reg, const uint8_t value)
{
	return RFM69_RAW_writeByteRegister(reg | RFM69_WRITE_REGISTER, value);
}

LOCAL inline uint8_t RFM69_burstReadReg(const uint8_t reg, void *buf, uint8_t len)
{
	return RFM69_spiMultiByteTransfer(reg & RFM69_READ_REGISTER, (uint8_t *)buf, len, true);
}

LOCAL inline uint8_t RFM69_burstWriteReg(const uint8_t reg, const void *buf, uint8_t len)
{
	return RFM69_spiMultiByteTransfer(reg | RFM69_WRITE_REGISTER, (uint8_t *)buf, len, false);
}

LOCAL inline rfm69_RSSI_t RFM69_RSSItoInternal(const int16_t externalRSSI)
{
	return (rfm69_RSSI_t)-(externalRSSI * 2);
}

LOCAL inline int16_t RFM69_internalToRSSI(const rfm69_RSSI_t internalRSSI)
{
	return (int16_t)-(internalRSSI / 2);
}

LOCAL bool RFM69_initialise(const uint32_t frequencyHz)
{
	RFM69_DEBUG(PSTR("RFM69:INIT\n"));
	// power up radio if power pin defined
#if defined(MY_RFM69_POWER_PIN)
	hwPinMode(MY_RFM69_POWER_PIN, OUTPUT);
#endif
	RFM69_powerUp();
	// reset radio module if rst pin defined
#if defined(MY_RFM69_RST_PIN)
	hwPinMode(MY_RFM69_RST_PIN, OUTPUT);
	hwDigitalWrite(MY_RFM69_RST_PIN, HIGH);
	// 100uS
	delayMicroseconds(100);
	hwDigitalWrite(MY_RFM69_RST_PIN, LOW);
	// wait until chip ready
	delay(5);
	RFM69_DEBUG(PSTR("RFM69:INIT:PIN,CS=%" PRIu8 ",IQP=%" PRIu8 ",IQN=%" PRIu8 ",RST=%" PRIu8 "\n"),
	            MY_RFM69_CS_PIN,MY_RFM69_IRQ_PIN,
	            MY_RFM69_IRQ_NUM,MY_RFM69_RST_PIN);
#else
	RFM69_DEBUG(PSTR("RFM69:INIT:PIN,CS=%" PRIu8 ",IQP=%" PRIu8 ",IQN=%" PRIu8 "\n"),MY_RFM69_CS_PIN,
	            MY_RFM69_IRQ_PIN,
	            MY_RFM69_IRQ_NUM);
#endif

	// set variables
	RFM69.address = RFM69_BROADCAST_ADDRESS;
	RFM69.dataReceived = false;
	RFM69.ackReceived = false;
	RFM69.txSequenceNumber = 0;	// initialise TX sequence counter
	RFM69.powerLevel = MY_RFM69_TX_POWER_DBM + 1;	// will be overwritten when set
	RFM69.radioMode = RFM69_RADIO_MODE_SLEEP;
	RFM69.ATCenabled = false;
	RFM69.ATCtargetRSSI = RFM69_RSSItoInternal(MY_RFM69_ATC_TARGET_RSSI_DBM);

	// SPI init
#if !defined(__linux__)
	hwDigitalWrite(MY_RFM69_CS_PIN, HIGH);
	hwPinMode(MY_RFM69_CS_PIN, OUTPUT);
#endif
	RFM69_SPI.begin();
	(void)RFM69_setRadioMode(RFM69_RADIO_MODE_STDBY);
	// set configuration, encryption is disabled
	RFM69_setConfiguration();
	RFM69_setFrequency(frequencyHz);
	(void)RFM69_setTxPowerLevel(MY_RFM69_TX_POWER_DBM);

#if defined(MY_DEBUG_VERBOSE_RFM69_REGISTERS)
	RFM69_readAllRegs();
#else
	(void)RFM69_readAllRegs;
#endif
	//RFM69_DEBUG(PSTR("RFM69:INIT:HWV=%" PRIu8 "\n"),RFM69_readReg(RFM69_REG_VERSION));

	if (!RFM69_sanityCheck()) {
		// sanity check failed, check wiring or replace module
		RFM69_DEBUG(PSTR("!RFM69:INIT:SANCHK FAIL\n"));
		return false;
	}
	// IRQ
	RFM69_irq = false;
	hwPinMode(MY_RFM69_IRQ_PIN, INPUT);
	attachInterrupt(MY_RFM69_IRQ_NUM, RFM69_interruptHandler, RISING);
	return true;
}

LOCAL void RFM69_clearFIFO(void)
{
	(void)RFM69_writeReg(RFM69_REG_IRQFLAGS2, RFM69_IRQFLAGS2_FIFOOVERRUN);
}
// IRQ handler: PayloadReady (RX) & PacketSent (TX) mapped to DI0
LOCAL void IRQ_HANDLER_ATTR RFM69_interruptHandler(void)
{
	// set flag
	RFM69_irq = true;
}

LOCAL void RFM69_interruptHandling(void)
{
	const uint8_t regIrqFlags2 = RFM69_readReg(RFM69_REG_IRQFLAGS2);
	if (RFM69.radioMode == RFM69_RADIO_MODE_RX && (regIrqFlags2 & RFM69_IRQFLAGS2_PAYLOADREADY)) {
		(void)RFM69_setRadioMode(RFM69_RADIO_MODE_STDBY);
		// use the fifo level irq as indicator if header bytes received
		if (regIrqFlags2 & RFM69_IRQFLAGS2_FIFOLEVEL) {
			RFM69_prepareSPITransaction();
			RFM69_csn(LOW);
#if defined(__linux__)
			char data[RFM69_MAX_PACKET_LEN + 1];   // max packet len + 1 byte for the command
			data[0] = RFM69_REG_FIFO & RFM69_READ_REGISTER;
			RFM69_SPI.transfern(data, 3);

			RFM69.currentPacket.header.packetLen = data[1];
			RFM69.currentPacket.header.recipient = data[2];

			if (RFM69.currentPacket.header.packetLen > RFM69_MAX_PACKET_LEN) {
				RFM69.currentPacket.header.packetLen = RFM69_MAX_PACKET_LEN;
			}

			data[0] = RFM69_REG_FIFO & RFM69_READ_REGISTER;
			//SPI.transfern(data, RFM69.currentPacket.header.packetLen - 1); //TODO: Wrong packetLen?
			RFM69_SPI.transfern(data, RFM69.currentPacket.header.packetLen);

			//(void)memcpy((void *)&RFM69.currentPacket.data[2], (void *)&data[1], RFM69.currentPacket.header.packetLen - 2);   //TODO: Wrong packetLen?
			(void)memcpy((void *)&RFM69.currentPacket.data[2], (void *)&data[1],
			             RFM69.currentPacket.header.packetLen - 1);

			if (RFM69.currentPacket.header.version >= RFM69_MIN_PACKET_HEADER_VERSION) {
				RFM69.currentPacket.payloadLen = min(RFM69.currentPacket.header.packetLen - (RFM69_HEADER_LEN - 1),
				                                     RFM69_MAX_PACKET_LEN);
				RFM69.ackReceived = RFM69_getACKReceived(RFM69.currentPacket.header.controlFlags);
				RFM69.dataReceived = !RFM69.ackReceived;
			}
#else
			(void)RFM69_SPI.transfer(RFM69_REG_FIFO & RFM69_READ_REGISTER);
			// set reading pointer
			uint8_t *current = (uint8_t *)&RFM69.currentPacket;
			bool headerRead = false;
			// first read header
			uint8_t readingLength = RFM69_HEADER_LEN;
			while (readingLength--) {
				*current++ = RFM69_SPI.transfer((uint8_t)RFM69_NOP);
				if (!readingLength && !headerRead) {
					// header read
					headerRead = true;
					if (RFM69.currentPacket.header.version >= RFM69_MIN_PACKET_HEADER_VERSION) {
						// read payload
						readingLength = min(RFM69.currentPacket.header.packetLen - (RFM69_HEADER_LEN - 1),
						                    RFM69_MAX_PACKET_LEN);
						// save payload length
						RFM69.currentPacket.payloadLen = readingLength;
						RFM69.ackReceived = RFM69_getACKReceived(RFM69.currentPacket.header.controlFlags);
						RFM69.dataReceived = !RFM69.ackReceived;
					}
				}
			}
#endif
			RFM69_csn(HIGH);
			RFM69_concludeSPITransaction();
		}
		RFM69.currentPacket.RSSI = RFM69_readRSSI();
		// radio remains in stdby until packet read
	} else {
		// back to RX
		(void)RFM69_setRadioMode(RFM69_RADIO_MODE_RX);
	}
}

LOCAL void RFM69_handler(void)
{
	if (RFM69_irq) {
		// radio is in STDBY
		// clear flag, 8bit - no need for critical section
		RFM69_irq = false;
		RFM69_interruptHandling();
	}
}

LOCAL bool RFM69_available(void)
{
	if (RFM69.dataReceived) {
		// data received - we are still in STDBY
		return true;
	} else if (RFM69.radioMode == RFM69_RADIO_MODE_TX) {
		// still in TX
		return false;
	} else if (RFM69.radioMode != RFM69_RADIO_MODE_RX) { // adding this check speeds up loop() :)
		// no data received and not in RX
		(void)RFM69_setRadioMode(RFM69_RADIO_MODE_RX);
	}
	return false;
}

LOCAL uint8_t RFM69_receive(uint8_t *buf, const uint8_t maxBufSize)
{
	const uint8_t payloadLen = min(RFM69.currentPacket.payloadLen, maxBufSize);
	const uint8_t sender = RFM69.currentPacket.header.sender;
	const rfm69_sequenceNumber_t sequenceNumber = RFM69.currentPacket.header.sequenceNumber;
	const uint8_t controlFlags = RFM69.currentPacket.header.controlFlags;
	const rfm69_RSSI_t RSSI = RFM69.currentPacket.RSSI;

	if (buf != NULL) {
		(void)memcpy((void *)buf, (void *)&RFM69.currentPacket.payload, payloadLen);
	}
	// clear data flag
	RFM69.dataReceived = false;
	if (RFM69_getACKRequested(controlFlags) && !RFM69_getACKReceived(controlFlags)) {
#if defined(MY_GATEWAY_FEATURE) && (F_CPU>16*1000000ul)
		// delay for fast GW and slow nodes
		delay(50);
#endif
		RFM69_sendACK(sender, sequenceNumber, RSSI);
	}
	return payloadLen;
}

LOCAL bool RFM69_channelFree(void)
{
	// returns true if channel activity under RFM69_CSMA_LIMIT_DBM
	const rfm69_RSSI_t RSSI = RFM69_readRSSI(false);
	RFM69_DEBUG(PSTR("RFM69:CSMA:RSSI=%" PRIi16 "\n"), RFM69_internalToRSSI(RSSI));
	return (RSSI > RFM69_RSSItoInternal(MY_RFM69_CSMA_LIMIT_DBM));
}

LOCAL bool RFM69_sendFrame(rfm69_packet_t *packet, const bool increaseSequenceCounter)
{
	// ensure we are in RX for correct RSSI sampling, dirty hack to enforce rx restart :)
	RFM69.radioMode = RFM69_RADIO_MODE_STDBY;
	(void)RFM69_setRadioMode(RFM69_RADIO_MODE_RX);
	delay(1); // timing for correct RSSI sampling
	const uint32_t CSMA_START_MS = hwMillis();
	while (!RFM69_channelFree() &&
	        ((hwMillis() - CSMA_START_MS) < MY_RFM69_CSMA_TIMEOUT_MS)) {
		doYield();
	}
	// set radio to standby to load fifo
	(void)RFM69_setRadioMode(RFM69_RADIO_MODE_STDBY);
	if (increaseSequenceCounter) {
		// increase sequence counter, overflow is ok
		RFM69.txSequenceNumber++;
	}
	// clear FIFO and flags
	RFM69_clearFIFO();
	// assign sequence number
	packet->header.sequenceNumber = RFM69.txSequenceNumber;

	// write packet
	const uint8_t finalLen = packet->payloadLen + RFM69_HEADER_LEN; // including length byte
	(void)RFM69_burstWriteReg(RFM69_REG_FIFO, packet->data, finalLen);

	// send message
	(void)RFM69_setRadioMode(RFM69_RADIO_MODE_TX); // irq upon txsent
	const uint32_t txStartMS = hwMillis();
	while (!RFM69_irq && (hwMillis() - txStartMS < MY_RFM69_TX_TIMEOUT_MS)) {
		doYield();
	};
	return RFM69_irq;
}

LOCAL bool RFM69_send(const uint8_t recipient, uint8_t *data, const uint8_t len,
                      const rfm69_controlFlags_t flags, const bool increaseSequenceCounter)
{
	// assemble packet
	rfm69_packet_t packet;
	packet.header.version = RFM69_PACKET_HEADER_VERSION;
	packet.header.sender = RFM69.address;
	packet.header.recipient = recipient;
	packet.payloadLen = min(len, (uint8_t)RFM69_MAX_PAYLOAD_LEN);
	packet.header.controlFlags = flags;
	(void)memcpy((void *)&packet.payload, (void *)data, packet.payloadLen); // copy payload
	packet.header.packetLen = packet.payloadLen + (RFM69_HEADER_LEN - 1); // -1 length byte
	return RFM69_sendFrame(&packet, increaseSequenceCounter);
}

LOCAL uint32_t RFM69_getFrequency(void)
{
	uint32_t freqHz;
	freqHz = (uint32_t)RFM69_readReg(RFM69_REG_FRFMSB) << 16;
	freqHz |= RFM69_readReg(RFM69_REG_FRFMID) << 8;
	freqHz |= RFM69_readReg(RFM69_REG_FRFLSB);
	freqHz *= RFM69_FSTEP;
	return freqHz;
}

LOCAL void RFM69_setFrequency(const uint32_t frequencyHz)
{
	const uint32_t freqHz = (uint32_t)(frequencyHz / RFM69_FSTEP);
	RFM69_writeReg(RFM69_REG_FRFMSB, (uint8_t)((freqHz >> 16) & 0xFF));
	RFM69_writeReg(RFM69_REG_FRFMID, (uint8_t)((freqHz >> 8) & 0xFF));
	RFM69_writeReg(RFM69_REG_FRFLSB, (uint8_t)(freqHz & 0xFF));
}

LOCAL void RFM69_setHighPowerRegs(const bool onOff)
{
#if defined(RFM69_VERSION_HW)
	RFM69_writeReg(RFM69_REG_OCP, (onOff ? RFM69_OCP_OFF : RFM69_OCP_ON ) | RFM69_OCP_TRIM_95);
	RFM69_writeReg(RFM69_REG_TESTPA1, onOff ? 0x5D : 0x55);
	RFM69_writeReg(RFM69_REG_TESTPA2, onOff ? 0x7C : 0x70);
#else
	(void)onOff;
#endif
}

LOCAL bool RFM69_setTxPowerLevel(rfm69_powerlevel_t newPowerLevel)
{
	// limit power levels
	newPowerLevel = max((rfm69_powerlevel_t)RFM69_MIN_POWER_LEVEL_DBM, newPowerLevel);
	newPowerLevel = min((rfm69_powerlevel_t)RFM69_MAX_POWER_LEVEL_DBM, newPowerLevel);

	if (RFM69.powerLevel == newPowerLevel) {
		RFM69_DEBUG(PSTR("RFM69:PTX:NO ADJ\n"));
		return false;
	}

	RFM69.powerLevel = newPowerLevel;
	uint8_t palevel;
#if !defined(RFM69_VERSION_HW)
	// -18dBm to +13dBm, PA0, offset 0, i.e. -18dBm = level 0
	palevel = RFM69_PALEVEL_PA0_ON | ((uint8_t)(newPowerLevel + 18));

#else
	if (newPowerLevel <= (rfm69_powerlevel_t)13) {
		// -2dBm to +13dBm, PA1, offset 16, i.e. -2dBm = level 16
		palevel = RFM69_PALEVEL_PA1_ON | ((uint8_t)(newPowerLevel + 18));
	} else if (newPowerLevel >= (rfm69_powerlevel_t)RFM69_HIGH_POWER_DBM) {
		// +18dBm to +20dBm, PA1 and PA2, Boost settings, 18dBm = level 29
		palevel = RFM69_PALEVEL_PA1_ON | RFM69_PALEVEL_PA2_ON | ((uint8_t)(newPowerLevel + 11));
	} else {
		// +14dBm to +17dBm, PA1 and PA2
		palevel = RFM69_PALEVEL_PA1_ON | RFM69_PALEVEL_PA2_ON | ((uint8_t)(newPowerLevel + 14));
	}
#endif
	RFM69_writeReg(RFM69_REG_PALEVEL, palevel);
	RFM69_DEBUG(PSTR("RFM69:PTX:LEVEL=%" PRIi8 " dBm\n"),newPowerLevel);
	return true;
}

LOCAL void RFM69_setAddress(const uint8_t addr)
{
	(void)RFM69_setRadioMode(RFM69_RADIO_MODE_STDBY);
	RFM69.address = addr;
	RFM69_writeReg(RFM69_REG_NODEADRS, addr);
}

LOCAL uint8_t RFM69_getAddress(void)
{
	return RFM69.address;
}

LOCAL bool RFM69_setRadioMode(const rfm69_radio_mode_t newRadioMode)
{
	if (RFM69.radioMode == newRadioMode) {
		// no change
		return false;
	}

	uint8_t regMode;

	if (newRadioMode == RFM69_RADIO_MODE_STDBY) {
		regMode = RFM69_OPMODE_SEQUENCER_ON | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_STANDBY;
	} else if (newRadioMode == RFM69_RADIO_MODE_SLEEP) {
		regMode = RFM69_OPMODE_SEQUENCER_OFF | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_SLEEP;
	} else if (newRadioMode == RFM69_RADIO_MODE_RX) {
		RFM69.dataReceived = false;
		RFM69.ackReceived = false;
		regMode = RFM69_OPMODE_SEQUENCER_ON | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_RECEIVER;
		RFM69_writeReg(RFM69_REG_DIOMAPPING1, RFM69_DIOMAPPING1_DIO0_01); // Interrupt on PayloadReady, DIO0
		// disable high power settings
		RFM69_setHighPowerRegs(false);
		RFM69_writeReg(RFM69_REG_PACKETCONFIG2,
		               (RFM69_readReg(RFM69_REG_PACKETCONFIG2) & 0xFB) | RFM69_PACKET2_RXRESTART); // avoid RX deadlocks
	} else if (newRadioMode == RFM69_RADIO_MODE_TX) {
		regMode = RFM69_OPMODE_SEQUENCER_ON | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_TRANSMITTER;
		RFM69_writeReg(RFM69_REG_DIOMAPPING1, RFM69_DIOMAPPING1_DIO0_00); // Interrupt on PacketSent, DIO0
		RFM69_setHighPowerRegs(RFM69.powerLevel >= (rfm69_powerlevel_t)RFM69_HIGH_POWER_DBM);
	} else if (newRadioMode == RFM69_RADIO_MODE_SYNTH) {
		regMode = RFM69_OPMODE_SEQUENCER_ON | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_SYNTHESIZER;
	} else {
		regMode = RFM69_OPMODE_SEQUENCER_ON | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_STANDBY;
	}

	// set new mode
	RFM69_writeReg(RFM69_REG_OPMODE, regMode);

	// Waking from sleep mode may take longer
	if (RFM69.radioMode == RFM69_RADIO_MODE_SLEEP) {
		// wait for ModeReady
		if (!RFM69_isModeReady()) {
			return false;
		}
	}
	RFM69.radioMode = newRadioMode;
	return true;
}

LOCAL void RFM69_powerUp(void)
{
#if defined(MY_RFM69_POWER_PIN)
	RFM69_DEBUG(PSTR("RFM69:PWU\n"));	// power up radio
	hwDigitalWrite(MY_RFM69_POWER_PIN, HIGH);
	delay(RFM69_POWERUP_DELAY_MS);
#endif
}
LOCAL void RFM69_powerDown(void)
{
#if defined(MY_RFM69_POWER_PIN)
	RFM69_DEBUG(PSTR("RFM69:PWD\n"));	// power down radio
	hwDigitalWrite(MY_RFM69_POWER_PIN, LOW);
#endif
}

LOCAL bool RFM69_sleep(void)
{
	RFM69_DEBUG(PSTR("RFM69:RSL\n"));	// put radio to sleep
	return RFM69_setRadioMode(RFM69_RADIO_MODE_SLEEP);
}

LOCAL bool RFM69_standBy(void)
{
	RFM69_DEBUG(PSTR("RFM69:RSB\n"));	// put radio to standby
	return RFM69_setRadioMode(RFM69_RADIO_MODE_STDBY);
}


// should be called immediately after reception in case sender wants ACK
LOCAL void RFM69_sendACK(const uint8_t recipient, const rfm69_sequenceNumber_t sequenceNumber,
                         const rfm69_RSSI_t RSSI)
{
	RFM69_DEBUG(PSTR("RFM69:SAC:SEND ACK,TO=%" PRIu8 ",RSSI=%" PRIi16 "\n"),recipient,
	            RFM69_internalToRSSI(RSSI));
	rfm69_ack_t ACK;
	ACK.sequenceNumber = sequenceNumber;
	ACK.RSSI = RSSI;
	rfm69_controlFlags_t flags = 0u;	// reset flags
	RFM69_setACKReceived(flags, true);
	RFM69_setACKRSSIReport(flags, true);
	(void)RFM69_send(recipient, (uint8_t *)&ACK, sizeof(rfm69_ack_t), flags);
}

LOCAL bool RFM69_executeATC(const rfm69_RSSI_t currentRSSI, const rfm69_RSSI_t targetRSSI)
{
	// RSSI range -80..-70 = internal representation 160(l)..140(u)
	rfm69_powerlevel_t newPowerLevel = RFM69.powerLevel;
	const rfm69_RSSI_t uRange = targetRSSI - RFM69_RSSItoInternal(RFM69_ATC_TARGET_RANGE_DBM);
	const rfm69_RSSI_t lRange = targetRSSI + RFM69_RSSItoInternal(RFM69_ATC_TARGET_RANGE_DBM);
	if (currentRSSI > lRange && newPowerLevel < RFM69_MAX_POWER_LEVEL_DBM) {
		// increase transmitter power
		newPowerLevel++;
	} else if (currentRSSI < uRange && newPowerLevel > RFM69_MIN_POWER_LEVEL_DBM) {
		// decrease transmitter power
		newPowerLevel--;
	} else {
		// nothing to adjust
		return false;
	}
	RFM69_DEBUG(PSTR("RFM69:ATC:ADJ TXL,cR=%" PRIi16 ",tR=%" PRIi16 "..%" PRIi16 ",TXL=%" PRIi8 "\n"),
	            RFM69_internalToRSSI(currentRSSI), RFM69_internalToRSSI(lRange), RFM69_internalToRSSI(uRange),
	            RFM69.powerLevel);
	return RFM69_setTxPowerLevel(newPowerLevel);
}

LOCAL void RFM69_ATCmode(const bool onOff, const int16_t targetRSSI)
{
	RFM69.ATCenabled = onOff;
	RFM69.ATCtargetRSSI = RFM69_RSSItoInternal(targetRSSI);
}


LOCAL bool RFM69_sendWithRetry(const uint8_t recipient, const void *buffer,
                               const uint8_t bufferSize, const bool noACK)
{
	for (uint8_t retry = 0; retry < RFM69_RETRIES; retry++) {
		RFM69_DEBUG(PSTR("RFM69:SWR:SEND,TO=%" PRIu8 ",SEQ=%" PRIu16 ",RETRY=%" PRIu8 "\n"), recipient,
		            RFM69.txSequenceNumber,retry);
		rfm69_controlFlags_t flags = 0u; // reset all flags
		RFM69_setACKRequested(flags, !noACK);
		RFM69_setACKRSSIReport(flags, RFM69.ATCenabled);
		(void)RFM69_send(recipient, (uint8_t *)buffer, bufferSize, flags, !retry);
		if (noACK) {
			// no ACK requested
			return true;
		}
		// radio is in RX
		const uint32_t enterMS = hwMillis();
		while (hwMillis() - enterMS < RFM69_RETRY_TIMEOUT_MS && !RFM69.dataReceived) {
			RFM69_handler();
			if (RFM69.ackReceived) {
				// radio is in stdby
				const uint8_t ACKsender = RFM69.currentPacket.header.sender;
				const rfm69_sequenceNumber_t ACKsequenceNumber = RFM69.currentPacket.ACK.sequenceNumber;
				const rfm69_controlFlags_t ACKflags = RFM69.currentPacket.header.controlFlags;
				const rfm69_RSSI_t ACKRSSI = RFM69.currentPacket.ACK.RSSI;
				RFM69.ackReceived = false;
				// packet read, back to RX
				RFM69_setRadioMode(RFM69_RADIO_MODE_RX);
				if (ACKsender == recipient && ACKsequenceNumber == RFM69.txSequenceNumber) {
					RFM69_DEBUG(PSTR("RFM69:SWR:ACK,FROM=%" PRIu8 ",SEQ=%" PRIu8 ",RSSI=%" PRIi16 "\n"), ACKsender,
					            ACKsequenceNumber,
					            RFM69_internalToRSSI(ACKRSSI));

					// ATC
					if (RFM69.ATCenabled && RFM69_getACKRSSIReport(ACKflags)) {
						(void)RFM69_executeATC(ACKRSSI, RFM69.ATCtargetRSSI);
					}
					return true;
				} // seq check
			}
		}
		RFM69_DEBUG(PSTR("!RFM69:SWR:NACK\n"));
	}
	return false;
}

LOCAL int16_t RFM69_getSendingRSSI(void)
{
	// own RSSI, as measured by the recipient - ACK part
	if (RFM69_getACKRSSIReport(RFM69.currentPacket.header.controlFlags)) {
		return RFM69_internalToRSSI(RFM69.currentPacket.ACK.RSSI);
	} else {
		// not valid
		return 127;
	}
}

LOCAL int16_t RFM69_getReceivingRSSI(void)
{
	// RSSI from sender
	return RFM69_internalToRSSI(RFM69.currentPacket.RSSI);
}

LOCAL bool RFM69_setTxPowerPercent(uint8_t newPowerPercent)
{
	newPowerPercent = min(newPowerPercent, (uint8_t)100);	// limit
	const rfm69_powerlevel_t newPowerLevel = static_cast<rfm69_powerlevel_t>
	        (RFM69_MIN_POWER_LEVEL_DBM + (RFM69_MAX_POWER_LEVEL_DBM
	                                      - RFM69_MIN_POWER_LEVEL_DBM) * (newPowerPercent / 100.0f));
	RFM69_DEBUG(PSTR("RFM69:SPP:PCT=%" PRIu8 ",TX LEVEL=%" PRIi8 "\n"), newPowerPercent,newPowerLevel);
	return RFM69_setTxPowerLevel(newPowerLevel);
}



LOCAL rfm69_powerlevel_t RFM69_getTxPowerLevel(void)
{
	// report TX level in dBm
	return RFM69.powerLevel;
}

LOCAL uint8_t RFM69_getTxPowerPercent(void)
{
	// report TX level in %
	const uint8_t result = static_cast<uint8_t>(100.0f * (RFM69.powerLevel -
	                       RFM69_MIN_POWER_LEVEL_DBM) /
	                       (RFM69_MAX_POWER_LEVEL_DBM
	                        - RFM69_MIN_POWER_LEVEL_DBM));
	return result;
}

LOCAL bool RFM69_sanityCheck(void)
{
	bool result = true; // default
	result &= RFM69_readReg(RFM69_REG_RSSITHRESH) == RFM69_RSSITHRESH_VALUE;
	result &= RFM69_readReg(RFM69_REG_SYNCVALUE1) == RFM69_SYNCVALUE1;
	result &= RFM69_readReg(RFM69_REG_SYNCVALUE2) == MY_RFM69_NETWORKID;
	return result;
}

LOCAL void RFM69_setConfiguration(void)
{
	const uint8_t rfm69_modem_config[] = { MY_RFM69_MODEM_CONFIGURATION };
	const uint8_t CONFIG[][2] = {
		{ RFM69_REG_OPMODE, RFM69_OPMODE_SEQUENCER_ON | RFM69_OPMODE_LISTEN_OFF | RFM69_OPMODE_STANDBY },
		{ RFM69_REG_DATAMODUL, rfm69_modem_config[0] },
		{ RFM69_REG_BITRATEMSB, rfm69_modem_config[1] },
		{ RFM69_REG_BITRATELSB, rfm69_modem_config[2] },
		{ RFM69_REG_FDEVMSB, rfm69_modem_config[3] },
		{ RFM69_REG_FDEVLSB, rfm69_modem_config[4] },
		{ RFM69_REG_LNA, RFM69_LNA_ZIN_200 | RFM69_LNA_CURRENTGAIN },
		{ RFM69_REG_RXBW, rfm69_modem_config[5] },
		{ RFM69_REG_AFCBW, rfm69_modem_config[5] }, // same as rxbw, experimental, based on datasheet
		//{ RFM69_REG_DIOMAPPING1, RFM69_DIOMAPPING1_DIO0_01 },
		{ RFM69_REG_DIOMAPPING2, RFM69_DIOMAPPING2_CLKOUT_OFF },
		{ RFM69_REG_IRQFLAGS2, RFM69_IRQFLAGS2_FIFOOVERRUN },		// clear FIFO and flags
		{ RFM69_REG_RSSITHRESH, RFM69_RSSITHRESH_VALUE },
		{ RFM69_REG_PREAMBLEMSB, RFM69_PREAMBLESIZE_MSB_VALUE },
		{ RFM69_REG_PREAMBLELSB, RFM69_PREAMBLESIZE_LSB_VALUE },
		{ RFM69_REG_SYNCCONFIG, RFM69_SYNC_ON | RFM69_SYNC_FIFOFILL_AUTO | RFM69_SYNC_SIZE_2 | RFM69_SYNC_TOL_0 },
		{ RFM69_REG_SYNCVALUE1, RFM69_SYNCVALUE1 },
		{ RFM69_REG_SYNCVALUE2, MY_RFM69_NETWORKID },
		{ RFM69_REG_PACKETCONFIG1, rfm69_modem_config[6] },
		{ RFM69_REG_PAYLOADLENGTH, RFM69_MAX_PACKET_LEN }, // in variable length mode: the max frame size, not used in TX
		{ RFM69_REG_NODEADRS, RFM69_BROADCAST_ADDRESS },	// init
		{ RFM69_REG_BROADCASTADRS, RFM69_BROADCAST_ADDRESS },
		{ RFM69_REG_FIFOTHRESH, RFM69_FIFOTHRESH_TXSTART_FIFOTHRESH | (RFM69_HEADER_LEN - 1) },	// start transmitting when rfm69 header loaded, fifo level irq when header bytes received (irq asserted when n bytes exceeded)
		{ RFM69_REG_PACKETCONFIG2, RFM69_PACKET2_RXRESTARTDELAY_2BITS | RFM69_PACKET2_AUTORXRESTART_OFF | RFM69_PACKET2_AES_OFF },
		{ RFM69_REG_TESTDAGC, RFM69_DAGC_IMPROVED_LOWBETA0 }, // continuous DAGC mode, use 0x30 if afc offset == 0
		{ 255, 0}
	};
	for (uint8_t i = 0; CONFIG[i][0] != 255; i++) {
		RFM69_writeReg(CONFIG[i][0], CONFIG[i][1]);
	}
}

LOCAL bool RFM69_isModeReady(void)
{
	uint16_t timeout = 0xFFFF;
	while (!(RFM69_readReg(RFM69_REG_IRQFLAGS1) & RFM69_IRQFLAGS1_MODEREADY) && timeout--) {
	};
	return (bool)timeout;
}

LOCAL void RFM69_encrypt(const char *key)
{
	(void)RFM69_setRadioMode(RFM69_RADIO_MODE_STDBY);
	if (key != NULL) {
		RFM69_burstWriteReg(RFM69_REG_AESKEY1, key, 16);
	}
	RFM69_writeReg(RFM69_REG_PACKETCONFIG2,
	               (RFM69_readReg(RFM69_REG_PACKETCONFIG2) & 0xFE) | (key ? RFM69_PACKET2_AES_ON :
	                       RFM69_PACKET2_AES_OFF));
}

LOCAL rfm69_RSSI_t RFM69_readRSSI(const bool forceTrigger)
{
	// RssiStart command and RssiDone flags are not usable when DAGC is turned on
	/*if (forceTrigger) {
		RFM69_writeReg(RFM69_REG_RSSICONFIG, RFM69_RSSI_START);
		uint16_t timeout = 0xFFFF;
		while (!(RFM69_readReg(RFM69_REG_RSSICONFIG) & RFM69_RSSI_DONE) && timeout--) {
		};
	}
	*/
	(void)forceTrigger;
	return (rfm69_RSSI_t)RFM69_readReg(RFM69_REG_RSSIVALUE);
}

LOCAL void RFM69_readAllRegs(void)
{
#ifdef RFM69_REGISTER_DETAIL
	int16_t capVal;

	//... State Variables for intelligent decoding
	uint8_t modeFSK = 0;
	int16_t bitRate = 0;
	int16_t freqDev = 0;
	int32_t freqCenter = 0;
#endif

	RFM69_DEBUG(PSTR("RFM69:DUMP:Registers Address | HEX value \n"));

	for (uint8_t regAddr = 1; regAddr <= 0x4F; regAddr++) {
		uint8_t regVal = RFM69_readReg(regAddr);
		RFM69_DEBUG(PSTR("RFM69:DUMP:REG=0x%02x Value=0x%02x\n"), regAddr, regVal);
#ifdef RFM69_REGISTER_DETAIL
		switch (regAddr) {
		case 0x1: {
			RFM69_DEBUG(PSTR("RFM69:DUMP:REG=0x%02x Controls the automatic Sequencer(see section 4.2)\n"),
			            regAddr);

			if (0x80 & regVal) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:SequencerOff : 1 -> Mode is forced by the user\n"));
			} else {
				RFM69_DEBUG(
				    PSTR("RFM69:DUMP:SequencerOff : 0 -> Operating mode as selected with Mode bits in RegOpMode is automatically reached with the Sequencer\n"));
			}

			RFM69_DEBUG(PSTR("RFM69:DUMP:Enables Listen mode, should be enabled whilst in Standby mode\n"));
			if (0x40 & regVal) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:ListenOn : 1 -> On\n"));
			} else {
				RFM69_DEBUG(PSTR("RFM69:DUMP:ListenOn : 0->Off(see section 4.3)\n"));
			}

			RFM69_DEBUG(
			    PSTR("RFM69:DUMP:Aborts Listen mode when set together with ListenOn=0 See section 4.3.4 for details (Always reads 0.)\n"));
			if (0x20 & regVal) {
				RFM69_DEBUG(
				    PSTR("RFM69:DUMP:ERROR - ListenAbort should NEVER return 1 this is a write only register\n"));
			}

			RFM69_DEBUG(PSTR("RFM69:DUMP:Transceiver's operating modes\n"));
			capVal = (regVal >> 2) & 0x7;
			if (capVal == 0b000) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:Mode : 000 -> Sleep mode (SLEEP)\n"));
			} else if (capVal == 0b001) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:Mode : 001 -> Standby mode (STDBY)\n"));
			} else if (capVal == 0b010) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:Mode : 010 -> Frequency Synthesizer mode (FS)\n"));
			} else if (capVal == 0b011) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:Mode : 011 -> Transmitter mode (TX)\n"));
			} else if (capVal == 0b100) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:Mode : 100 -> Receiver Mode (RX)\n"));
			} else {
				RFM69_DEBUG(PSTR("RFM69:DUMP:Mode : %d capVal \n"), capVal);

			}
			break;
		}

		case 0x2: {
			RFM69_DEBUG(PSTR("RFM69:DUMP:REG=0x%02x Data Processing mode \n"), regAddr);

			capVal = (regVal >> 5) & 0x3;
			if (capVal == 0b00) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:DataMode : 00 -> Packet mode\n"));
			} else if (capVal == 0b01) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:DataMode : 01 -> reserved\n"));
			} else if (capVal == 0b10) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:DataMode : 10 -> Continuous mode with bit synchronizer\n"));
			} else if (capVal == 0b11) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:DataMode : 11 -> Continuous mode without bit synchronizer\n"));
			}

			RFM69_DEBUG(PSTR("RFM69:DUMP:Modulation scheme\n"));
			capVal = (regVal >> 3) & 0x3;
			if (capVal == 0b00) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:Modulation Type : 00 -> FSK\n"));
				modeFSK = 1;
			} else if (capVal == 0b01) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:Modulation Type : 01 -> OOK\n"));
			} else if (capVal == 0b10) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:Modulation Type : 10 -> reserved\n"));
			} else if (capVal == 0b11) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:Modulation Type : 11 -> reserved\n"));
			}

			if (modeFSK) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:Data shaping : in FSK\n"));
			} else {
				RFM69_DEBUG(PSTR("RFM69:DUMP:Data shaping : in OOK\n"));
			}

			capVal = regVal & 0x3;
			if (modeFSK) {
				if (capVal == 0b00) {
					RFM69_DEBUG(PSTR("RFM69:DUMP:ModulationShaping : 00 -> no shaping\n"));
				} else if (capVal == 0b01) {
					RFM69_DEBUG(PSTR("RFM69:DUMP:ModulationShaping : 01 -> Gaussian filter, BT = 1.0\n"));
				} else if (capVal == 0b10) {
					RFM69_DEBUG(PSTR("RFM69:DUMP:ModulationShaping : 10 -> Gaussian filter, BT = 0.5\n"));
				} else if (capVal == 0b11) {
					RFM69_DEBUG(PSTR("RFM69:DUMP:ModulationShaping : 11 -> Gaussian filter, BT = 0.3\n"));
				}
			} else {
				if (capVal == 0b00) {
					RFM69_DEBUG(PSTR("RFM69:DUMP:ModulationShaping : 00 -> no shaping\n"));
				} else if (capVal == 0b01) {
					RFM69_DEBUG(PSTR("RFM69:DUMP:ModulationShaping : 01 -> filtering with f(cutoff) = BR\n"));
				} else if (capVal == 0b10) {
					RFM69_DEBUG(PSTR("RFM69:DUMP:ModulationShaping : 10 -> filtering with f(cutoff) = 2*BR\n"));
				} else if (capVal == 0b11) {
					RFM69_DEBUG(PSTR("RFM69:DUMP:ModulationShaping : ERROR - 11 is reserved\n"));
				}
			}

			break;
		}

		case 0x3: {
			bitRate = (regVal << 8);
			break;
		}

		case 0x4: {
			bitRate |= regVal;
			RFM69_DEBUG(
			    PSTR("RFM69:DUMP:REG=0x%02x Bit Rate (Chip Rate when Manchester encoding is enabled)\n"), regAddr);

			uint32_t val = 32UL * 1000UL * 1000UL / bitRate;
			RFM69_DEBUG(PSTR("RFM69:DUMP:BitRate : %lu\n"), val);
			break;
		}

		case 0x5: {
			freqDev = ((regVal & 0x3f) << 8);
			break;
		}

		case 0x6: {
			freqDev |= regVal;

			uint32_t val = 61UL * freqDev;
			RFM69_DEBUG(PSTR("RFM69:DUMP:REG=0x%02x Frequency deviation\n"), regAddr);
			RFM69_DEBUG(PSTR("RFM69:DUMP:Fdev : %lu\n"), val);
			break;
		}

		case 0x7: {
			uint32_t tempVal = regVal;
			freqCenter = (tempVal << 16);
			break;
		}

		case 0x8: {
			uint32_t tempVal = regVal;
			freqCenter = freqCenter | (tempVal << 8);
			break;
		}

		case 0x9: {
			freqCenter = freqCenter | regVal;
			uint32_t val = 61UL * freqCenter;
			RFM69_DEBUG(PSTR("RFM69:DUMP:REG=0x%02x RF Carrier frequency \n"), regAddr);
			RFM69_DEBUG(PSTR("RFM69:DUMP:FRF : %lu\n"), val);
			break;
		}

		case 0xa: {
			RFM69_DEBUG(PSTR("RFM69:DUMP:REG=0x%02x RC calibration control & status\n"), regAddr);
			if (0x40 & regVal) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:RcCalDone : 1 -> RC calibration is over\n"));
			} else {
				RFM69_DEBUG(PSTR("RFM69:DUMP:RcCalDone : 0 -> RC calibration is in progress\n"));
			}

			break;
		}

		case 0xb: {
			RFM69_DEBUG(
			    PSTR("RFM69:DUMP:REG=0x%02x Improved AFC routine for signals with modulation index lower than 2.  Refer to section 3.4.16 for details\n"),
			    regAddr);
			if (0x20 & regVal) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:AfcLowBetaOn : 1 -> Improved AFC routine\n"));
			} else {
				RFM69_DEBUG(PSTR("RFM69:DUMP:AfcLowBetaOn : 0 -> Standard AFC routine\n"));
			}
			break;
		}

		case 0xc: {
			RFM69_DEBUG(PSTR("RFM69:DUMP:REG=0x%02x Reserved\n"), regAddr);
			break;
		}

		case 0xd: {
			byte val;
			RFM69_DEBUG(PSTR("RFM69:DUMP:REG=0x%02x Resolution of Listen mode Idle time (calibrated RC osc)\n"),
			            regAddr);
			val = regVal >> 6;
			if (val == 0b00) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:ListenResolIdle : 00 -> reserved\n"));
			} else if (val == 0b01) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:ListenResolIdle : 01 -> 64 us\n"));
			} else if (val == 0b10) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:ListenResolIdle : 10 -> 4.1 ms\n"));
			} else if (val == 0b11) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:ListenResolIdle : 11 -> 262 ms\n"));
			}

			RFM69_DEBUG(PSTR("RFM69:DUMP:Resolution of Listen mode Rx time (calibrated RC osc)\n"));
			val = (regVal >> 4) & 0x3;
			if (val == 0b00) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:ListenResolRx : 00 -> reserved\n"));
			} else if (val == 0b01) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:ListenResolRx : 01 -> 64 us\n"));
			} else if (val == 0b10) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:ListenResolRx : 10 -> 4.1 ms\n"));
			} else if (val == 0b11) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:ListenResolRx : 11 -> 262 ms\n"));
			}

			RFM69_DEBUG(PSTR("RFM69:DUMP:Criteria for packet acceptance in Listen mode\n"));
			if (0x8 & regVal) {
				RFM69_DEBUG(
				    PSTR("RFM69:DUMP:ListenCriteria : 1 -> signal strength is above RssiThreshold and SyncAddress matched\n"));
			} else {
				RFM69_DEBUG(PSTR("RFM69:DUMP:ListenCriteria : 0 -> signal strength is above RssiThreshold\n"));
			}


			RFM69_DEBUG(PSTR("RFM69:DUMP:Action taken after acceptance of a packet in Listen mode\n"));
			val = (regVal >> 1) & 0x3;
			if (val == 0b00) {
				RFM69_DEBUG(
				    PSTR("RFM69:DUMP:ListenEnd : 00 -> chip stays in Rx mode. Listen mode stops and must be disabled (see section 4.3)\n"));
			} else if (val == 0b01) {
				RFM69_DEBUG(
				    PSTR("RFM69:DUMP:ListenEnd : 01 -> chip stays in Rx mode until PayloadReady or Timeout interrupt occurs.  It then goes to the mode defined by Mode. Listen mode stops and must be disabled (see section 4.3)\n"));
			} else if (val == 0b10) {
				RFM69_DEBUG(
				    PSTR("RFM69:DUMP:ListenEnd : 10 -> chip stays in Rx mode until PayloadReady or Timeout occurs.  Listen mode then resumes in Idle state.  FIFO content is lost at next Rx wakeup.\n"));
			} else if (val == 0b11) {
				RFM69_DEBUG(PSTR("RFM69:DUMP:ListenEnd : 11 -> Reserved\n"));
			}
			break;
		}

		default: {
		}
		}
#endif
		(void)regVal;
	}
}
