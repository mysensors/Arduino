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
* RF24 driver refactored and optimized for speed and size, copyright (C) 2016 Olivier Mauti <olivier@mysensors.org>
*/

// PA levels
#define RF24_PA_MIN				(0)
#define RF24_PA_LOW				(1)
#define RF24_PA_HIGH			(2)
#define RF24_PA_MAX				(3)

// power level limits
#define RF24_MIN_POWER_LEVEL	(0)
#define RF24_MAX_POWER_LEVEL	(3)

// data rate
#define RF24_1MBPS				(0)
#define RF24_2MBPS				(1)
#define RF24_250KBPS			(2)

// CRC
#define RF24_CRC_DISABLED		(0)
#define RF24_CRC_8				(2)
#define RF24_CRC_16				(3)

// ARD, auto retry delay
#ifndef RF24_SET_ARD
#define RF24_SET_ARD			(5)	//=1500us
#endif

// ARD, auto retry count
#ifndef RF24_SET_ARC
#define RF24_SET_ARC			(15)
#endif

// nRF24L01(+) register definitions
#define RF24_REG_NRF_CONFIG		(0x00)
#define RF24_REG_EN_AA			(0x01)
#define RF24_REG_EN_RXADDR		(0x02)
#define RF24_REG_SETUP_AW		(0x03)
#define RF24_REG_SETUP_RETR		(0x04)
#define RF24_REG_RF_CH			(0x05)
#define RF24_REG_RF_SETUP		(0x06)
#define RF24_REG_STATUS			(0x07)
#define RF24_REG_OBSERVE_TX		(0x08)
#define RF24_REG_RPD			(0x09)          // nRF24L01+
#define RF24_REG_CD				(RF24_REG_RPD)  // nRF24L01
#define RF24_REG_RX_ADDR_P0		(0x0A)
#define RF24_REG_RX_ADDR_P1		(0x0B)
#define RF24_REG_RX_ADDR_P2		(0x0C)
#define RF24_REG_RX_ADDR_P3		(0x0D)
#define RF24_REG_RX_ADDR_P4		(0x0E)
#define RF24_REG_RX_ADDR_P5		(0x0F)
#define RF24_REG_TX_ADDR		(0x10)
#define RF24_REG_RX_PW_P0		(0x11)
#define RF24_REG_RX_PW_P1		(0x12)
#define RF24_REG_RX_PW_P2		(0x13)
#define RF24_REG_RX_PW_P3		(0x14)
#define RF24_REG_RX_PW_P4		(0x15)
#define RF24_REG_RX_PW_P5		(0x16)
#define RF24_REG_FIFO_STATUS	(0x17)
#define RF24_REG_DYNPD			(0x1C)
#define RF24_REG_FEATURE		(0x1D)

// mask
#define RF24_REGISTER_MASK		(0x1F)

// instructions
#define RF24_CMD_READ_REGISTER				(0x00)
#define RF24_CMD_WRITE_REGISTER				(0x20)
#define RF24_CMD_ACTIVATE					(0x50)
#define RF24_CMD_READ_RX_PL_WID				(0x60)
#define RF24_CMD_READ_RX_PAYLOAD			(0x61)
#define RF24_CMD_WRITE_TX_PAYLOAD			(0xA0)
#define RF24_CMD_WRITE_ACK_PAYLOAD			(0xA8)
#define RF24_CMD_WRITE_TX_PAYLOAD_NO_ACK	(0xB0)
#define RF24_CMD_FLUSH_TX					(0xE1)
#define RF24_CMD_FLUSH_RX					(0xE2)
#define RF24_CMD_REUSE_TX_PL				(0xE3)
#define RF24_CMD_NOP						(0xFF)

// bit mnemonics
#define RF24_MASK_RX_DR		(6)
#define RF24_MASK_TX_DS		(5)
#define RF24_MASK_MAX_RT	(4)
#define RF24_EN_CRC			(3)
#define RF24_CRCO			(2)
#define RF24_PWR_UP			(1)
#define RF24_PRIM_RX		(0)

// auto ACK
#define RF24_ENAA_P5		(5)
#define RF24_ENAA_P4		(4)
#define RF24_ENAA_P3		(3)
#define RF24_ENAA_P2		(2)
#define RF24_ENAA_P1		(1)
#define RF24_ENAA_P0		(0)

// rx pipe
#define RF24_ERX_P5			(5)
#define RF24_ERX_P4			(4)
#define RF24_ERX_P3			(3)
#define RF24_ERX_P2			(2)
#define RF24_ERX_P1			(1)
#define RF24_ERX_P0			(0)

// dynamic payload
#define RF24_DPL_P5			(5)
#define RF24_DPL_P4			(4)
#define RF24_DPL_P3			(3)
#define RF24_DPL_P2			(2)
#define RF24_DPL_P1			(1)
#define RF24_DPL_P0			(0)

#define RF24_AW				(0)
#define RF24_ARD			(4)
#define RF24_ARC			(0)
#define RF24_PLL_LOCK		(4)
#define RF24_CONT_WAVE	(7)
#define RF24_RF_DR			(3)
#define RF24_RF_PWR			(6)
#define RF24_RX_DR			(6)
#define RF24_TX_DS			(5)
#define RF24_MAX_RT			(4)
#define RF24_RX_P_NO		(1)
#define RF24_TX_FULL		(0)
#define RF24_PLOS_CNT		(4)
#define RF24_ARC_CNT		(0)
#define RF24_TX_REUSE		(6)
#define RF24_FIFO_FULL		(5)
#define RF24_TX_EMPTY		(4)
#define RF24_RX_FULL		(1)
#define RF24_RX_EMPTY		(0)
#define RF24_RPD    		(0)         // nRF24L01+
#define RF24_CD     		(RF24_RPD)  // nRF24L01

// features
#define RF24_EN_DPL			(2)
#define RF24_EN_ACK_PAY		(1)
#define RF24_EN_DYN_ACK		(0)

#define RF24_LNA_HCURR		(0)
#define RF24_RF_DR_LOW		(5)
#define RF24_RF_DR_HIGH		(3)
#define RF24_RF_PWR_LOW		(1)
#define RF24_RF_PWR_HIGH	(2)
