/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors formrs a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of
 * the network topology allowing messages to be routed to nodes.
 *
 * Created by Frank Holtz
 * Copyright (C) 2017-2020 Frank Holtz
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

#include "Radio.h"
#include "Radio_ESB.h"
#include "hal/architecture/NRF5/MyHwNRF5.h"
#include "drivers/CircularBuffer/CircularBuffer.h"

// internal functions
static uint8_t reverse_byte(uint8_t address);
static void _stopTimer();
#ifdef MY_DEBUG_VERBOSE_NRF5_ESB
void print_radio_stats();
#endif

// RX Buffer
static NRF5_ESB_Packet rx_circular_buffer_buffer[MY_NRF5_ESB_RX_BUFFER_SIZE];
// Poiter to the active buffer
static NRF5_ESB_Packet rx_tx_buffer;
// Circular buffer
static CircularBuffer<NRF5_ESB_Packet>
rx_circular_buffer(rx_circular_buffer_buffer, MY_NRF5_ESB_RX_BUFFER_SIZE);
// Dedect duplicate packages for every pipe available
static volatile uint32_t package_ids[8];

// Flag for ack received
static volatile bool ack_received;
// Flag for timeout event
static volatile bool events_timeout;
// Flag for address received for emergency timer
static volatile bool events_address;

// TX PID (2 bits)
uint8_t tx_pid = 0;
// Last RSSI sample provided by NRF5_ESB_readMessage
static volatile int16_t rssi_rx;
// Last RSSI sample by last package
static volatile int16_t rssi_tx;
// Buffer node address
static uint8_t node_address = 0;
// TX power level
static int8_t tx_power_level = (MY_NRF5_ESB_PA_LEVEL << RADIO_TXPOWER_TXPOWER_Pos);


// Initialize radio unit
static bool NRF5_ESB_initialize()
{
	NRF5_RADIO_DEBUG(PSTR("NRF5:INIT:ESB\n"));

#if defined(SOFTDEVICE_PRESENT)
	// Disable the SoftDevice; requires NRF5 SDK available
	sd_softdevice_disable();
#endif

	// Power on radio unit
	NRF_RADIO->POWER = 1;
#ifdef NRF52
	// Fix PAN#182
	*(volatile uint32_t *) 0x4000173C |= (1 << 10);
#endif

	// Disable shorts
	NRF_RADIO->SHORTS = 0;

	// Disable radio
	NRF_RADIO->TASKS_DISABLE = 1;

	// Enable radio interrupt
	NVIC_SetPriority(RADIO_IRQn, 1);
	NVIC_ClearPendingIRQ(RADIO_IRQn);
	NVIC_EnableIRQ(RADIO_IRQn);

	// Enable timer interrupt
	NVIC_SetPriority(NRF5_RADIO_TIMER_IRQN, 2);
	NVIC_ClearPendingIRQ(NRF5_RADIO_TIMER_IRQN);
	NVIC_EnableIRQ(NRF5_RADIO_TIMER_IRQN);

	// Clear all radio events
	NRF_RADIO->EVENTS_ADDRESS = 0;
	NRF_RADIO->EVENTS_BCMATCH = 0;
	NRF_RADIO->EVENTS_DEVMATCH = 0;
	NRF_RADIO->EVENTS_DEVMISS = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_PAYLOAD = 0;
	NRF_RADIO->EVENTS_READY = 0;
	NRF_RADIO->EVENTS_RSSIEND = 0;

	// Disable all interrupts
	NRF_RADIO->INTENCLR = (uint32_t)~0;

	// Select interrupt events (End of packet and bitcounter event for ACK management)
	NRF_RADIO->INTENSET = NRF5_EBS_RADIO_INT_RX;

	// Configure radio parameters: tx power
	NRF_RADIO->TXPOWER = tx_power_level;

	// Configure radio parameters: radio channel
	NRF_RADIO->FREQUENCY = MY_NRF5_ESB_CHANNEL;

	// Configure radio parameters: data rate
	NRF_RADIO->MODE = MY_NRF5_ESB_MODE;

	// Configure radio parameters: CRC16
	NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos);
	NRF_RADIO->CRCINIT = 0xFFFFUL;
	NRF_RADIO->CRCPOLY = 0x11021UL;

	// Radio address config
	uint8_t address[MY_NRF5_ESB_ADDR_WIDTH] = {MY_NRF5_ESB_BASE_RADIO_ID};

	// Configure addresses
	NRF_RADIO->PREFIX0 = (NRF5_ESB_NODE_ADDR_MSK | reverse_byte(node_address) <<
	                      (NRF5_ESB_NODE_ADDR << 5));
	NRF_RADIO->BASE0 = reverse_byte(address[1]) << 24 |
	                   reverse_byte(address[2]) << 16 |
	                   reverse_byte(address[3]) << 8 | reverse_byte(address[4]);
	NRF_RADIO->BASE1 = reverse_byte(address[1]) << 24 |
	                   reverse_byte(address[2]) << 16 |
	                   reverse_byte(address[3]) << 8 | reverse_byte(address[4]);
	NRF_RADIO->PREFIX1 = NRF5_ESB_TX_ADDR_MSK; // Broadcast and send address

	// Enable listening on Node and BC address
	NRF_RADIO->RXADDRESSES = (1 << NRF5_ESB_NODE_ADDR) | (1 << NRF5_ESB_BC_ADDR);

	// Packet configuration for nRF24 compatibility
	NRF_RADIO->PCNF0 = (6 << RADIO_PCNF0_LFLEN_Pos) | // 6 Bits length field
	                   (0 << RADIO_PCNF0_S0LEN_Pos) | // No S0
#ifdef RADIO_PCNF0_S1INCL_Pos
	                   (1 << RADIO_PCNF0_S1INCL_Pos) | // Force include S1 in RAM
#endif
	                   (3 << RADIO_PCNF0_S1LEN_Pos); // 3 Bits S1 (NOACK and PID)

	// Packet configuration
	NRF_RADIO->PCNF1 =
	    (MAX_MESSAGE_SIZE << RADIO_PCNF1_MAXLEN_Pos) | // maximum length
	    (0 << RADIO_PCNF1_STATLEN_Pos) |                 // minimum message length
	    ((MY_NRF5_ESB_ADDR_WIDTH - 1) << RADIO_PCNF1_BALEN_Pos) | // Set base address length
	    (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos) | // Big endian
	    (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos); // Disable whitening

#ifdef RADIO_MODECNF0_RU_Default
	//  Configure nRF52 specific mode register
	NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Default << RADIO_MODECNF0_RU_Pos) |
	                      (RADIO_MODECNF0_DTX_Center << RADIO_MODECNF0_DTX_Pos);
#endif

#ifdef NRF52
	// Fix PAN#102 and PAN#106
	//*((volatile uint32_t *)0x40001774) = (*((volatile uint32_t *)0x40001774) & 0xFFFFFFFE) | 0x01000000;
#endif

#ifdef NRF51
	// Enable timer
	NRF5_RADIO_TIMER->POWER = 1;
#endif

	// Configure DMA target address
	NRF_RADIO->PACKETPTR = (uint32_t)&rx_tx_buffer;

	// Stop timer, if running
	_stopTimer();

	// Reset address event flag used for emergency reset timer
	events_address = false;

	// Prepare timer running at 1 MHz/1us
	NRF5_RADIO_TIMER->PRESCALER = 4;
	// Timer mode
	NRF5_RADIO_TIMER->MODE = TIMER_MODE_MODE_Timer;
	// in 16 Bit mode
	NRF5_RADIO_TIMER->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
	// Stop timer when CC3 reached
	NRF5_RADIO_TIMER->SHORTS =
	    TIMER_SHORTS_COMPARE3_CLEAR_Msk | TIMER_SHORTS_COMPARE3_STOP_Msk;
	// Enable interrupt
	NRF5_RADIO_TIMER->INTENCLR = (uint32_t)~0;;
	NRF5_RADIO_TIMER->INTENSET = TIMER_INTENSET_COMPARE3_Enabled << TIMER_INTENSET_COMPARE3_Pos;


#ifdef MY_DEBUG_VERBOSE_NRF5_ESB
	intcntr_ready=0;
	intcntr_end=0;
#endif

	return true;
}

static void NRF5_ESB_powerDown()
{
	NRF5_RADIO_DEBUG(PSTR("NRF5:PD\n"));

	// Disable inerrupt
	NVIC_DisableIRQ(RADIO_IRQn);
	NVIC_DisableIRQ(NRF5_RADIO_TIMER_IRQN);

	// Clear PPI
	NRF_PPI->CHENCLR = NRF5_ESB_PPI_BITS;

	// Save power level
	tx_power_level = NRF_RADIO->TXPOWER;

	// Power off readio unit
	NRF_RADIO->POWER = 0;

	// Shutdown timer
	NRF5_RADIO_TIMER->TASKS_SHUTDOWN = 1;
#ifdef NRF51
	// Power off timer
	NRF5_RADIO_TIMER->POWER = 0;
#endif
}

static void NRF5_ESB_powerUp()
{
	NRF5_ESB_initialize();
}

static void NRF5_ESB_sleep()
{
	NRF5_RADIO_DEBUG(PSTR("NRF5:SLP\n"));

	// Disable shorts
	NRF_RADIO->SHORTS = 0;

	// Disable radio
	NRF_RADIO->TASKS_DISABLE = 1;
}

static void NRF5_ESB_standBy()
{
	NRF5_RADIO_DEBUG(PSTR("NRF5:SBY\n"));
	NRF5_ESB_startListening();
}

static bool NRF5_ESB_sanityCheck()
{
	// always true
	return true;
}

static void NRF5_ESB_setNodeAddress(const uint8_t address)
{
	node_address = address;
	NRF_RADIO->PREFIX0 = (NRF_RADIO->PREFIX0 & NRF5_ESB_NODE_ADDR_MSK) |
	                     reverse_byte(node_address) << (NRF5_ESB_NODE_ADDR << 5);
}

static uint8_t NRF5_ESB_getNodeID()
{
	return reverse_byte((NRF_RADIO->PREFIX0 & NRF5_ESB_NODE_ADDR_MSK) >> (NRF5_ESB_NODE_ADDR << 5));
}

static void NRF5_ESB_startListening()
{
	NRF5_RADIO_DEBUG(PSTR("NRF5:STL\n"));

	// Check if radio is initialized
	if (NRF_RADIO->POWER == 0) {
		NRF5_ESB_initialize();
	}

	// Bring radio into RX mode
	NRF_RADIO->SHORTS = NRF5_ESB_SHORTS_RX;

	// Switch to RX
	if (NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled) {
		NRF_RADIO->TASKS_RXEN = 1;
	} else {
		NRF_RADIO->TASKS_DISABLE = 1;
	}
}

static bool NRF5_ESB_isDataAvailable()
{
	return rx_circular_buffer.available() > 0;
}

static uint8_t NRF5_ESB_readMessage(void *data)
{
	uint8_t ret = 0;

	// get content from rx buffer
	NRF5_ESB_Packet *buffer = rx_circular_buffer.getBack();
	// Nothing to read?
	if (buffer != NULL) {
		// copy content
		memcpy(data, buffer->data, buffer->len);
		ret = buffer->len;
		rssi_rx = 0-buffer->rssi;

		// Debug message
#ifdef MY_DEBUG_VERBOSE_NRF5_ESB
		NRF5_RADIO_DEBUG(PSTR("NRF5:RX:LEN=%" PRIu8 ",NOACK=%" PRIu8 ",PID=%" PRIu8 ",RSSI=%" PRIi16 ",RX=%"
		                      PRIu32 "\n"),
		                 buffer->len, buffer->noack, buffer->pid, rssi_rx, buffer->rxmatch);
#endif

		// release buffer
		rx_circular_buffer.popBack();
	}

	return ret;
}

static bool NRF5_ESB_sendMessage(uint8_t recipient, const void *buf, uint8_t len, const bool noACK)
{
	// Increment TX PID
	tx_pid++;

	NRF5_RADIO_DEBUG(PSTR("NRF5:SND:TO=%" PRIu8 ",LEN=%" PRIu8 ",PID=%" PRIu8 ",NOACK=%" PRIu8 "\n"),
	                 recipient, len, tx_pid,
	                 noACK); // send message

	// Check if radio is initialized
	if (NRF_RADIO->POWER == 0) {
		NRF5_ESB_initialize();
	}

	// check length and truncate data
	if (len > MAX_MESSAGE_SIZE) {
		len = MAX_MESSAGE_SIZE;
	}

	// Calculate number of retries
	int8_t tx_retries;
	if (recipient == BROADCAST_ADDRESS) {
		tx_retries = NRF5_ESB_BC_ARC;
	} else {
		tx_retries = ((noACK == false)?(NRF5_ESB_ARC_ACK):(NRF5_ESB_ARC_NOACK));
	}
	int8_t tx_retries_start = tx_retries;
	ack_received = false;
#ifdef MY_DEBUG_VERBOSE_NRF5_ESB
	int wakeups=0;
	intcntr_addrmatch = 0;
	intcntr_end = 0;
	intcntr_disabled = 0;
	intcntr_timer_cc3 = 0;
#endif

	/** Loop until tx_retries == 0
	 * Because the radio is in RX state between TX
	 * the radio is initialized with each iteration
	 */
	while ((tx_retries > 0) && (ack_received == false)) {
		tx_retries--;
		/** Wait until RX is end, if any activity
		 * Wait a maximum of 10ms
		 */
		uint8_t max_wait = 10;
		while ((events_address) && (max_wait>0)) {
			// Sleep until next interrupt
			hwSleep(1);
			max_wait--;
		}

		// Stop RX
		NRF_RADIO->SHORTS = 0;
		NRF_RADIO->TASKS_DISABLE = 1;

		/** Prepare Buffer
		 **/
		// copy data to tx_buffer
		memcpy(&rx_tx_buffer.data[0], buf, len);

		// build metadata
		rx_tx_buffer.len = len;
#ifndef MY_NRF5_ESB_REVERSE_ACK_TX
		rx_tx_buffer.noack = noACK || recipient==BROADCAST_ADDRESS;
#else
		// reverse the noack bit
		rx_tx_buffer.noack = !(noACK || recipient==BROADCAST_ADDRESS);
#endif
		rx_tx_buffer.pid = tx_pid;

		// Set RSSI to invalid
		rssi_tx = INVALID_RSSI;

		// Reset event flags
		events_timeout = false;

		/** Configure radio parameters
		 **/
		// configure TX address
		NRF_RADIO->PREFIX1 = (NRF_RADIO->PREFIX1 & NRF5_ESB_TX_ADDR_MSK) |
		                     (reverse_byte(recipient) << (NRF5_ESB_TX_ADDR - 4));
		// Enable listening on Node, BC and TX address
		NRF_RADIO->RXADDRESSES = (1 << NRF5_ESB_NODE_ADDR) | (1 << NRF5_ESB_BC_ADDR) |
		                         (1 << NRF5_ESB_TX_ADDR);
		// Configure TX address to address at index NRF5_ESB_TX_ADDR
		NRF_RADIO->TXADDRESS = NRF5_ESB_TX_ADDR;
		// Enable TX when ready, Enable TX after disabling task
		NRF_RADIO->SHORTS = NRF5_ESB_SHORTS_TX;
		// Disable all interrupts
		NRF_RADIO->INTENCLR = (uint32_t)~0;
		// Select interrupt events for TX
		NRF_RADIO->INTENSET = NRF5_EBS_RADIO_INT_TX;

		/** Configure timer
		 * CC[3] is used for retransmit and timeout
		 **/
		// reset timer
		_stopTimer();

		if (tx_retries > 0) {
			// Set retransmit time with a litte jitter
			NRF5_RADIO_TIMER->CC[3] = NRF5_ESB_ARD + NRF5_ESB_MAX_PACKET_TIME + (tx_pid <<
			                          NRF5_ESB_byte_time());
		} else {
			// Wait for ACK with a litte jitter
			NRF5_RADIO_TIMER->CC[3] = NRF5_ESB_ARD + NRF5_ESB_ACK_WAIT + NRF5_ESB_MAX_PACKET_TIME +
			                          (tx_pid << NRF5_ESB_byte_time());;
		}

		/** Start TX and timer
		 **/
		NRF_RADIO->TASKS_TXEN = 1;
		//NRF5_RADIO_TIMER->TASKS_START = 1;

		/** wait until ACK is seen or timeout
		 **/
		// Wait for end of transmission
		while ((ack_received == false) && (events_timeout==false)) {
			// Power off CPU until next interrupt
			hwSleep();
#ifdef MY_DEBUG_VERBOSE_NRF5_ESB
			wakeups++;
#endif
		}
	}

	// reset timer
	_stopTimer();

	// Calculate RSSI
	if ((ack_received == true) && (rssi_tx == INVALID_RSSI)) {
		// calculate pseudo-RSSI based on retransmission counter (ARC)
		// min -104dBm at 250kBps
		// Arbitrary definition: ARC 0 == -29, ARC 15 = -104
		rssi_tx = (-29 - (8 * (tx_retries_start - tx_retries)));
	}

#ifdef MY_DEBUG_VERBOSE_NRF5_ESB
	NRF5_RADIO_DEBUG(PSTR("NRF5:SND:ACK=%" PRIu8 ",RTRY=%" PRIi8 ",RSSI=%" PRIi16
	                      ",WAKE=%" PRIu32 "\n"),
	                 ack_received, tx_retries_start - tx_retries, rssi_tx, wakeups);
	print_radio_stats();
#endif
	return ack_received;
};

static int16_t NRF5_ESB_getSendingRSSI()
{
	return rssi_tx;
}

static int16_t NRF5_ESB_getReceivingRSSI()
{
	return rssi_rx;
}

/*
 * Internal helper functions
 */

#ifdef MY_DEBUG_VERBOSE_NRF5_ESB
// Print interrupt counter statistics
void print_radio_stats()
{
	NRF5_RADIO_DEBUG(PSTR("NRF5:INTCTR:ADDRM=%" PRIu32 ",END=%" PRIu32 ",DISABLED=%" PRIu32 ",TMRC3=%"
	                      PRIu32 "\n"),
	                 intcntr_addrmatch, intcntr_end, intcntr_disabled, intcntr_timer_cc3);
	intcntr_addrmatch = 0;
	intcntr_end = 0;
	intcntr_disabled = 0;
	intcntr_timer_cc3 = 0;

	NRF5_RADIO_DEBUG(PSTR("NRF5:REGISTERS:STATE=0x%" PRIX32 ",SHORTS=0x%" PRIX32 ",INTENSET=0x%" PRIX32
	                      ",POWER=0x%" PRIX32 "\n"),
	                 NRF_RADIO->STATE, NRF_RADIO->SHORTS,NRF_RADIO->INTENSET, NRF_RADIO->POWER);
	NRF5_RADIO_DEBUG(PSTR("NRF5:ADDR:PREFIX0=0x04%"  PRIX32 ",PREFIX1=0x04%" PRIX32
	                      ",TXADDRESS=0x%" PRIX32 ",RXADDRESSES=0x%" PRIX32 ",RXMATCH=0x%" PRIX32 "\n"),
	                 NRF_RADIO->PREFIX0, NRF_RADIO->PREFIX1,NRF_RADIO->TXADDRESS,
	                 NRF_RADIO->RXADDRESSES, NRF_RADIO->RXMATCH);
}
#endif


// Reverse a byte for address
static uint8_t reverse_byte(uint8_t address)
{
#if __CORTEX_M >= (0x01U)
	return __REV(__RBIT(address));
#else
	address = ((address * 0x0802LU & 0x22110LU) | (address * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
#endif
	return address;
}


static void _stopTimer()
{
	// Stop timer
	NRF5_RADIO_TIMER->TASKS_STOP = 1;
	// NRF52 PAN#78
	NRF5_RADIO_TIMER->TASKS_SHUTDOWN = 1;
	// Reset timer
	NRF5_RADIO_TIMER->TASKS_CLEAR = 1;
	// Reset compare events
#ifdef NRF51
	for (uint8_t i=0; i<4; i++) {
#else
	for (uint8_t i=0; i<6; i++) {
#endif
		NRF_RESET_EVENT(NRF5_RADIO_TIMER->EVENTS_COMPARE[i]);
	}
	// Maximum time between ADDRESS and END event
	NRF5_RADIO_TIMER->CC[3] = NRF5_ESB_MAX_PACKET_TIME;
}

// Switch back to RX without ACK
inline void _stopACK()
{
	NRF_RADIO->SHORTS = NRF5_ESB_SHORTS_RX;
	NRF_RADIO->TASKS_DISABLE = 1;
}

// Calculate time to transmit an byte in µs as bit shift -> 2^X
static inline uint8_t NRF5_ESB_byte_time()
{
	if ((MY_NRF5_ESB_MODE == NRF5_1MBPS) or
	        (MY_NRF5_ESB_MODE == NRF5_BLE_1MBPS)) {
		return (3);
	} else if (MY_NRF5_ESB_MODE == NRF5_2MBPS) {
		return (2);
	}
#ifdef RADIO_MODE_MODE_Nrf_250Kbit
	else if (MY_NRF5_ESB_MODE == NRF5_250KBPS) {
		return (5);
	}
#endif
}

extern "C" {
	/** Radio Interrupt handler */
	void RADIO_IRQHandler()
	{
		// Address is send or received
		if (NRF_RADIO->EVENTS_ADDRESS == 1) {
			NRF_RESET_EVENT(NRF_RADIO->EVENTS_ADDRESS);
			// Start emergency timer to dedect missed END-Events (nRF52832 PAN#102) or ACKs
			NRF5_RADIO_TIMER->TASKS_START = 1;
			// Set flag for timer interrupt
			events_address = true;
#ifdef MY_DEBUG_VERBOSE_NRF5_ESB
			intcntr_addrmatch++;
#endif
			/** What to do after RX/TX END?
			 *	The radio is switching to RX after each RX or TX packet
			 *  expect a packet was addressed to the node address.
			 **/
			if (NRF_RADIO->STATE >= RADIO_STATE_STATE_TxRu) {
				// TX-Mode -> switch to RX
				NRF_RADIO->SHORTS = NRF5_ESB_SHORTS_TX_RX;
				// Disable all interrupts
				NRF_RADIO->INTENCLR = (uint32_t)~0;
				// Select interrupt events for RX
				NRF_RADIO->INTENSET = NRF5_EBS_RADIO_INT_RX;
			} else {
				/* Regular or ACK packet
				 * After an RX packet, the radio is keept in RX mode
				 */
				if (NRF_RADIO->RXMATCH == NRF5_ESB_NODE_ADDR) {
					// Switch to TX for ACK after END.
					// noACK flag is ignored for compatibility with NRF24 clones
					NRF_RADIO->SHORTS = NRF5_ESB_SHORTS_RX_TX;
				} else {
					// Switch to RX after END
					NRF_RADIO->SHORTS = NRF5_ESB_SHORTS_RX;
				}
				// Stop listening on TX_ADDR. Enable listening on node and broadcast address
				NRF_RADIO->RXADDRESSES = (1 << NRF5_ESB_NODE_ADDR) | (1 << NRF5_ESB_BC_ADDR);
			}

			// Reset TX address to node address for ACK
			NRF_RADIO->TXADDRESS = NRF5_ESB_NODE_ADDR;
		}

		/** This event generated only after RX ends
		 */
		if (NRF_RADIO->EVENTS_END == 1) {
			// Reset END-Event
			NRF_RESET_EVENT(NRF_RADIO->EVENTS_END);
			// Reset emergency timer reset flag
			events_address = false;

#ifdef MY_DEBUG_VERBOSE_NRF5_ESB
			intcntr_end++;
#endif
			// Check CRC
			if (NRF_RADIO->CRCSTATUS == 0) {
				// discard RX data
				_stopACK();
			} else 	{
#ifdef MY_DEBUG_VERBOSE_NRF5_ESB
				// Store debug data
				rx_tx_buffer.rxmatch = NRF_RADIO->RXMATCH;
#endif
				// Store RSSI sample
				rx_tx_buffer.rssi = NRF_RADIO->RSSISAMPLE;

				// Addressed to RX or TX address?
				if ((NRF_RADIO->RXMATCH == NRF5_ESB_NODE_ADDR) ||
				        (NRF_RADIO->RXMATCH == NRF5_ESB_BC_ADDR)) {
					/** ignore doubled packages
					 *  calculate a package id from PID and CRC
					 **/
					uint32_t pkgid = rx_tx_buffer.pid << 16 | NRF_RADIO->RXCRC;
					if (pkgid != package_ids[NRF_RADIO->RXMATCH]) {
						// Add package to rx buffer
						if (rx_circular_buffer.pushFront(&rx_tx_buffer)) {
							// correct package -> store id to dedect duplicates
							package_ids[NRF_RADIO->RXMATCH] = pkgid;
						} else {
							/** buffer has reached capacity -> stop ACK
								switch back to RX
							**/
							_stopACK();
						}
					}
				} else {
					/** ACK to TX address received?
					 * An ACK paket can have a payload of
					 * 0, 1 or 2 bytes. Longer ACKs are ignored
					 * The RSSI sample is the first byte
					 * The second byte can be used for protocol extensions
					 **/
					if (rx_tx_buffer.len <= 2) {
						// notify TX process
						ack_received = true;
						// rssi value in ACK included?
						if (rx_tx_buffer.len > 1) {
							rssi_tx = 0-rx_tx_buffer.data[0];
						}
					}
				}

				/** prepare ACK
				 *  The ACK is send, when the radio has to switch to TX
				 *  otherwise the buffer content is ignored
				 **/
				rx_tx_buffer.data[0]=rx_tx_buffer.rssi;
				rx_tx_buffer.len=1;
#ifndef MY_NRF5_ESB_REVERSE_ACK_TX
				rx_tx_buffer.noack = 1;
#else
				rx_tx_buffer.noack = 0;
#endif
			}
		}
		// Handle Disabled event (used in NRF5_ESB_sendMessage)
		if (NRF_RADIO->EVENTS_DISABLED == 1) {
			NRF_RESET_EVENT(NRF_RADIO->EVENTS_DISABLED);
			events_address = false;
#ifdef MY_DEBUG_VERBOSE_NRF5_ESB
			intcntr_disabled++;
#endif
		}
	}

	/** Timer Interrupt Handler
	 *  This timer is used to handle TX retransmit timing
	 *
	 */
	void NRF5_RADIO_TIMER_IRQ_HANDLER()
	{
		// TX Timeout (ARD, ACK)
		if (NRF5_RADIO_TIMER->EVENTS_COMPARE[3] == 1) {
			NRF_RESET_EVENT(NRF5_RADIO_TIMER->EVENTS_COMPARE[3]);

			// Timeout after address event?
			if (events_address) {
				// Force switch radio to RX after disable
				NRF_RADIO->SHORTS = NRF5_ESB_SHORTS_RX;
				// Disable Radio
				NRF_RADIO->TASKS_DISABLE = 1;
			}
			// Set timeout event flag
			events_timeout = true;
			// Reset address flag
			events_address = false;
#ifdef MY_DEBUG_VERBOSE_NRF5_ESB
			intcntr_timer_cc3++;
#endif
		}
	}
} // extern "C"
