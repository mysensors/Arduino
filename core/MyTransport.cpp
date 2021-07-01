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
 */

#include "MyTransport.h"

// debug
#if defined(MY_DEBUG_VERBOSE_TRANSPORT)
#define TRANSPORT_DEBUG(x,...) DEBUG_OUTPUT(x, ##__VA_ARGS__)	//!< debug
extern char _convBuf[MAX_PAYLOAD_SIZE * 2 + 1];
#else
#define TRANSPORT_DEBUG(x,...)	//!< debug NULL
#endif


// SM: transitions and update states
static transportState_t stInit = { stInitTransition, stInitUpdate };
static transportState_t stParent = { stParentTransition, stParentUpdate };
static transportState_t stID = { stIDTransition, stIDUpdate };
static transportState_t stUplink = { stUplinkTransition, stUplinkUpdate };
static transportState_t stReady = { stReadyTransition, stReadyUpdate };
static transportState_t stFailure = { stFailureTransition, stFailureUpdate };

// transport SM variables
static transportSM_t _transportSM;

// transport configuration
static transportConfig_t _transportConfig;

// callback transportOk
transportCallback_t _transportReady_cb = NULL;

// enhanced ID assignment
#if !defined(MY_GATEWAY_FEATURE) && (MY_NODE_ID == AUTO)
static uint8_t _transportToken = AUTO;
#endif

// global variables
extern MyMessage _msg;		// incoming message
extern MyMessage _msgTmp;	// outgoing message

#if defined(MY_RAM_ROUTING_TABLE_ENABLED)
static routingTable_t _transportRoutingTable;		//!< routing table
static uint32_t _lastRoutingTableSave;			//!< last routing table dump
#endif

// regular sanity check, activated by default on GW and repeater nodes
#if defined(MY_TRANSPORT_SANITY_CHECK)
static uint32_t _lastSanityCheck;		//!< last sanity check
#endif

// regular network discovery, sends I_DISCOVER_REQUESTS to update routing table
// sufficient to have GW triggering requests to also update repeater nodes
#if defined(MY_GATEWAY_FEATURE)
static uint32_t _lastNetworkDiscovery;	//!< last network discovery
#endif

// stInit: initialise transport HW
void stInitTransition(void)
{
	TRANSPORT_DEBUG(PSTR("TSM:INIT\n"));
	// initialise status variables
	_transportSM.pingActive = false;
	_transportSM.transportActive = false;
	_transportSM.lastUplinkCheck = 0;

#if defined(MY_TRANSPORT_SANITY_CHECK)
	_lastSanityCheck = hwMillis();
#endif
#if defined(MY_GATEWAY_FEATURE)
	_lastNetworkDiscovery = 0;
#endif
#if defined(MY_RAM_ROUTING_TABLE_ENABLED)
	_lastRoutingTableSave = hwMillis();
#endif

	// Read node settings (ID, parent ID, GW distance) from EEPROM
	hwReadConfigBlock((void *)&_transportConfig, (void *)EEPROM_NODE_ID_ADDRESS,
	                  sizeof(transportConfig_t));
}

void stInitUpdate(void)
{
	// initialise radio
	if (!transportHALInit()) {
		TRANSPORT_DEBUG(PSTR("!TSM:INIT:TSP FAIL\n"));
		setIndication(INDICATION_ERR_INIT_TRANSPORT);
		transportSwitchSM(stFailure);
	} else {
		TRANSPORT_DEBUG(PSTR("TSM:INIT:TSP OK\n"));
		_transportSM.transportActive = true;
#if defined (MY_PASSIVE_NODE)
		_transportConfig.passiveMode = true;
		TRANSPORT_DEBUG(PSTR("TSM:INIT:TSP PSM\n"));	// transport passive mode
#else
		_transportConfig.passiveMode = false;
#endif
#if defined(MY_GATEWAY_FEATURE)
		// Set configuration for gateway
		TRANSPORT_DEBUG(PSTR("TSM:INIT:GW MODE\n"));
		_transportConfig.parentNodeId = GATEWAY_ADDRESS;
		_transportConfig.distanceGW = 0u;
		_transportConfig.nodeId = GATEWAY_ADDRESS;
		transportHALSetAddress(GATEWAY_ADDRESS);
		// GW mode: skip FPAR,ID,UPL states
		transportSwitchSM(stReady);
#else
		if (MY_NODE_ID != AUTO) {
			TRANSPORT_DEBUG(PSTR("TSM:INIT:STATID=%" PRIu8 "\n"),(uint8_t)MY_NODE_ID);
			// Set static ID
			_transportConfig.nodeId = (uint8_t)MY_NODE_ID;
			// Save static ID to eeprom (for bootloader)
			hwWriteConfig(EEPROM_NODE_ID_ADDRESS, (uint8_t)MY_NODE_ID);
		}
		// assign ID if set
		if (_transportConfig.nodeId == AUTO || transportAssignNodeID(_transportConfig.nodeId)) {
			// if node ID valid (>0 and <255), proceed to next state
			transportSwitchSM(stParent);
		} else {
			// ID invalid (0 or 255)
			transportSwitchSM(stFailure);
		}
#endif
	}
}

// stParent: find parent
void stParentTransition(void)
{
	TRANSPORT_DEBUG(PSTR("TSM:FPAR\n"));	// find parent
	setIndication(INDICATION_FIND_PARENT);
	_transportSM.uplinkOk = false;
	_transportSM.preferredParentFound = false;
#if defined(MY_PARENT_NODE_IS_STATIC) || defined(MY_PASSIVE_NODE)
	TRANSPORT_DEBUG(PSTR("TSM:FPAR:STATP=%" PRIu8 "\n"), (uint8_t)MY_PARENT_NODE_ID);	// static parent
	_transportSM.findingParentNode = false;
	_transportConfig.distanceGW = 1u;	// assumption, CHKUPL:GWDC will update this variable
	_transportConfig.parentNodeId = (uint8_t)MY_PARENT_NODE_ID;
	// save parent ID to eeprom (for bootloader)
	hwWriteConfig(EEPROM_PARENT_NODE_ID_ADDRESS, (uint8_t)MY_PARENT_NODE_ID);
#else
	_transportSM.findingParentNode = true;
	_transportConfig.distanceGW = DISTANCE_INVALID;	// Set distance to max and invalidate parent node ID
	_transportConfig.parentNodeId = AUTO;
	// Broadcast find parent request
	(void)transportRouteMessage(build(_msgTmp, BROADCAST_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
	                                  I_FIND_PARENT_REQUEST).set(""));
#endif
}

// stParentUpdate
void stParentUpdate(void)
{
#if defined(MY_PARENT_NODE_IS_STATIC) || defined(MY_PASSIVE_NODE)
	// skipping find parent
	setIndication(INDICATION_GOT_PARENT);
	transportSwitchSM(stID);
#else
	if (transportTimeInState() > MY_TRANSPORT_STATE_TIMEOUT_MS || _transportSM.preferredParentFound) {
		// timeout or preferred parent found
		if (_transportConfig.parentNodeId != AUTO) {
			// parent assigned
			TRANSPORT_DEBUG(PSTR("TSM:FPAR:OK\n"));	// find parent ok
			_transportSM.findingParentNode = false;
			setIndication(INDICATION_GOT_PARENT);
			// go to next state
			transportSwitchSM(stID);
		} else {
			// timeout w/o reply or valid parent
			if (_transportSM.stateRetries < MY_TRANSPORT_STATE_RETRIES) {
				// retries left
				TRANSPORT_DEBUG(PSTR("!TSM:FPAR:NO REPLY\n"));		// find parent, no reply
				// reenter state
				transportSwitchSM(stParent);
			} else {
				// no retries left, finding parent failed
				TRANSPORT_DEBUG(PSTR("!TSM:FPAR:FAIL\n"));
				setIndication(INDICATION_ERR_FIND_PARENT);
				transportSwitchSM(stFailure);
			}
		}
	}
#endif
}

// stID: verify and request ID if necessary
void stIDTransition(void)
{
	TRANSPORT_DEBUG(PSTR("TSM:ID\n"));	// verify/request node ID
	if (_transportConfig.nodeId == AUTO) {
		// send ID request
		setIndication(INDICATION_REQ_NODEID);
#if !defined(MY_GATEWAY_FEATURE) && (MY_NODE_ID == AUTO)
		_transportToken = (uint8_t)(hwMillis() & 0xFF);
		if (_transportToken == AUTO) {
			_transportToken++;    // AUTO as token not allowed
		}
		const uint8_t sensorID = _transportToken;
#else
		const uint8_t sensorID = NODE_SENSOR_ID;
#endif
		TRANSPORT_DEBUG(PSTR("TSM:ID:REQ\n"));	// request node ID
		(void)transportRouteMessage(build(_msgTmp, GATEWAY_ADDRESS, sensorID, C_INTERNAL,
		                                  I_ID_REQUEST).set(""));
	}
}

void stIDUpdate(void)
{
	if (_transportConfig.nodeId != AUTO) {
		// current node ID is valid
		TRANSPORT_DEBUG(PSTR("TSM:ID:OK\n"));
		setIndication(INDICATION_GOT_NODEID);
		// proceed to next state
		transportSwitchSM(stUplink);
	} else if (transportTimeInState() > MY_TRANSPORT_STATE_TIMEOUT_MS) {
		// timeout
		if (_transportSM.stateRetries < MY_TRANSPORT_STATE_RETRIES) {
			// retries left: reenter state
			transportSwitchSM(stID);
		} else {
			// no retries left
			TRANSPORT_DEBUG(PSTR("!TSM:ID:FAIL\n"));
			setIndication(INDICATION_ERR_GET_NODEID);
			transportSwitchSM(stFailure);
		}
	}
}

void stUplinkTransition(void)
{
#if !defined(MY_TRANSPORT_UPLINK_CHECK_DISABLED)
	TRANSPORT_DEBUG(PSTR("TSM:UPL\n"));
	setIndication(INDICATION_CHECK_UPLINK);
	_transportSM.pingResponse = INVALID_HOPS;
	_transportSM.pingActive = true;
	(void)transportRouteMessage(build(_msgTmp,GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
	                                  I_PING).set((uint8_t)0x01));
#endif
}

void stUplinkUpdate(void)
{
#if !defined(MY_TRANSPORT_UPLINK_CHECK_DISABLED)
	if (_transportSM.pingResponse != INVALID_HOPS) {
		_transportSM.lastUplinkCheck = hwMillis();
		// uplink ok, i.e. GW replied
		TRANSPORT_DEBUG(PSTR("TSM:UPL:OK\n"));	// uplink ok
		if (_transportSM.pingResponse != _transportConfig.distanceGW) {
			TRANSPORT_DEBUG(PSTR("TSM:UPL:DGWC,O=%" PRIu8 ",N=%" PRIu8 "\n"), _transportConfig.distanceGW,
			                _transportSM.pingResponse);	// distance to GW changed
			_transportConfig.distanceGW = _transportSM.pingResponse;
		}
		transportSwitchSM(stReady);		// proceed to next state
	} else if (transportTimeInState() > MY_TRANSPORT_STATE_TIMEOUT_MS) {
		// timeout
		if (_transportSM.stateRetries < MY_TRANSPORT_STATE_RETRIES) {
			// retries left: reenter state
			transportSwitchSM(stUplink);
		} else {
			// no retries left
			TRANSPORT_DEBUG(PSTR("!TSM:UPL:FAIL\n"));	// uplink check failed
			_transportSM.pingActive = false;
			setIndication(INDICATION_ERR_CHECK_UPLINK);
			transportSwitchSM(stParent);	// go back to stParent
		}
	}
#else
	TRANSPORT_DEBUG(PSTR("TSM:UPL:DISABLED\n"));	// uplink check disabled
	transportSwitchSM(stReady);
#endif
}

void stReadyTransition(void)
{
	// transport is ready and fully operational
	TRANSPORT_DEBUG(PSTR("TSM:READY:ID=%" PRIu8 ",PAR=%" PRIu8 ",DIS=%" PRIu8 "\n"),
	                _transportConfig.nodeId,
	                _transportConfig.parentNodeId, _transportConfig.distanceGW);
	_transportSM.uplinkOk = true;
	_transportSM.failureCounter = 0u;			// reset failure counter
	_transportSM.failedUplinkTransmissions = 0u;	// reset failed uplink TX counter
	// callback
	if (_transportReady_cb) {
		_transportReady_cb();
	}
}

// stReadyUpdate: monitors link
void stReadyUpdate(void)
{
#if defined(MY_GATEWAY_FEATURE)
	if (!_lastNetworkDiscovery ||
	        (hwMillis() - _lastNetworkDiscovery > MY_TRANSPORT_DISCOVERY_INTERVAL_MS)) {
		_lastNetworkDiscovery = hwMillis();
		TRANSPORT_DEBUG(PSTR("TSM:READY:NWD REQ\n"));	// send transport network discovery
		(void)transportRouteMessage(build(_msgTmp, BROADCAST_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
		                                  I_DISCOVER_REQUEST).set(""));
	}
#else
	if (_transportSM.failedUplinkTransmissions > MY_TRANSPORT_MAX_TX_FAILURES) {
		// too many uplink transmissions failed, find new parent (if non-static)
#if !defined(MY_PARENT_NODE_IS_STATIC)
		TRANSPORT_DEBUG(PSTR("!TSM:READY:UPL FAIL,SNP\n"));		// uplink failed, search new parent
		transportSwitchSM(stParent);
#else
		TRANSPORT_DEBUG(PSTR("!TSM:READY:UPL FAIL,STATP\n"));	// uplink failed, static parent
		// reset counter
		_transportSM.failedUplinkTransmissions = 0u;
#endif
	}
#endif

#if defined(MY_RAM_ROUTING_TABLE_ENABLED)
	if (hwMillis() - _lastRoutingTableSave > MY_ROUTING_TABLE_SAVE_INTERVAL_MS) {
		_lastRoutingTableSave = hwMillis();
		transportSaveRoutingTable();
	}
#endif
}

// stFailure: entered upon HW init failure or max retries exceeded
void stFailureTransition(void)
{
	if (_transportSM.failureCounter < MY_TRANSPORT_MAX_TSM_FAILURES) {
		_transportSM.failureCounter++;		// increment consecutive TSM failure counter
	}
	TRANSPORT_DEBUG(PSTR("TSM:FAIL:CNT=%" PRIu8 "\n"),_transportSM.failureCounter);
	_transportSM.uplinkOk = false;			// uplink nok
	_transportSM.transportActive = false;	// transport inactive
	setIndication(INDICATION_ERR_INIT_TRANSPORT);
#if defined(MY_SENSOR_NETWORK)
	TRANSPORT_DEBUG(PSTR("TSM:FAIL:DIS\n"));	// disable transport, no need until re-init
	transportDisable();		// sleep
#endif
}

void stFailureUpdate(void)
{
	if (transportTimeInState() > ( isTransportExtendedFailure()?
	                               MY_TRANSPORT_TIMEOUT_EXT_FAILURE_STATE_MS:
	                               MY_TRANSPORT_TIMEOUT_FAILURE_STATE_MS) ) {
		TRANSPORT_DEBUG(PSTR("TSM:FAIL:RE-INIT\n"));	// attempt to re-initialise transport
		transportSwitchSM(stInit);
	}
}

void transportSwitchSM(transportState_t &newState)
{
	if (_transportSM.currentState != &newState) {
		_transportSM.stateRetries = 0u;	// state change, reset retry counter
		_transportSM.currentState = &newState;	// change state
	} else {
		_transportSM.stateRetries++;	// increment retries
	}
	if (_transportSM.currentState) {
		_transportSM.currentState->Transition();	// State transition
	}
	_transportSM.stateEnter = hwMillis();	// save time
}

uint32_t transportTimeInState(void)
{
	return hwMillis() - _transportSM.stateEnter;
}

void transportUpdateSM(void)
{
	if (_transportSM.currentState) {
		_transportSM.currentState->Update();
	}
}

bool isTransportReady(void)
{
	return _transportSM.uplinkOk;
}

bool isTransportExtendedFailure(void)
{
	return _transportSM.failureCounter == MY_TRANSPORT_MAX_TSM_FAILURES;
}

bool isTransportSearchingParent(void)
{
	return _transportSM.findingParentNode;
}

bool isMessageReceived(void)
{
	return _transportSM.msgReceived;
}

void resetMessageReceived(void)
{
	_transportSM.msgReceived = false;
}

void transportInitialise(void)
{
	_transportSM.failureCounter = 0u;	// reset failure counter
	transportLoadRoutingTable();		// load routing table to RAM (if feature enabled)
	// initial state
	_transportSM.currentState = NULL;
	transportSwitchSM(stInit);
}

void transportDisable(void)
{
	if (RADIO_CAN_POWER_OFF == true) {
		TRANSPORT_DEBUG(PSTR("TSF:TDI:TPD\n"));	// power down transport
		transportHALPowerDown();
	} else {
		TRANSPORT_DEBUG(PSTR("TSF:TDI:TSL\n"));	// send transport to sleep
		transportHALSleep();
	}
}

void transportReInitialise(void)
{
	if (RADIO_CAN_POWER_OFF == true) {
		TRANSPORT_DEBUG(PSTR("TSF:TRI:TPU\n"));		// transport power up
		transportHALPowerUp();
		transportHALSetAddress(_transportConfig.nodeId);
	} else {
		TRANSPORT_DEBUG(PSTR("TSF:TRI:TSB\n"));		// transport standby
		transportHALStandBy();
	}
}

bool transportWaitUntilReady(const uint32_t waitingMS)
{
	// check if transport ready
	TRANSPORT_DEBUG(PSTR("TSF:WUR:MS=%" PRIu32 "\n"), waitingMS);	// timeout
	uint32_t enterMS = hwMillis();
	bool result = false;
	while (!result && ( hwMillis() - enterMS < waitingMS || !waitingMS)) {
		transportProcess();
		result = isTransportReady();
		doYield();
	}
	return result;
}

// update TSM and process incoming messages
void transportProcess(void)
{
	// update state machine
	transportUpdateSM();
	// process transport FIFO
	transportProcessFIFO();
}

bool transportCheckUplink(const bool force)
{
	if (!force && (hwMillis() - _transportSM.lastUplinkCheck) < MY_TRANSPORT_CHKUPL_INTERVAL_MS) {
		TRANSPORT_DEBUG(PSTR("TSF:CKU:OK,FCTRL\n"));	// flood control
		return true;
	}
	// ping GW
	const uint8_t hopsCount = transportPingNode(GATEWAY_ADDRESS);
	// verify hops
	if (hopsCount != INVALID_HOPS) {
		// update
		_transportSM.lastUplinkCheck = hwMillis();
		TRANSPORT_DEBUG(PSTR("TSF:CKU:OK\n"));
		// did distance to GW change upstream, eg. re-routing of uplink nodes
		if (hopsCount != _transportConfig.distanceGW) {
			TRANSPORT_DEBUG(PSTR("TSF:CKU:DGWC,O=%" PRIu8 ",N=%" PRIu8 "\n"), _transportConfig.distanceGW,
			                hopsCount);	// distance to GW changed
			_transportConfig.distanceGW = hopsCount;
		}
		return true;
	} else {
		TRANSPORT_DEBUG(PSTR("TSF:CKU:FAIL\n"));
		return false;
	}
}

bool transportAssignNodeID(const uint8_t newNodeId)
{
	// verify if ID valid
	if (newNodeId != GATEWAY_ADDRESS && newNodeId != AUTO) {
		_transportConfig.nodeId = newNodeId;
		transportHALSetAddress(newNodeId);
		// Write ID to EEPROM
		hwWriteConfig(EEPROM_NODE_ID_ADDRESS, newNodeId);
		TRANSPORT_DEBUG(PSTR("TSF:SID:OK,ID=%" PRIu8 "\n"),newNodeId);	// Node ID assigned
		return true;
	} else {
		TRANSPORT_DEBUG(PSTR("!TSF:SID:FAIL,ID=%" PRIu8 "\n"),newNodeId);	// ID is invalid, cannot assign ID
		setIndication(INDICATION_ERR_NET_FULL);
		return false;
	}
}

bool transportRouteMessage(MyMessage &message)
{
	const uint8_t destination = message.getDestination();

	if (_transportSM.findingParentNode && destination != BROADCAST_ADDRESS) {
		TRANSPORT_DEBUG(PSTR("!TSF:RTE:FPAR ACTIVE\n")); // find parent active, message not sent
		// request to send a non-BC message while finding parent active, abort
		return false;
	}

	uint8_t route;

	if (destination == GATEWAY_ADDRESS) {
		route = _transportConfig.parentNodeId;		// message to GW always routes via parent
	} else if (destination == BROADCAST_ADDRESS) {
		route = BROADCAST_ADDRESS;		// message to BC does not require routing
	} else {
#if defined(MY_REPEATER_FEATURE)
		// destination not GW & not BC, get route
		route = transportGetRoute(destination);
		if (route == AUTO) {
			TRANSPORT_DEBUG(PSTR("!TSF:RTE:%" PRIu8 " UNKNOWN\n"), destination);	// route unknown
#if !defined(MY_GATEWAY_FEATURE)
			if (message.getLast() != _transportConfig.parentNodeId) {
				// message not from parent, i.e. child node - route it to parent
				route = _transportConfig.parentNodeId;
			} else {
				// route unknown and msg received from parent, send it to destination assuming in rx radius
				route = destination;
			}
#else
			// if GW, all unknown destinations are directly addressed
			route = destination;
#endif
		}
#else
#if !defined(MY_TRANSPORT_N2N_FEATURE_DISABLED)
		if (destination > GATEWAY_ADDRESS && destination < BROADCAST_ADDRESS) {
			// node2node traffic: assume node is in vincinity. If transmission fails, hand over to parent
			if (transportSendWrite(destination, message)) {
				TRANSPORT_DEBUG(PSTR("TSF:RTE:N2N OK\n"));
				return true;
			}
			TRANSPORT_DEBUG(PSTR("!TSF:RTE:N2N FAIL\n"));
		}
#else
		TRANSPORT_DEBUG(PSTR("TSF:RTE:N2N DIS\n"));
#endif
		route = _transportConfig.parentNodeId;	// not a repeater, all traffic routed via parent
#endif
	}
	// send message
	const bool result = transportSendWrite(route, message);
#if !defined(MY_GATEWAY_FEATURE)
	// update counter
	if (route == _transportConfig.parentNodeId) {

		if (!result) {
			setIndication(INDICATION_ERR_TX);
			_transportSM.failedUplinkTransmissions++;
		} else {
			_transportSM.failedUplinkTransmissions = 0u;
#if defined(MY_SIGNAL_REPORT_ENABLED)
			// update uplink quality monitor
			const int16_t signalStrengthRSSI = transportGetSignalReport(SR_TX_RSSI);
			_transportSM.uplinkQualityRSSI = static_cast<transportRSSI_t>((1 - UPLINK_QUALITY_WEIGHT) *
			                                 _transportSM.uplinkQualityRSSI
			                                 + (UPLINK_QUALITY_WEIGHT * transportRSSItoInternal(signalStrengthRSSI)));
#endif
		}
	}
#else
	if(!result) {
		setIndication(INDICATION_ERR_TX);
	}
#endif

	return result;
}

bool transportSendRoute(MyMessage &message)
{
	bool result = false;
	if (isTransportReady()) {
		result = transportRouteMessage(message);
	} else {
		// TNR: transport not ready
		TRANSPORT_DEBUG(PSTR("!TSF:SND:TNR\n"));
	}
	return result;
}

// only be used inside transport
bool transportWait(const uint32_t waitingMS, const uint8_t cmd, const uint8_t msgType)
{
	const uint32_t enterMS = hwMillis();
	// invalidate msg type
	_msg.setType(!msgType);
	bool expectedResponse = false;
	while ((hwMillis() - enterMS < waitingMS) && !expectedResponse) {
		// process incoming messages
		transportProcessFIFO();
		doYield();
		expectedResponse = (_msg.getCommand() == cmd && _msg.getType() == msgType);
	}
	return expectedResponse;
}

uint8_t transportPingNode(const uint8_t targetId)
{
	if(!_transportSM.pingActive) {
		TRANSPORT_DEBUG(PSTR("TSF:PNG:SEND,TO=%" PRIu8 "\n"), targetId);
		if(targetId == _transportConfig.nodeId) {
			// pinging self
			_transportSM.pingResponse = 0u;
		} else {
			_transportSM.pingActive = true;
			_transportSM.pingResponse = INVALID_HOPS;
			(void)transportRouteMessage(build(_msgTmp, targetId, NODE_SENSOR_ID, C_INTERNAL,
			                                  I_PING).set((uint8_t)0x01));
			// Wait for ping reply or timeout
			(void)transportWait(2000, C_INTERNAL, I_PONG);
		}
		// make sure missing I_PONG msg does not block pinging function by leaving pingActive=true
		_transportSM.pingActive = false;
		return _transportSM.pingResponse;
	} else {
		TRANSPORT_DEBUG(PSTR("!TSF:PNG:ACTIVE\n"));	// ping active, cannot start new ping
		return INVALID_HOPS;
	}
}

uint32_t transportGetHeartbeat(void)
{
	return transportTimeInState();
}

void transportProcessMessage(void)
{
	// Manage signing timeout
	(void)signerCheckTimer();
	// receive message
	setIndication(INDICATION_RX);
	uint8_t payloadLength;
	// last is the first byte of the payload buffer
	if (!transportHALReceive(&_msg, &payloadLength)) {
		return;
	}
	// get message length and limit size
	const uint8_t msgLength = _msg.getLength();
	// calculate expected length

	const uint8_t command = _msg.getCommand();
	const uint8_t type = _msg.getType();
	const uint8_t sender = _msg.getSender();
	const uint8_t last = _msg.getLast();
	const uint8_t destination = _msg.getDestination();

	TRANSPORT_DEBUG(PSTR("TSF:MSG:READ,%" PRIu8 "-%" PRIu8 "-%" PRIu8 ",s=%" PRIu8 ",c=%" PRIu8 ",t=%"
	                     PRIu8 ",pt=%" PRIu8 ",l=%" PRIu8 ",sg=%" PRIu8 ":%s\n"),
	                sender, last, destination, _msg.getSensor(), command, type, _msg.getPayloadType(), msgLength,
	                _msg.getSigned(), ((command == C_INTERNAL &&
	                                    type == I_NONCE_RESPONSE) ? "<NONCE>" : _msg.getString(_convBuf)));

	// Reject messages that do not pass verification
	if (!signerVerifyMsg(_msg)) {
		setIndication(INDICATION_ERR_SIGN);
		TRANSPORT_DEBUG(PSTR("!TSF:MSG:SIGN VERIFY FAIL\n"));
		return;
	}

	// update routing table if msg not from parent
#if defined(MY_REPEATER_FEATURE)
#if !defined(MY_GATEWAY_FEATURE)
	if (last != _transportConfig.parentNodeId) {
#else
	// GW doesn't have parent
	{
#endif
		// Message is from one of the child nodes and not sent from this node. Add it to routing table.
		if (sender != _transportConfig.nodeId)
		{
			transportSetRoute(sender, last);
		}
	}
#endif // MY_REPEATER_FEATURE

	// set message received flag
	_transportSM.msgReceived = true;

	// Is message addressed to this node?
	if (destination == _transportConfig.nodeId) {
		// null terminate data
		_msg.data[msgLength] = 0u;
		// Check if sender requests an echo.
		if (_msg.getRequestEcho()) {
			TRANSPORT_DEBUG(PSTR("TSF:MSG:ECHO REQ\n"));	// ECHO requested
			_msgTmp = _msg;	// Copy message
			// Reply without echo flag (otherwise we would end up in an eternal loop)
			_msgTmp.setRequestEcho(false);
			_msgTmp.setEcho(true); // set ECHO flag
			_msgTmp.setSender(_transportConfig.nodeId);
			_msgTmp.setDestination(sender);
			// send ECHO, use transportSendRoute since ECHO reply is not internal, i.e. if !transportOK do not reply
			(void)transportSendRoute(_msgTmp);
		}
		if(!_msg.isEcho()) {
			// only process if not ECHO
			if (command == C_INTERNAL) {
				// Process signing related internal messages
				if (signerProcessInternal(_msg)) {
					return; // Signer processing indicated no further action needed
				}
#if !defined(MY_GATEWAY_FEATURE)
				if (type == I_ID_RESPONSE) {
#if (MY_NODE_ID == AUTO)
					// only active if node ID dynamic
					if ((_msg.getSensor() == _transportToken) || (_msg.getSensor() == AUTO)) {
						(void)transportAssignNodeID(_msg.getByte());
					} else {
						TRANSPORT_DEBUG(PSTR("!TSF:MSG:ID TK INVALID\n"));
					}
#endif
					return; // no further processing required
				}
				if (type == I_FIND_PARENT_RESPONSE) {
#if !defined(MY_GATEWAY_FEATURE) && !defined(MY_PARENT_NODE_IS_STATIC)
					if (_transportSM.findingParentNode) {	// only process if find parent active
						// Reply to a I_FIND_PARENT_REQUEST message. Check if the distance is shorter than we already have.
						uint8_t distance = _msg.getByte();
						if (isValidDistance(distance)) {
							distance++;	// Distance to gateway is one more for us w.r.t. parent
							// update settings if distance shorter or preferred parent found
							if (((isValidDistance(distance) && distance < _transportConfig.distanceGW) || (!_autoFindParent &&
							        sender == (uint8_t)MY_PARENT_NODE_ID)) && !_transportSM.preferredParentFound) {
								// Found a neighbor closer to GW than previously found
								if (!_autoFindParent && sender == (uint8_t)MY_PARENT_NODE_ID) {
									_transportSM.preferredParentFound = true;
									TRANSPORT_DEBUG(PSTR("TSF:MSG:FPAR PREF\n"));	// find parent, preferred parent found
								}
								_transportConfig.distanceGW = distance;
								_transportConfig.parentNodeId = sender;
								TRANSPORT_DEBUG(PSTR("TSF:MSG:FPAR OK,ID=%" PRIu8 ",D=%" PRIu8 "\n"), _transportConfig.parentNodeId,
								                _transportConfig.distanceGW);
							}
						}
					} else {
						TRANSPORT_DEBUG(PSTR("!TSF:MSG:FPAR INACTIVE\n"));	// find parent response received, but inactive
					}
					return; // no further processing required
#endif
				}
#endif // !defined(MY_GATEWAY_FEATURE)
				// general
				if (type == I_PING) {
					TRANSPORT_DEBUG(PSTR("TSF:MSG:PINGED,ID=%" PRIu8 ",HP=%" PRIu8 "\n"), sender,
					                _msg.getByte()); // node pinged
#if defined(MY_GATEWAY_FEATURE) && (F_CPU>16000000)
					// delay for fast GW and slow nodes
					delay(5);
#endif
					(void)transportRouteMessage(build(_msgTmp, sender, NODE_SENSOR_ID, C_INTERNAL,
					                                  I_PONG).set((uint8_t)1));
					return; // no further processing required
				}
				if (type == I_PONG) {
					if (_transportSM.pingActive) {
						_transportSM.pingActive = false;
						_transportSM.pingResponse = _msg.getByte();
						TRANSPORT_DEBUG(PSTR("TSF:MSG:PONG RECV,HP=%" PRIu8 "\n"),
						                _transportSM.pingResponse); // pong received
					} else {
						TRANSPORT_DEBUG(PSTR("!TSF:MSG:PONG RECV,INACTIVE\n")); // pong received, but !pingActive
					}
					return; // no further processing required
				}
				if (type == I_SIGNAL_REPORT_REVERSE) {
					return; // no further processing required
				}
				if (type == I_SIGNAL_REPORT_REQUEST) {
					int16_t value = INVALID_RSSI;
#if defined(MY_SIGNAL_REPORT_ENABLED)
					const char internalCommand = _msg.data[0];
					if (_msg.data[1] != '!') {
						value = transportSignalReport(internalCommand);
					} else {
						// send request
						if (transportRouteMessage(build(_msgTmp, _msg.getLast(), NODE_SENSOR_ID, C_INTERNAL,
						                                I_SIGNAL_REPORT_REVERSE).set((uint8_t)255))) {
							// S>s, R>r, ascii delta = 32
							value = transportSignalReport(internalCommand + 32);	// reverse
						};
					}
#endif
					(void)transportRouteMessage(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
					                                  I_SIGNAL_REPORT_RESPONSE).set(value));
					return; // no further processing required
				}
				if (_processInternalCoreMessage()) {
					return; // no further processing required
				}
			} else if (command == C_STREAM) {
#if defined(MY_OTA_FIRMWARE_FEATURE)
				if(firmwareOTAUpdateProcess()) {
					return; // OTA FW update processing indicated no further action needed
				}
#endif
			}
		} else {
			TRANSPORT_DEBUG(
			    PSTR("TSF:MSG:ECHO\n")); // received message is ECHO, no internal processing, handover to msg callback
		}
#if defined(MY_OTA_LOG_RECEIVER_FEATURE)
		if ((type == I_LOG_MESSAGE) && (command == C_INTERNAL)) {
			OTALogPrint(_msg);
			return; // no further processing required
		}
#endif //defined(MY_OTA_LOG_RECEIVER_FEATURE)
#if defined(MY_GATEWAY_FEATURE)
		// Hand over message to controller
		(void)gatewayTransportSend(_msg);
#endif
		// Call incoming message callback if available
		if (receive) {
			receive(_msg);
		}
	} else if (destination == BROADCAST_ADDRESS) {
		TRANSPORT_DEBUG(PSTR("TSF:MSG:BC\n"));	// broadcast msg
		if (command == C_INTERNAL) {
			if (isTransportReady()) {
				// only reply if node is fully operational
				if (type == I_FIND_PARENT_REQUEST) {
#if defined(MY_REPEATER_FEATURE)
					if (sender != _transportConfig.parentNodeId) {	// no circular reference
						TRANSPORT_DEBUG(PSTR("TSF:MSG:FPAR REQ,ID=%" PRIu8 "\n"), sender);	// FPAR: find parent request
						// check if uplink functional - node can only be parent node if link to GW functional
						// this also prevents circular references in case GW ooo
						if (transportCheckUplink()) {
							_transportSM.lastUplinkCheck = hwMillis();
							TRANSPORT_DEBUG(PSTR("TSF:MSG:GWL OK\n")); // GW uplink ok
							// random delay minimizes collisions
							delay(hwMillis() & 0x3ff);
							(void)transportRouteMessage(build(_msgTmp, sender, NODE_SENSOR_ID, C_INTERNAL,
							                                  I_FIND_PARENT_RESPONSE).set(_transportConfig.distanceGW));
						} else {
							TRANSPORT_DEBUG(PSTR("!TSF:MSG:GWL FAIL\n")); // GW uplink fail, do not respond to parent request
						}
					}
#endif
					return; // no further processing required, do not forward
				}
			} // isTransportReady
			if (type == I_FIND_PARENT_RESPONSE) {
				return;	// no further processing required, do not forward
			}
#if !defined(MY_GATEWAY_FEATURE)
			if (type == I_DISCOVER_REQUEST) {
				if (last == _transportConfig.parentNodeId) {
					// random wait to minimize collisions
					delay(hwMillis() & 0x3ff);
					(void)transportRouteMessage(build(_msgTmp, sender, NODE_SENSOR_ID, C_INTERNAL,
					                                  I_DISCOVER_RESPONSE).set(_transportConfig.parentNodeId));
					// no return here (for fwd if repeater)
				}
			}
#endif
		}
		// controlled BC relay
#if defined(MY_REPEATER_FEATURE)
		// controlled BC repeating: forward only if message received from parent and sender not self to prevent circular fwds
		if(last == _transportConfig.parentNodeId && sender != _transportConfig.nodeId &&
		        isTransportReady()) {
			TRANSPORT_DEBUG(PSTR("TSF:MSG:FWD BC MSG\n")); // controlled broadcast msg forwarding
			(void)transportRouteMessage(_msg);
		}
#endif

		// Callback for BC, only for non-internal messages
		if (command != C_INTERNAL) {
#if !defined(MY_GATEWAY_FEATURE)
			// only proceed if message received from parent
			if (last != _transportConfig.parentNodeId) {
				return;
			}
#endif
#if defined(MY_GATEWAY_FEATURE)
			// Hand over message to controller
			(void)gatewayTransportSend(_msg);
#endif
			if (receive) {
				TRANSPORT_DEBUG(PSTR("TSF:MSG:RCV CB\n")); // hand over message to receive callback function
				receive(_msg);
			}
		}

	} else {
		// msg not to us and not BC, relay msg
#if defined(MY_REPEATER_FEATURE)
		if (isTransportReady()) {
			if (command == C_INTERNAL) {
				if (type == I_PING || type == I_PONG) {
					uint8_t hopsCnt = _msg.getByte();
					TRANSPORT_DEBUG(PSTR("TSF:MSG:REL PxNG,HP=%" PRIu8 "\n"), hopsCnt);
					if (hopsCnt != MAX_HOPS) {
						hopsCnt++;
						_msg.set(hopsCnt);
					}
				}
			}
			// Relay this message to another node
			TRANSPORT_DEBUG(PSTR("TSF:MSG:REL MSG\n"));	// relay msg
			(void)transportRouteMessage(_msg);
		}
#else
		TRANSPORT_DEBUG(PSTR("!TSF:MSG:REL MSG,NREP\n"));	// message relaying request, but not a repeater
#endif
	}
	(void)last;	//avoid cppcheck warning
}

void transportInvokeSanityCheck(void)
{
	// Suppress this because the function may return a variable value in some configurations
	// cppcheck-suppress knownConditionTrueFalse
	if (!transportHALSanityCheck()) {
		TRANSPORT_DEBUG(PSTR("!TSF:SAN:FAIL\n"));	// sanity check fail
		transportSwitchSM(stFailure);
	} else {
		TRANSPORT_DEBUG(PSTR("TSF:SAN:OK\n"));		// sanity check ok
	}
}

void transportProcessFIFO(void)
{
	if (!_transportSM.transportActive) {
		// transport not active, no further processing required
		return;
	}

#if defined(MY_TRANSPORT_SANITY_CHECK)
	if (hwMillis() - _lastSanityCheck > MY_TRANSPORT_SANITY_CHECK_INTERVAL_MS) {
		_lastSanityCheck = hwMillis();
		transportInvokeSanityCheck();
	}
#endif

	uint8_t _processedMessages = MAX_SUBSEQ_MSGS;
	// process all msgs in FIFO or counter exit
	while (transportHALDataAvailable() && _processedMessages--) {
		transportProcessMessage();
	}
#if defined(MY_OTA_FIRMWARE_FEATURE)
	if (isTransportReady()) {
		// only process if transport ok
		firmwareOTAUpdateRequest();
	}
#endif
}

bool transportSendWrite(const uint8_t to, MyMessage &message)
{
	message.setLast(_transportConfig.nodeId); // Update last

	// sign message if required
	if (!signerSignMsg(message)) {
		TRANSPORT_DEBUG(PSTR("!TSF:MSG:SIGN FAIL\n"));
		setIndication(INDICATION_ERR_SIGN);
		return false;
	}

	// msg length changes if signed
	const uint8_t totalMsgLength = HEADER_SIZE + ( message.getSigned() ? MAX_PAYLOAD_SIZE :
	                               message.getLength() );
	const bool noACK = _transportConfig.passiveMode || (to == BROADCAST_ADDRESS);
	// send
	setIndication(INDICATION_TX);
	const bool result = transportHALSend(to, &message, totalMsgLength,
	                                     noACK);

	TRANSPORT_DEBUG(PSTR("%sTSF:MSG:SEND,%" PRIu8 "-%" PRIu8 "-%" PRIu8 "-%" PRIu8 ",s=%" PRIu8 ",c=%"
	                     PRIu8 ",t=%" PRIu8 ",pt=%" PRIu8 ",l=%" PRIu8 ",sg=%" PRIu8 ",ft=%" PRIu8 ",st=%s:%s\n"),
	                (noACK ? "?" : result ? "" : "!"), message.getSender(), message.getLast(),
	                to,
	                message.getDestination(),
	                message.getSensor(),
	                message.getCommand(), message.getType(),
	                message.getPayloadType(), message.getLength(), message.getSigned(),
	                _transportSM.failedUplinkTransmissions,
	                (result ? "OK" : "NACK"),
	                ((message.getCommand() == C_INTERNAL &&
	                  message.getType() == I_NONCE_RESPONSE) ? "<NONCE>" : message.getString(_convBuf)));

	return result;
}

void transportRegisterReadyCallback(transportCallback_t cb)
{
	_transportReady_cb = cb;
}

uint8_t transportGetNodeId(void)
{
	return _transportConfig.nodeId;
}
uint8_t transportGetParentNodeId(void)
{
	return _transportConfig.parentNodeId;
}
uint8_t transportGetDistanceGW(void)
{
	return _transportConfig.distanceGW;
}


void transportClearRoutingTable(void)
{
	for (uint16_t i = 0; i < SIZE_ROUTES; i++) {
		transportSetRoute((uint8_t)i, BROADCAST_ADDRESS);
	}
	transportSaveRoutingTable();	// save cleared routing table to EEPROM (if feature enabled)
	TRANSPORT_DEBUG(PSTR("TSF:CRT:OK\n"));	// clear routing table
}

void transportLoadRoutingTable(void)
{
#if defined(MY_RAM_ROUTING_TABLE_ENABLED)
	hwReadConfigBlock((void*)&_transportRoutingTable.route, (void*)EEPROM_ROUTES_ADDRESS, SIZE_ROUTES);
	TRANSPORT_DEBUG(PSTR("TSF:LRT:OK\n"));	//  load routing table
#endif
}

void transportSaveRoutingTable(void)
{
#if defined(MY_RAM_ROUTING_TABLE_ENABLED)
	hwWriteConfigBlock((void*)&_transportRoutingTable.route, (void*)EEPROM_ROUTES_ADDRESS, SIZE_ROUTES);
	TRANSPORT_DEBUG(PSTR("TSF:SRT:OK\n"));	//  save routing table
#endif
}

void transportSetRoute(const uint8_t node, const uint8_t route)
{
#if defined(MY_RAM_ROUTING_TABLE_ENABLED)
	_transportRoutingTable.route[node] = route;
#else
	hwWriteConfig(EEPROM_ROUTES_ADDRESS + node, route);
#endif
}

uint8_t transportGetRoute(const uint8_t node)
{
	uint8_t result;
#if defined(MY_RAM_ROUTING_TABLE_ENABLED)
	result = _transportRoutingTable.route[node];
#else
	result = hwReadConfig(EEPROM_ROUTES_ADDRESS + node);
#endif
	return result;
}

void transportReportRoutingTable(void)
{
#if defined(MY_REPEATER_FEATURE)
	for (uint16_t cnt = 0; cnt < SIZE_ROUTES; cnt++) {
		const uint8_t route = transportGetRoute(cnt);
		if (route != BROADCAST_ADDRESS) {
			TRANSPORT_DEBUG(PSTR("TSF:RRT:ROUTE N=%" PRIu8 ",R=%" PRIu8 "\n"), cnt, route);
			uint8_t outBuf[2] = { (uint8_t)cnt,route };
			(void)_sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_DEBUG).set(outBuf,
			                 2));
			wait(200);
		}
	}
#endif
}

void transportTogglePassiveMode(const bool OnOff)
{
#if !defined (MY_PASSIVE_NODE)
	_transportConfig.passiveMode = OnOff;
#else
	(void)OnOff;
#endif
}

int16_t transportGetSignalReport(const signalReport_t signalReport)
{
#if defined(MY_SIGNAL_REPORT_ENABLED)
	int16_t result;
	switch (signalReport) {
	case SR_RX_RSSI:
		result = transportHALGetReceivingRSSI();
		break;
	case SR_TX_RSSI:
		result = transportHALGetSendingRSSI();
		break;
	case SR_RX_SNR:
		result = transportHALGetReceivingSNR();
		break;
	case SR_TX_SNR:
		result = transportHALGetSendingSNR();
		break;
	case SR_TX_POWER_LEVEL:
		result = transportHALGetTxPowerLevel();
		break;
	case SR_TX_POWER_PERCENT:
		result = transportHALGetTxPowerPercent();
		break;
	case SR_UPLINK_QUALITY:
		result = transportInternalToRSSI(_transportSM.uplinkQualityRSSI);
		break;
	default:
		result = 0;
		break;
	}
	return result;
#else
	(void)signalReport;
	return 0;
#endif
}

int16_t transportSignalReport(const char command)
{
#if defined(MY_SIGNAL_REPORT_ENABLED)
	signalReport_t reportCommand;
	switch (command) {
	case 'S':
		// SNR (if available) of incoming message
		reportCommand = SR_RX_SNR;
		break;
	case 's':
		// SNR (if available) of incoming ACK message
		reportCommand = SR_TX_SNR;
		break;
	case 'R':
		// RSSI (if available) of incoming message
		reportCommand = SR_RX_RSSI;
		break;
	case 'r':
		// RSSI (if available) of incoming ACK message
		reportCommand = SR_TX_RSSI;
		break;
	case 'P':
		// TX powerlevel in %
		reportCommand = SR_TX_POWER_PERCENT;
		break;
	case 'T':
		// TX powerlevel in dBm
		reportCommand = SR_TX_POWER_LEVEL;
		break;
	case 'U':
		// Uplink quality
		reportCommand = SR_UPLINK_QUALITY;
		break;
	default:
		reportCommand = SR_NOT_DEFINED;
		break;
	}
	const uint16_t result = transportGetSignalReport(reportCommand);
	TRANSPORT_DEBUG(PSTR("TSF:SIR:CMD=%" PRIu8 ",VAL=%" PRIu16 "\n"), reportCommand, result);
	return result;
#else
	(void)command;
	return 0;
#endif
}
