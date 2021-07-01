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

/**
* @file MyTransport.h
*
* @defgroup MyTransportgrp MyTransport
* @ingroup internals
* @{
*
* Transport-related log messages, format: [!]SYSTEM:[SUB SYSTEM:]MESSAGE
* - [!] Exclamation mark is prepended in case of error
* - SYSTEM:
*  - <b>TSM</b>: messages emitted by the transport state machine
*  - <b>TSF</b>: messages emitted by transport support functions
* - SUB SYSTEMS:
*  - Transport state machine (<b>TSM</b>)
*   - TSM:<b>INIT</b>		from <b>stInit</b> Initialize transport and radio
*   - TSM:<b>FPAR</b>		from <b>stParent</b> Find parent
*   - TSM:<b>ID</b>			from <b>stID</b> Check/request node ID, if dynamic node ID set
*   - TSM:<b>UPL</b>		from <b>stUplink</b> Verify uplink connection by pinging GW
*   - TSM:<b>READY</b>	from <b>stReady</b> Transport is ready and fully operational
*   - TSM:<b>FAIL</b>		from <b>stFailure</b> Failure in transport link or transport HW
*  - Transport support function (<b>TSF</b>)
*   - TSF:<b>CKU</b>		from @ref transportCheckUplink(), checks connection to GW
*   - TSF:<b>SID</b>		from @ref transportAssignNodeID(), assigns node ID
*   - TSF:<b>PNG</b>		from @ref transportPingNode(), pings a node
*   - TSF:<b>WUR</b>		from @ref transportWaitUntilReady(), waits until transport is ready
*   - TSF:<b>CRT</b>		from @ref transportClearRoutingTable(), clears routing table stored in EEPROM
*   - TSF:<b>LRT</b>		from @ref transportLoadRoutingTable(), loads RAM routing table from EEPROM (only GW/repeaters)
*   - TSF:<b>SRT</b>		from @ref transportSaveRoutingTable(), saves RAM routing table to EEPROM (only GW/repeaters)
*   - TSF:<b>MSG</b>		from @ref transportProcessMessage(), processes incoming message
*   - TSF:<b>SAN</b>		from @ref transportInvokeSanityCheck(), calls transport-specific sanity check
*   - TSF:<b>RTE</b>		from @ref transportRouteMessage(), sends message
*   - TSF:<b>SND</b>		from @ref transportSendRoute(), sends message if transport is ready (exposed)
*   - TSF:<b>TDI</b>		from @ref transportDisable()
*   - TSF:<b>TRI</b>		from @ref transportReInitialise()
*   - TSF:<b>SIR</b>		from @ref transportSignalReport()
*
* Transport debug log messages:
*
* |E| SYS | SUB   | Message										| Comment
* |-|-----|-------|---------------------------|---------------------------------------------------------------------
* | | TSM | INIT  |														| <b>Transition to stInit state</b>
* | | TSM | INIT  | STATID=%%d								| Node ID is static
* | | TSM | INIT  | TSP OK										| Transport device configured and fully operational
* | | TSM | INIT  | TSP PSM										| Transport passive mode set
* | | TSM | INIT  | GW MODE										| Node is set up as GW, thus omitting ID and findParent states
* |!| TSM | INIT  | TSP FAIL									| Transport device initialization failed
* | | TSM | FPAR  |														| <b>Transition to stParent state</b>
* | | TSM | FPAR  | STATP=%%d									| Static parent set, skip finding parent
* | | TSM | FPAR  | OK												| Parent node identified
* |!| TSM | FPAR  | NO REPLY									| No potential parents replied to find parent request
* |!| TSM | FPAR  | FAIL											| Finding parent failed
* | | TSM | ID    |														| <b>Transition to stID state</b>
* | | TSM | ID    | OK,ID=%%d									| Node ID is valid
* | | TSM | ID    | REQ												| Request node ID from controller
* |!| TSM | ID    | FAIL,ID=%%d								| ID verification failed, ID invalid, no ID received from controller
* | | TSM | UPL   |														| <b>Transition to stUplink state</b>
* | | TSM | UPL   | OK												| Uplink OK, GW returned ping
* | | TSF | UPL   | DGWC,O=%%d,N=%%d					| Uplink check revealed changed network topology, old distance (O), new distance (N)
* |!| TSM | UPL   | FAIL											| Uplink check failed, i.e. GW could not be pinged
* | | TSM | READY | SRT												| Save routing table
* | | TSM | READY | ID=%%d,PAR=%%d,DIS=%%d		| <b>Transition to stReady</b> Transport ready, node ID (ID), parent node ID (PAR), distance to GW (DIS)
* |!| TSM | READY | UPL FAIL,SNP							| Too many failed uplink transmissions, search new parent
* |!| TSM | READY | FAIL,STATP								| Too many failed uplink transmissions, static parent enforced
* | | TSM | FAIL  | CNT=%%d										| <b>Transition to stFailure state</b>, consecutive failure counter (CNT)
* | | TSM | FAIL  | DIS												| Disable transport
* | | TSM | FAIL  | RE-INIT										| Attempt to re-initialize transport
* | | TSF | CKU   | OK												| Uplink OK
* | | TSF | CKU   | OK,FCTRL									| Uplink OK, flood control prevents pinging GW in too short intervals
* | | TSF | CKU   | DGWC,O=%%d,N=%%d					| Uplink check revealed changed network topology, old distance (O), new distance (N)
* | | TSF | CKU   | FAIL											| No reply received when checking uplink
* | | TSF | SID   | OK,ID=%%d									| Node ID assigned
* |!| TSF | SID   | FAIL,ID=%%d								| Assigned ID is invalid
* | | TSF | PNG   | SEND,TO=%%d								| Send ping to destination (TO)
* | | TSF | WUR   | MS=%%lu										| Wait until transport ready, timeout (MS)
* | | TSF | MSG   | ECHO REQ										| ECHO message requested
* | | TSF | MSG   | ECHO												| ECHO message, do not proceed but forward to callback
* | | TSF | MSG   | FPAR RES,ID=%%d,D=%%d			| Response to find parent received from node (ID) with distance (D) to GW
* | | TSF | MSG   | FPAR PREF FOUND						| Preferred parent found, i.e. parent defined via MY_PARENT_NODE_ID
* | | TSF | MSG   | FPAR OK,ID=%%d,D=%%d			| Find parent response from node (ID) is valid, distance (D) to GW
* | | TSF | MSG   | FPAR INACTIVE							| Find parent response received, but no find parent request active, skip response
* | | TSF | MSG   | FPAR REQ,ID=%%d						| Find parent request from node (ID)
* | | TSF | MSG   | PINGED,ID=%%d,HP=%%d			| Node pinged by node (ID) with (HP) hops
* | | TSF | MSG   | PONG RECV,HP=%%d					| Pinged node replied with (HP) hops
* | | TSF | MSG   | BC												| Broadcast message received
* | | TSF | MSG   | GWL OK										| Link to GW ok
* | | TSF | MSG   | FWD BC MSG								| Controlled broadcast message forwarding
* | | TSF | MSG   | RCV CB										| Hand over message to @ref receive() callback function
* | | TSF | MSG   | REL MSG										| Relay message
* | | TSF | MSG   | REL PxNG,HP=%%d						| Relay PING/PONG message, increment hop counter (HP)
* |!| TSF | MSG   | SIGN VERIFY FAIL					| Signing verification failed
* |!| TSF | MSG   | REL MSG,NORP							| Node received a message for relaying, but node is not a repeater, message skipped
* |!| TSF | MSG   | SIGN FAIL									| Signing message failed
* |!| TSF | MSG   | GWL FAIL									| GW uplink failed
* |!| TSF | MSG   | ID TK INVALID							| Token for ID request invalid
* | | TSF | SAN   | OK												| Sanity check passed
* |!| TSF | SAN   | FAIL											| Sanity check failed, attempt to re-initialize radio
* | | TSF | CRT   | OK												| Clearing routing table successful
* | | TSF | LRT   | OK												| Loading routing table successful
* | | TSF | SRT   | OK												| Saving routing table successful
* |!| TSF | RTE   | FPAR ACTIVE								| Finding parent active, message not sent
* |!| TSF | RTE   | DST %%d UNKNOWN						| Routing for destination (DST) unknown, send message to parent
* | | TSF | RTE   | N2N OK										| Node-to-node communication succeeded
* | | TSF | RTE   | N2N DIS										| Node-to-node communication disabled by configuration
* |!| TSF | RTE   | N2N FAIL									| Node-to-node communication failed, handing over to parent for re-routing
* | | TSF | RRT   | ROUTE N=%%d,R=%%d					| Routing table, messages to node (N) are routed via node (R)
* |!| TSF | SND   | TNR												| Transport not ready, message cannot be sent
* | | TSF | TDI   | TSL												| Set transport to sleep
* | | TSF | TDI   | TPD												| Power down transport
* | | TSF | TRI   | TRI												| Reinitialise transport
* | | TSF | TRI   | TSB												| Set transport to standby
* | | TSF | SIR   | CMD=%d,VAL=%d							| Get signal report
*
*
* Incoming / outgoing messages:
*
* See <a href="https://www.mysensors.org/download/serial_api_20">here</a> for more detail on the format and definitions.
*
* Receiving a message
* - TSF:MSG:READ,sender-last-destination,s=%%d,c=%%d,t=%%d,pt=%%d,l=%%d,sg=%%d:%%s
*
* Sending a message
* - [!/?]TSF:MSG:SEND,sender-last-next-destination,s=%%d,c=%%d,t=%%d,pt=%%d,l=%%d,sg=%%d,ft=%%d,st=%%s:%%s
*
* Prepended char:
* - <b>none</b>=sending OK
* - <b>!</b>=error sending
* - <b>?</b>=sending status unknown
* Message fields:
* - <b>s</b>=sensor ID
* - <b>c</b>=command
* - <b>t</b>=msg type
* - <b>pt</b>=payload type
* - <b>l</b>=length
* - <b>sg</b>=signing flag
* - <b>ft</b>=failed uplink transmission counter
* - <b>st</b>=send status, OK=success, NACK=no radio ACK received
*
* @startuml
* state top as "Transport" {
* state Init
* state Failure
* state Ready
* state Parent
* state ID
* state Uplink
* }
*
* [*] --> Init
* Init : entry / Read config from eeprom
* Init --> Failure : [! transportInit()\n|| ID == 0\n|| ID == 255 ]
* Init --> Ready : [MY_GATEWAY_FEATURE]
* Init --> Parent : [else]
*
* Parent : entry / Broadcast Find Parent
* Parent --> ID : [MY_PARENT_NODE_IS_STATIC\n|| MY_PASSIVE_NODE\n|| Parent found]
* Parent --> Parent : [timeout\n&& retries left]
* Parent --> Failure : [timeout\n&& no retries left]
*
* ID : entry / Request Node ID
* ID --> Uplink : [ID valid]
* ID --> ID : [timeout\n&& retries left]
* ID --> Failure : [timeout\n&& no retries left]
*
* Uplink : entry / Check uplink (PING)
* Uplink --> Uplink : [timeout\n&& retries left]
* Uplink --> Parent : [timeout\n&& no retries left]
* Uplink --> Ready : [MY_TRANSPORT_UPLINK_CHECK_DISABLED\n|| Uplink ok (PONG)]
*
* Ready : entry / Transport ready callback
* Ready : MY_GATEWAY_FEATURE && Network discovery required / Send discovery
* Ready --> Parent : [!MY_PARENT_NODE_IS_STATIC\n&& Uplink failure overflow]
*
* Failure : entry / Disable transport
* Failure --> Init : [timeout]
* top --> Failure : [MY_TRANSPORT_SANITY_CHECK\n&& !transportSanityCheck]
* @enduml
*
* @brief API declaration for MyTransport
*
*/

#ifndef MyTransport_h
#define MyTransport_h

#include "hal/transport/MyTransportHAL.h"

#ifndef MY_TRANSPORT_MAX_TX_FAILURES
#if defined(MY_REPEATER_FEATURE)
#define MY_TRANSPORT_MAX_TX_FAILURES	(10u)		//!< search for a new parent node after this many transmission failures, higher threshold for repeating nodes
#else
#define MY_TRANSPORT_MAX_TX_FAILURES	(5u)		//!< search for a new parent node after this many transmission failures, lower threshold for non-repeating nodes
#endif
#endif

#ifndef MY_TRANSPORT_MAX_TSM_FAILURES
#define MY_TRANSPORT_MAX_TSM_FAILURES		(7u)		//!< Max. number of consecutive TSM failure state entries (3bits)
#endif

#ifndef MY_TRANSPORT_TIMEOUT_FAILURE_STATE_MS
#define MY_TRANSPORT_TIMEOUT_FAILURE_STATE_MS		(10*1000ul)		//!< Duration failure state (in ms)
#endif
#ifndef MY_TRANSPORT_TIMEOUT_EXT_FAILURE_STATE_MS
#define MY_TRANSPORT_TIMEOUT_EXT_FAILURE_STATE_MS	(60*1000ul)		//!< Duration extended failure state (in ms)
#endif
#ifndef MY_TRANSPORT_STATE_TIMEOUT_MS
#define MY_TRANSPORT_STATE_TIMEOUT_MS			(2*1000ul)		//!< general state timeout (in ms)
#endif
#ifndef MY_TRANSPORT_CHKUPL_INTERVAL_MS
#define MY_TRANSPORT_CHKUPL_INTERVAL_MS			(10*1000ul)		//!< Interval to re-check uplink (in ms)
#endif
#ifndef MY_TRANSPORT_STATE_RETRIES
#define MY_TRANSPORT_STATE_RETRIES				(3u)			//!< retries before switching to FAILURE
#endif

#define AUTO									(255u)			//!< ID 255 is reserved
#define BROADCAST_ADDRESS			(255u)			//!< broadcasts are addressed to ID 255
#define DISTANCE_INVALID			(255u)			//!< invalid distance when searching for parent
#define MAX_HOPS							(254u)			//!< maximal number of hops for ping/pong
#define INVALID_HOPS					(255u)			//!< invalid hops
#define MAX_SUBSEQ_MSGS				(5u)				//!< Maximum number of subsequently processed messages in FIFO (to prevent transport deadlock if HW issue)
#define UPLINK_QUALITY_WEIGHT	(0.05f)			//!< UPLINK_QUALITY_WEIGHT


// parent node check
#if defined(MY_PARENT_NODE_IS_STATIC) && !defined(MY_PARENT_NODE_ID)
#error MY_PARENT_NODE_IS_STATIC but no MY_PARENT_NODE_ID defined!
#endif

#define _autoFindParent (bool)(MY_PARENT_NODE_ID == AUTO)				//!<  returns true if static parent id is undefined
#define isValidDistance(_distance) (bool)(_distance!=DISTANCE_INVALID)	//!<  returns true if distance is valid
#define isValidParent(_parent) (bool)(_parent != AUTO)					//!<  returns true if parent is valid

/**
 * @brief Callback type
 */
typedef void(*transportCallback_t)(void);

/**
 * @brief Node configuration
 *
 * This structure stores node-related configurations
 */
typedef struct {
	uint8_t nodeId;								//!< Current node id
	uint8_t parentNodeId;					//!< Where this node sends its messages
	uint8_t distanceGW;						//!< This nodes distance to sensor net gateway (number of hops)
	uint8_t passiveMode : 1;			//!< Passive mode
	uint8_t reserved : 7;					//!< Reserved
} transportConfig_t;

/**
 * @brief SM state
 *
 * This structure stores SM state definitions
 */
typedef struct {
	void(*Transition)(void);					//!< state transition function
	void(*Update)(void);							//!< state update function
} transportState_t;
/**
* @brief Datatype for internal RSSI storage
*/
typedef int16_t transportRSSI_t;				//!< Datatype for internal RSSI storage

// helper macro for conversion
#define transportInternalToRSSI(__value)	((int16_t)__value >> 4)				//!< Convert internal RSSI to RSSI
#define transportRSSItoInternal(__value)	((transportRSSI_t)__value << 4)		//!< Convert RSSI to internal RSSI

/**
 * @brief Status variables and SM state
 *
 * This structure stores transport status and SM variables
 */
typedef struct {
	// SM variables
	transportState_t *currentState;					//!< pointer to current FSM state
	uint32_t stateEnter;										//!< state enter timepoint
	// general transport variables
	uint32_t lastUplinkCheck;								//!< last uplink check, required to prevent GW flooding
	// 8 bits
	bool findingParentNode : 1;							//!< flag finding parent node is active
	bool preferredParentFound : 1;					//!< flag preferred parent found
	bool uplinkOk : 1;											//!< flag uplink ok
	bool pingActive : 1;										//!< flag ping active
	bool transportActive : 1;								//!< flag transport active
	uint8_t stateRetries : 3;								//!< retries / state re-enter (max 7)
	// 8 bits
	uint8_t failedUplinkTransmissions : 4;	//!< counter failed uplink transmissions (max 15)
	uint8_t failureCounter : 3;							//!< counter for TSM failures (max 7)
	bool msgReceived : 1;										//!< flag message received
	uint8_t pingResponse;										//!< stores I_PONG hops
#if defined(MY_SIGNAL_REPORT_ENABLED)
	transportRSSI_t uplinkQualityRSSI;			//!< Uplink quality, internal RSSI representation
#endif
} transportSM_t;

/**
* @brief RAM routing table
*/
typedef struct {
	uint8_t route[SIZE_ROUTES];	//!< route for node
} routingTable_t;

// PRIVATE functions

/**
* @brief Initialize SM variables and transport HW
*/
void stInitTransition(void);
/**
* @brief Initialize transport
*/
void stInitUpdate(void);
/**
* @brief Find parent
*/
void stParentTransition(void);
/**
* @brief Verify find parent responses
*/
void stParentUpdate(void);
/**
* @brief Send ID request
*/
void stIDTransition(void);
/**
* @brief Verify ID response and GW link
*/
void stIDUpdate(void);
/**
* @brief Send uplink ping request
*/
void stUplinkTransition(void);
/**
* @brief Verify uplink response
*/
void stUplinkUpdate(void);
/**
* @brief Set transport OK
*/
void stReadyTransition(void);
/**
* @brief Monitor transport link
*/
void stReadyUpdate(void);
/**
* @brief Transport failure and power down radio
*/
void stFailureTransition(void);
/**
* @brief Re-initialize transport after timeout
*/
void stFailureUpdate(void);
/**
* @brief Switch SM state
* @param newState New state to switch SM to
*/
void transportSwitchSM(transportState_t &newState);
/**
* @brief Update SM state
*/
void transportUpdateSM(void);
/**
* @brief Request time in current SM state
* @return ms in current state
*/
uint32_t transportTimeInState(void);
/**
* @brief Call transport driver sanity check
*/
void transportInvokeSanityCheck(void);
/**
* @brief Process all pending messages in RX FIFO
*/
void transportProcessFIFO(void);
/**
* @brief Receive message from RX FIFO and process
*/
void transportProcessMessage(void);
/**
* @brief Assign node ID
* @param newNodeId New node ID
* @return true if node ID is valid and successfully assigned
*/
bool transportAssignNodeID(const uint8_t newNodeId);
/**
* @brief Wait and process messages for a defined amount of time until specified message received
* @param waitingMS Time to wait and process incoming messages in ms
* @param cmd Specific command
* @param msgType Specific message type
* @return true if specified command received within waiting time
*/
bool transportWait(const uint32_t waitingMS, const uint8_t cmd, const uint8_t msgType);
/**
* @brief Ping node
* @param targetId Node to be pinged
* @return hops from pinged node or 255 if no answer received within 2000ms
*/
uint8_t transportPingNode(const uint8_t targetId);
/**
* @brief Send and route message according to destination
*
* This function is used in MyTransport and omits the transport state check, i.e. message can be sent even if transport is not ready
*
* @param message
* @return true if message sent successfully
*/
bool transportRouteMessage(MyMessage &message);
/**
* @brief Send and route message according to destination with transport state check
* @param message
* @return true if message sent successfully and false if sending error or transport !OK
*/
bool transportSendRoute(MyMessage &message);
/**
* @brief Send message to recipient
* @param to Recipient of message
* @param message
* @return true if message sent successfully
*/
bool transportSendWrite(const uint8_t to, MyMessage &message);
/**
* @brief Check uplink to GW, includes flooding control
* @param force to override flood control timer
* @return true if uplink ok
*/
bool transportCheckUplink(const bool force = false);

// PUBLIC functions

/**
* @brief Wait until transport is ready
* @param waitingMS timeout in MS, set 0 (default) for no timeout, i.e. wait indefinitely. For a node in standalone mode (optional network connection) set >0 to allow a node entering the main loop() function.
* @return true if transport is ready
*/
bool transportWaitUntilReady(const uint32_t waitingMS = 0);
/**
* @brief Initialize transport and SM
*/
void transportInitialise(void);
/**
* @brief Process FIFO msg and update SM
*/
void transportProcess(void);
/**
* @brief Flag transport ready
* @return true if transport is initialized and ready
*/
bool isTransportReady(void);
/**
* @brief Flag searching parent ongoing
* @return true if transport is searching for parent
*/
bool isTransportSearchingParent(void);
/**
* @brief Flag TSM extended failure
* @return true if TSM had too many consecutive failure state entries
*/
bool isTransportExtendedFailure(void);
/**
* @brief Flag valid message received
* @return true if valid message received, needs to be reset if used
*/
bool isMessageReceived(void);
/**
* @brief Reset message received flag
*/
void resetMessageReceived(void);
/**
* @brief Clear routing table
*/
void transportClearRoutingTable(void);
/**
* @brief Return heart beat
* @return MS in current state
*/
uint32_t transportGetHeartbeat(void);
/**
* @brief Load routing table from EEPROM to RAM.
* Only for GW devices with enough RAM, i.e. ESP8266, RPI Sensebender GW, etc.
* Atmega328 has only limited amount of RAM
*/
void transportLoadRoutingTable(void);
/**
* @brief Save routing table to EEPROM.
*/
void transportSaveRoutingTable(void);
/**
* @brief Update routing table
* @param node
* @param route
*/
void transportSetRoute(const uint8_t node, const uint8_t route);
/**
* @brief Load route to node
* @param node
* @return route to node
*/
uint8_t transportGetRoute(const uint8_t node);
/**
* @brief Reports content of routing table
*/
void transportReportRoutingTable(void);
/**
* @brief Get node ID
* @return node ID
*/
uint8_t transportGetNodeId(void);
/**
* @brief Get parent node ID
* @return parent node ID
*/
uint8_t transportGetParentNodeId(void);
/**
* @brief Get distance to GW
* @return distance (=hops) to GW
*/
uint8_t transportGetDistanceGW(void);
/**
* @brief Toggle passive mode, i.e. transport does not wait for ACK
* @param OnOff
*/
void transportTogglePassiveMode(const bool OnOff);
/**
* @brief Disable transport, if xxx_POWER_PIN is defined, transport is powered down, else send to sleep
*/
void transportDisable(void);
/**
* @brief Reinitialise transport. Put transport to standby - If xxx_POWER_PIN set, power up and go to standby
*/
void transportReInitialise(void);

/**
* @brief Get transport signal report
* @param command:
* R = RSSI (if available) of incoming @ref I_SIGNAL_REPORT_REQUEST message (from last hop)
* R! = RSSI (if available) of ACK to @ref I_SIGNAL_REPORT_REVERSE message received from last hop
* S = SNR (if available) of incoming @ref I_SIGNAL_REPORT_REQUEST message (from last hop)
* S! = SNR (if available) of ACK to @ref I_SIGNAL_REPORT_REVERSE message received from last hop
* P = TX powerlevel in %
* T = TX powerlevel in dBm
* U = Uplink quality (via ACK from parent node), avg. RSSI
* @return Signal report (if report is not available, INVALID_RSSI, INVALID_SNR, INVALID_PERCENT, or INVALID_LEVEL is sent instead)
*/
int16_t transportSignalReport(const char command) __attribute__((unused));

/**
* @brief Get transport signal report
* @param signalReport
* @return report
*/
int16_t transportGetSignalReport(const signalReport_t signalReport) __attribute__((unused));

#endif // MyTransport_h
/** @}*/
