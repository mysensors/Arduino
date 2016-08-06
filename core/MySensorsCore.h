/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

 /**
 * @file MySensorsCore.h
 *
 * @defgroup MySensorsCoregrp MySensorsCore
 * @ingroup internals
 * @{
 *
 * MySensorsCore-related log messages, format: [!]SYSTEM:[SUB SYSTEM:]MESSAGE
 * - [!] Exclamation mark is prepended in case of error
 * - SYSTEM:
 *  - <b>MCO</b> messages emitted by MySensorsCore
 * - SUB SYSTEMS:
 *  - MCO:<b>BGN</b>	from @ref _begin()
 *  - MCO:<b>REG</b>	from @ref _registerNode()
 *  - MCO:<b>SND</b>	from @ref send()
 *  - MCO:<b>PIM</b>	from @ref _processInternalMessages()
 *  - MCO:<b>NLK</b>	from @ref nodeLock()
 *
 * MySensorsCore debug log messages:
 *
 * |E| SYS	| SUB	| Message									| Comment
 * |-|------|-------|-------------------------------------------|----------------------------------------------------------------------------
 * | | MCO  | BGN	| INIT %%s,CP=%%s,LIB=%%s					| Core initialization, capabilities (CP), library version (VER)
 * | | MCO	| BGN	| INIT OK,ID=%%d,PAR=%%d,DIS=%%d,REG=%%d	| Core initialized, parent ID (PAR), distance to GW (DIS), registration (REG)
 * | | MCO	| BGN	| NODE UNLOCKED								| Node successfully unlocked (see signing chapter)
 * |!| MCO	| BGN	| TSP FAIL									| Transport initialization failed
 * | | MCO	| REG	| REQ										| Registration request
 * | | MCO	| REG	| NOT NEEDED								| No registration needed (i.e. GW)
 * |!| MCO	| SND	| NODE NOT REG								| Node is not registered, cannot send message
 * | | MCO	| PIM	| NODE REG=%%d								| Registration response received, registration status (REG)
 * | | MCO	| PIM	| ROUTE N=%%d,R=%%d							| Routing table, messages to node (N) are routed via node (R)
 * | | MCO	| NLK	| NODE LOCKED. UNLOCK: GND PIN %%d AND RESET| Node locked during booting, see signing chapter for additional information
 *
 *
 * @brief API declaration for MySensorsCore
 */


#ifndef MySensorsCore_h
#define MySensorsCore_h

#include "Version.h"   // Auto generated by bot
#include "MyConfig.h"
#include "MyEepromAddresses.h"
#include "MyMessage.h"
#include <stddef.h>
#include <stdarg.h>

#define GATEWAY_ADDRESS ((uint8_t)0)			//!< Node ID for GW sketch
#define NODE_SENSOR_ID 0xFF						//!< Node child is always created/presented when a node is started
#define MY_CORE_VERSION ((uint8_t)2)			//!< core version
#define MY_CORE_MIN_VERSION ((uint8_t)2)		//!< min core version required for compatibility

#ifdef MY_DEBUG
	#define debug(x,...) hwDebugPrint(x, ##__VA_ARGS__)	//!< debug
#else
	#define debug(x,...)								//!< debug NULL
#endif

/**
 * @brief Node configuration
 *
 * This structure stores node-related configurations
 */
struct NodeConfig {
	uint8_t nodeId;								//!< Current node id
	uint8_t parentNodeId;						//!< Where this node sends its messages
	uint8_t distance;							//!< This nodes distance to sensor net gateway (number of hops)
};

/**
 * @brief Controller configuration
 *
 * This structure stores controllerrelated configurations
 */
struct ControllerConfig {
	uint8_t isMetric;							//!< Flag indicating if metric or imperial measurements are used
};

extern NodeConfig _nc;							//!< Node config
extern MyMessage _msg;							//!< Buffer for incoming messages
extern MyMessage _msgTmp;						//!< Buffer for temporary messages (acks and nonces among others)
#ifdef MY_DEBUG
	extern char _convBuf[MAX_PAYLOAD * 2 + 1];
#endif

// **** public functions ********

/**
 * Return this nodes id.
 */
uint8_t getNodeId();

/**
 * Return the parent node id.
 */
uint8_t getParentNodeId();

/**
* Sends node information to the gateway.
*/
void presentNode();

/**
* Each node must present all attached sensors before any values can be handled correctly by the controller.
* It is usually good to present all attached sensors after power-up in setup().
*
* @param sensorId Select a unique sensor id for this sensor. Choose a number between 0-254.
* @param sensorType The sensor type. See sensor typedef in MyMessage.h.
* @param description A textual description of the sensor.
* @param ack Set this to true if you want destination node to send ack back to this node. Default is not to request any ack.
* @param description A textual description of the sensor.
*/
void present(uint8_t sensorId, uint8_t sensorType, const char *description="", bool ack=false);

/**
 * Sends sketch meta information to the gateway. Not mandatory but a nice thing to do.
 * @param name String containing a short Sketch name or NULL  if not applicable
 * @param version String containing a short Sketch version or NULL if not applicable
 * @param ack Set this to true if you want destination node to send ack back to this node. Default is not to request any ack.
 *
 */
void sendSketchInfo(const char *name, const char *version, bool ack=false);

/**
* Sends a message to gateway or one of the other nodes in the radio network
*
* @param msg Message to send
* @param ack Set this to true if you want destination node to send ack back to this node. Default is not to request any ack.
* @return true Returns true if message reached the first stop on its way to destination.
*/
bool send(MyMessage &msg, bool ack=false);


/**
 * Send this nodes battery level to gateway.
 * @param level Level between 0-100(%)
 * @param ack Set this to true if you want destination node to send ack back to this node. Default is not to request any ack.
 *
 */
void sendBatteryLevel(uint8_t level, bool ack=false);

/**
 * Send a heartbeat message (I'm alive!) to the gateway/controller.
 * The payload will be an incremental 16 bit integer value starting at 1 when sensor is powered on.
 */
void sendHeartbeat(void);

/**
* Requests a value from gateway or some other sensor in the radio network.
* Make sure to add callback-method in begin-method to handle request responses.
*
* @param childSensorId  The unique child id for the different sensors connected to this Arduino. 0-254.
* @param variableType The variableType to fetch
* @param destination The nodeId of other node in radio network. Default is gateway
*/
void request(uint8_t childSensorId, uint8_t variableType, uint8_t destination=GATEWAY_ADDRESS);

/**
 * Requests time from controller. Answer will be delivered to receiveTime function in sketch.
 *
 */
void requestTime();

/**
 * Returns the most recent node configuration received from controller
 */
ControllerConfig getConfig();

/**
 * Save a state (in local EEPROM). Good for actuators to "remember" state between
 * power cycles.
 *
 * You have 256 bytes to play with. Note that there is a limitation on the number
 * of writes the EEPROM can handle (~100 000 cycles on ATMega328).
 *
 * @param pos The position to store value in (0-255)
 * @param value to store in position
 */
void saveState(uint8_t pos, uint8_t value);

/**
 * Load a state (from local EEPROM).
 *
 * @param pos The position to fetch value from  (0-255)
 * @return Value to store in position
 */
uint8_t loadState(uint8_t pos);

/**
 * Wait for a specified amount of time to pass.  Keeps process()ing.
 * This does not power-down the radio nor the Arduino.
 * Because this calls process() in a loop, it is a good way to wait
 * in your loop() on a repeater node or sensor that listens to messages.
 * @param ms Number of milliseconds to sleep.
 */
void wait(unsigned long ms);

/**
 * Wait for a specified amount of time to pass or until specified message received.  Keeps process()ing.
 * This does not power-down the radio nor the Arduino.
 * Because this calls process() in a loop, it is a good way to wait
 * in your loop() on a repeater node or sensor that listens to messages.
 * @param ms Number of milliseconds to sleep.
 * @param cmd Command of incoming message.
 * @param msgtype Message type.
 * @return True if specified message received
 */
bool wait(unsigned long ms, uint8_t cmd, uint8_t msgtype);

/**
 * Sleep (PowerDownMode) the MCU and radio. Wake up on timer.
 * @param ms Number of milliseconds to sleep.
 * @return -1 if timer woke it up, -2 if not possible (e.g. ongoing FW update)
 */
int8_t sleep(unsigned long ms);
/**
* Same as sleep(), send heartbeat and process incoming messages before going to sleep.
* Specify the time to wait for incoming messages by defining MY_SMART_SLEEP_WAIT_DURATION to a time (ms).
* @param ms Number of milliseconds to sleep.
* @return -1 if timer woke it up, -2 if not possible (e.g. ongoing FW update)
*/
int8_t smartSleep(unsigned long ms);

/**
 * Sleep (PowerDownMode) the MCU and radio. Wake up on timer or pin change.
 * See: http://arduino.cc/en/Reference/attachInterrupt for details on modes and which pin
 * is assigned to what interrupt. On Nano/Pro Mini: 0=Pin2, 1=Pin3
 * @param interrupt Interrupt that should trigger the wakeup
 * @param mode RISING, FALLING, CHANGE
 * @param ms Number of milliseconds to sleep or 0 to sleep forever
 * @return Interrupt number if wake up was triggered by pin change, -1 if wake up was triggered by timer, -2 if sleep was not possible (e.g. ongoing FW update)
 */
int8_t sleep(uint8_t interrupt, uint8_t mode, unsigned long ms=0);
/**
* Same as sleep(), send heartbeat and process incoming messages before going to sleep.
* Specify the time to wait for incoming messages by defining MY_SMART_SLEEP_WAIT_DURATION to a time (ms).
* @param interrupt Interrupt that should trigger the wakeup
* @param mode RISING, FALLING, CHANGE
* @param ms Number of milliseconds to sleep or 0 to sleep forever
* @return Interrupt number if wake up was triggered by pin change, -1 if wake up was triggered by timer, -2 if sleep was not possible (e.g. ongoing FW update)
*/
int8_t smartSleep(uint8_t interrupt, uint8_t mode, unsigned long ms=0);

/**
 * Sleep (PowerDownMode) the MCU and radio. Wake up on timer or pin change for two separate interrupts.
 * See: http://arduino.cc/en/Reference/attachInterrupt for details on modes and which pin
 * is assigned to what interrupt. On Nano/Pro Mini: 0=Pin2, 1=Pin3
 * @param interrupt1 First interrupt that should trigger the wakeup
 * @param mode1 Mode for first interrupt (RISING, FALLING, CHANGE)
 * @param interrupt2 Second interrupt that should trigger the wakeup
 * @param mode2 Mode for second interrupt (RISING, FALLING, CHANGE)
 * @param ms Number of milliseconds to sleep or 0 to sleep forever
 * @return Interrupt number if wake up was triggered by pin change, -1 if wake up was triggered by timer, -2 if sleep was not possible (e.g. ongoing FW update)
 */
int8_t sleep(uint8_t interrupt1, uint8_t mode1, uint8_t interrupt2, uint8_t mode2, unsigned long ms=0);
/**
* Same as sleep(), send heartbeat and process incoming messages before going to sleep.
* Specify the time to wait for incoming messages by defining MY_SMART_SLEEP_WAIT_DURATION to a time (ms).
* @param interrupt1 First interrupt that should trigger the wakeup
* @param mode1 Mode for first interrupt (RISING, FALLING, CHANGE)
* @param interrupt2 Second interrupt that should trigger the wakeup
* @param mode2 Mode for second interrupt (RISING, FALLING, CHANGE)
* @param ms Number of milliseconds to sleep or 0 to sleep forever
* @return Interrupt number if wake up was triggered by pin change, -1 if wake up was triggered by timer, -2 if sleep was not possible (e.g. ongoing FW update)
*/
int8_t smartSleep(uint8_t interrupt1, uint8_t mode1, uint8_t interrupt2, uint8_t mode2, unsigned long ms=0);

#ifdef MY_NODE_LOCK_FEATURE
/**
 * @ingroup MyLockgrp
 * @ingroup internals
 * @brief Lock a node and transmit provided message with 30m intervals
 *
 * This function is called if suspicious activity has exceeded the threshold (see
 * @ref ATTACK_COUNTER_MAX). Unlocking with a normal Arduino bootloader require erasing the EEPROM
 * while unlocking with a custom bootloader require holding @ref UNLOCK_PIN low during power on/reset.
 *
 * @param str The string to transmit.
 */
void nodeLock(const char* str);
#endif

// **** private functions ********

/**
* @brief Node initialisation
*/
void _begin();
/**
* @brief Main framework process
*/
void _process(void);
/**
* @brief Processes internal messages
* @return True if received message requires further processing
*/
bool _processInternalMessages();
/**
* @brief Puts node to a infinite loop if unrecoverable situation detected
*/
void _infiniteLoop();
/**
* @brief Handles registration request
*/
void _registerNode();
/**
* @brief Sends message according to routing table
* @param message
* @return True if message successfully sent to next node
*/
bool _sendRoute(MyMessage &message);

/**
* @brief Callback for incoming messages
* @param message
*/
void receive(const MyMessage &message)  __attribute__((weak));
/**
* @brief Callback for incoming time messages
*/
void receiveTime(unsigned long)  __attribute__((weak));
/**
* @brief Node presenation
*/
void presentation()  __attribute__((weak));
/**
* @brief Called before node initialises
*/
void before() __attribute__((weak));
/**
* @brief Called before any hwInitialization is done
*/
void preHwInit() __attribute__((weak));
/**
* @brief Called after node initialises but before main loop
*/
void setup() __attribute__((weak));
/**
* @brief Main loop
*/
void loop() __attribute__((weak));


// Inline function and macros
static inline MyMessage& build(MyMessage &msg, uint8_t sender, uint8_t destination, uint8_t sensor, uint8_t command, uint8_t type, bool enableAck) {
	msg.sender = sender;
	msg.destination = destination;
	msg.sensor = sensor;
	msg.type = type;
	mSetCommand(msg,command);
	mSetRequestAck(msg,enableAck);
	mSetAck(msg,false);
	return msg;
}

static inline MyMessage& buildGw(MyMessage &msg, uint8_t type) {
	msg.sender = GATEWAY_ADDRESS;
	msg.destination = GATEWAY_ADDRESS;
	msg.sensor = NODE_SENSOR_ID;
	msg.type = type;
	mSetCommand(msg, C_INTERNAL);
	mSetRequestAck(msg, false);
	mSetAck(msg, false);
	return msg;
}


#endif

/** @}*/
