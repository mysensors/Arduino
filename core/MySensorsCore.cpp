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

#include "MySensorsCore.h"

// debug output
#if defined(MY_DEBUG_VERBOSE_CORE)
#define CORE_DEBUG(x,...)	DEBUG_OUTPUT(x, ##__VA_ARGS__)	//!< debug
#else
#define CORE_DEBUG(x,...)	//!< debug NULL
#endif

// message buffers
MyMessage _msg;			// Buffer for incoming messages
MyMessage _msgTmp;		// Buffer for temporary messages (acks and nonces among others)

// core configuration
static coreConfig_t _coreConfig;

#if defined(MY_DEBUG_VERBOSE_CORE)
static uint8_t waitLock = 0;
static uint8_t processLock = 0;
#endif

#if defined(DEBUG_OUTPUT_ENABLED)
char _convBuf[MAX_PAYLOAD_SIZE * 2 + 1];
#endif

// Callback for transport=ok transition
void _callbackTransportReady(void)
{
	if (!_coreConfig.presentationSent) {
#if !defined(MY_GATEWAY_FEATURE)	// GW calls presentNode() when client connected
		presentNode();
#endif
		_registerNode();
		_coreConfig.presentationSent = true;
	}
}

void _process(void)
{
#if defined(MY_DEBUG_VERBOSE_CORE)
	if (processLock) {
		CORE_DEBUG(PSTR("!MCO:PRO:RC=%" PRIu8 "\n"), processLock);	// recursive call detected
	}
	processLock++;
#endif
	doYield();

#if defined(MY_INCLUSION_MODE_FEATURE)
	inclusionProcess();
#endif

#if defined(MY_GATEWAY_FEATURE)
	gatewayTransportProcess();
#endif

#if defined(MY_SENSOR_NETWORK)
	transportProcess();
#endif

#if defined(__linux__)
	// To avoid high cpu usage
	usleep(10000); // 10ms
#endif
#if defined(MY_DEBUG_VERBOSE_CORE)
	processLock--;
#endif
}

void _infiniteLoop(void)
{
#if defined(__linux__)
	exit(1);
#else
	while(1) {
		doYield();
	}
#endif
}

void _begin(void)
{
#if defined(MY_DIAGNOSTICS)
	(void)hwInit();
	diagnosticsRun();
	_infiniteLoop();
#endif
#if defined(MY_CORE_ONLY)
	// initialize HW and run setup if present
	(void)hwInit();
	if (setup) {
		setup();
	}
	return;
#endif
	// reset wdt
	hwWatchdogReset();

	if (preHwInit) {
		preHwInit();
	}

	const bool hwInitResult = hwInit();

#if !defined(MY_SPLASH_SCREEN_DISABLED) && !defined(MY_GATEWAY_FEATURE)
	displaySplashScreen();
#endif

#if defined(F_CPU)
	CORE_DEBUG(PSTR("MCO:BGN:INIT " MY_NODE_TYPE ",CP=" MY_CAPABILITIES ",FQ=%" PRIu16 ",REL=%"
	                PRIu8 ",VER="
	                MYSENSORS_LIBRARY_VERSION "\n"), (uint16_t)(F_CPU/1000000UL),
	           MYSENSORS_LIBRARY_VERSION_PRERELEASE_NUMBER);
#else
	CORE_DEBUG(PSTR("MCO:BGN:INIT " MY_NODE_TYPE ",CP=" MY_CAPABILITIES ",FQ=NA,REL=%"
	                PRIu8 ",VER="
	                MYSENSORS_LIBRARY_VERSION "\n"), MYSENSORS_LIBRARY_VERSION_PRERELEASE_NUMBER);
#endif
	if (!hwInitResult) {
		CORE_DEBUG(PSTR("!MCO:BGN:HW ERR\n"));
		setIndication(INDICATION_ERR_HW_INIT);
		_infiniteLoop();
	}

	// set defaults
	_coreConfig.presentationSent = false;

	// Call sketch before() (if defined)
	if (before) {
		CORE_DEBUG(PSTR("MCO:BGN:BFR\n"));	// before callback
		before();
	}

#if defined(MY_DEFAULT_TX_LED_PIN) || defined(MY_DEFAULT_RX_LED_PIN) || defined(MY_DEFAULT_ERR_LED_PIN)
	ledsInit();
#endif

	signerInit();

	// Read latest received controller configuration from EEPROM
	// Note: _coreConfig.isMetric is bool, hence empty EEPROM (=0xFF) evaluates to true (default)
	hwReadConfigBlock((void *)&_coreConfig.controllerConfig, (void *)EEPROM_CONTROLLER_CONFIG_ADDRESS,
	                  sizeof(controllerConfig_t));

#if defined(MY_OTA_FIRMWARE_FEATURE)
	// Read firmware config from EEPROM, i.e. type, version, CRC, blocks
	readFirmwareSettings();
#endif

#if defined(MY_SENSOR_NETWORK)
	// Save static parent ID in eeprom (used by bootloader)
	hwWriteConfig(EEPROM_PARENT_NODE_ID_ADDRESS, MY_PARENT_NODE_ID);
	// Initialise transport layer
	transportInitialise();
	// Register transport=ready callback
	transportRegisterReadyCallback(_callbackTransportReady);
	// wait until transport is ready
	(void)transportWaitUntilReady(MY_TRANSPORT_WAIT_READY_MS);
#endif

	_checkNodeLock();

#if defined(MY_GATEWAY_FEATURE)
#if defined(MY_INCLUSION_BUTTON_FEATURE)
	inclusionInit();
#endif

	// initialise the transport driver
	if (!gatewayTransportInit()) {
		setIndication(INDICATION_ERR_INIT_GWTRANSPORT);
		CORE_DEBUG(PSTR("!MCO:BGN:TSP FAIL\n"));
		// Nothing more we can do
		_infiniteLoop();
	}
#endif

	// Call sketch setup() (if defined)
	if (setup) {
		CORE_DEBUG(PSTR("MCO:BGN:STP\n"));	// setup callback
		setup();
	}
#if defined(MY_SENSOR_NETWORK)
	CORE_DEBUG(PSTR("MCO:BGN:INIT OK,TSP=%" PRIu8 "\n"), isTransportReady() &&
	           transportHALSanityCheck());
#else
	// no sensor network defined, call presentation & registration
	_callbackTransportReady();
	CORE_DEBUG(PSTR("MCO:BGN:INIT OK,TSP=NA\n"));
#endif
	// reset wdt before handing over to loop
	hwWatchdogReset();
}


void _registerNode(void)
{
#if defined (MY_REGISTRATION_FEATURE) && !defined(MY_GATEWAY_FEATURE)
	CORE_DEBUG(PSTR("MCO:REG:REQ\n"));	// registration request
	setIndication(INDICATION_REQ_REGISTRATION);
	_coreConfig.nodeRegistered = MY_REGISTRATION_DEFAULT;
	uint8_t counter = MY_REGISTRATION_RETRIES;
	// only proceed if register response received or retries exceeded
	do {
		(void)_sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
		                       I_REGISTRATION_REQUEST).set(MY_CORE_VERSION));
	} while (!wait(2000, C_INTERNAL, I_REGISTRATION_RESPONSE) && counter--);
#else
	_coreConfig.nodeRegistered = true;
	CORE_DEBUG(PSTR("MCO:REG:NOT NEEDED\n"));
#endif
}

void presentNode(void)
{
	setIndication(INDICATION_PRESENT);
	// Present node and request config
#if defined(MY_GATEWAY_FEATURE)
	// Send presentation for this gateway device
#if defined(MY_REPEATER_FEATURE)
	(void)present(NODE_SENSOR_ID, S_ARDUINO_REPEATER_NODE);
#else
	(void)present(NODE_SENSOR_ID, S_ARDUINO_NODE);
#endif
#else

#if defined(MY_OTA_FIRMWARE_FEATURE)
	presentBootloaderInformation();
#endif

	// Send signing preferences for this node to the GW
	signerPresentation(_msgTmp, GATEWAY_ADDRESS);

	// Send presentation for this radio node
#if defined(MY_REPEATER_FEATURE)
	(void)present(NODE_SENSOR_ID, S_ARDUINO_REPEATER_NODE);
#else
	(void)present(NODE_SENSOR_ID, S_ARDUINO_NODE);
#endif

	// Send a configuration exchange request to controller
	// Node sends parent node. Controller answers with latest node configuration
	(void)_sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
	                       I_CONFIG).set(getParentNodeId()));

	// Wait configuration reply.
	(void)wait(2000, C_INTERNAL, I_CONFIG);

#endif

	if (presentation) {
		presentation();
	}
}


uint8_t getNodeId(void)
{
	uint8_t result;
#if defined(MY_GATEWAY_FEATURE)
	result = GATEWAY_ADDRESS;
#elif defined(MY_SENSOR_NETWORK)
	result = transportGetNodeId();
#else
	result = VALUE_NOT_DEFINED;
#endif
	return result;
}

uint8_t getParentNodeId(void)
{
	uint8_t result;
#if defined(MY_GATEWAY_FEATURE)
	result = VALUE_NOT_DEFINED;	// GW doesn't have a parent
#elif defined(MY_SENSOR_NETWORK)
	result = transportGetParentNodeId();
#else
	result = VALUE_NOT_DEFINED;
#endif
	return result;
}

uint8_t getDistanceGW(void)
{
	uint8_t result;
#if defined(MY_GATEWAY_FEATURE)
	result = 0;
#elif defined(MY_SENSOR_NETWORK)
	result = transportGetDistanceGW();
#else
	result = VALUE_NOT_DEFINED;
#endif
	return result;
}

controllerConfig_t getControllerConfig(void)
{
	return _coreConfig.controllerConfig;
}

// cppcheck-suppress constParameter
bool _sendRoute(MyMessage &message)
{
#if defined(MY_CORE_ONLY)
	(void)message;
#endif
#if defined(MY_GATEWAY_FEATURE)
	if (message.getDestination() == getNodeId()) {
		// This is a message sent from a sensor attached on the gateway node.
		// Pass it directly to the gateway transport layer.
		return gatewayTransportSend(message);
	}
#endif
#if defined(MY_SENSOR_NETWORK)
	return transportSendRoute(message);
#else
	return false;
#endif
}

bool send(MyMessage &message, const bool requestEcho)
{
	message.setSender(getNodeId());
	message.setCommand(C_SET);
	message.setRequestEcho(requestEcho);

#if defined(MY_REGISTRATION_FEATURE) && !defined(MY_GATEWAY_FEATURE)
	if (_coreConfig.nodeRegistered) {
		return _sendRoute(message);
	} else {
		CORE_DEBUG(PSTR("!MCO:SND:NODE NOT REG\n"));	// node not registered
		return false;
	}
#else
	return _sendRoute(message);
#endif
}

bool sendBatteryLevel(const uint8_t value, const bool requestEcho)
{
	return _sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_BATTERY_LEVEL,
	                        requestEcho).set(value));
}

bool sendHeartbeat(const bool requestEcho)
{
#if defined(MY_SENSOR_NETWORK)
	const uint32_t heartbeat = transportGetHeartbeat();
	return _sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_HEARTBEAT_RESPONSE,
	                        requestEcho).set(heartbeat));
#elif defined(MY_GATEWAY_FEATURE)
	const uint32_t heartbeat = hwMillis();
	return _sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_HEARTBEAT_RESPONSE,
	                        requestEcho).set(heartbeat));
#else
	(void)requestEcho;
	return false;
#endif
}



bool present(const uint8_t childSensorId, const mysensors_sensor_t sensorType,
             const char *description,
             const bool requestEcho)
{
	return _sendRoute(build(_msgTmp, GATEWAY_ADDRESS, childSensorId, C_PRESENTATION,
	                        static_cast<uint8_t>(sensorType),
	                        requestEcho).set(childSensorId == NODE_SENSOR_ID ? MYSENSORS_LIBRARY_VERSION : description));
}

#if !defined(__linux__)
bool present(const uint8_t childSensorId, const mysensors_sensor_t sensorType,
             const __FlashStringHelper *description,
             const bool requestEcho)
{
	return _sendRoute(build(_msgTmp, GATEWAY_ADDRESS, childSensorId, C_PRESENTATION,
	                        static_cast<uint8_t>(sensorType),
	                        requestEcho).set(childSensorId == NODE_SENSOR_ID ? F(" MYSENSORS_LIBRARY_VERSION "): description));
}
#endif


bool sendSketchInfo(const char *name, const char *version, const bool requestEcho)
{
	bool result = true;
	if (name) {
		result &= _sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_SKETCH_NAME,
		                           requestEcho).set(name));
	}
	if (version) {
		result &= _sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_SKETCH_VERSION,
		                           requestEcho).set(version));
	}
	return result;
}

#if !defined(__linux__)
bool sendSketchInfo(const __FlashStringHelper *name, const __FlashStringHelper *version,
                    const bool requestEcho)
{
	bool result = true;
	if (name) {
		result &= _sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_SKETCH_NAME,
		                           requestEcho).set(name));
	}
	if (version) {
		result &= _sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_SKETCH_VERSION,
		                           requestEcho).set(version));
	}
	return result;
}
#endif

bool request(const uint8_t childSensorId, const uint8_t variableType, const uint8_t destination)
{
	return _sendRoute(build(_msgTmp, destination, childSensorId, C_REQ, variableType).set(""));
}

bool requestTime(const bool requestEcho)
{
	return _sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_TIME,
	                        requestEcho).set(""));
}

// Message delivered through _msg
bool _processInternalCoreMessage(void)
{
	const uint8_t type = _msg.getType();
	if (_msg.getSender() == GATEWAY_ADDRESS) {
		if (type == I_REBOOT) {
#if !defined(MY_DISABLE_REMOTE_RESET)
			setIndication(INDICATION_REBOOT);
			// WDT fuse should be enabled
			hwReboot();
#endif
		} else if (type == I_REGISTRATION_RESPONSE) {
#if defined (MY_REGISTRATION_FEATURE) && !defined(MY_GATEWAY_FEATURE)
			_coreConfig.nodeRegistered = _msg.getBool();
			setIndication(INDICATION_GOT_REGISTRATION);
			CORE_DEBUG(PSTR("MCO:PIM:NODE REG=%" PRIu8 "\n"), _coreConfig.nodeRegistered);	// node registration
#endif
		} else if (type == I_CONFIG) {
			// Pick up configuration from controller (currently only metric/imperial) and store it in eeprom if changed
			_coreConfig.controllerConfig.isMetric = _msg.data[0] == 0x00 ||
			                                        _msg.data[0] == 'M'; // metric if null terminated or M
			hwWriteConfigBlock((void*)&_coreConfig.controllerConfig, (void*)EEPROM_CONTROLLER_CONFIG_ADDRESS,
			                   sizeof(controllerConfig_t));
		} else if (type == I_PRESENTATION) {
			// Re-send node presentation to controller
			presentNode();
		} else if (type == I_HEARTBEAT_REQUEST) {
			(void)sendHeartbeat();
		} else if (type == I_VERSION) {
#if !defined(MY_GATEWAY_FEATURE)
			(void)_sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
			                       I_VERSION).set(MYSENSORS_LIBRARY_VERSION_INT));
#endif
		} else if (type == I_TIME) {
			// Deliver time to callback
			if (receiveTime) {
				receiveTime(_msg.getULong());
			}
		}  else if (type == I_CHILDREN) {
			if (_msg.data[0] == 'C') {
#if defined(MY_REPEATER_FEATURE) && defined(MY_SENSOR_NETWORK)
				// Clears child relay data for this node
				setIndication(INDICATION_CLEAR_ROUTING);
				transportClearRoutingTable();
				(void)_sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_CHILDREN).set("OK"));
#endif
			}
		} else if (type == I_DEBUG) {
#if defined(MY_SPECIAL_DEBUG)
			const char debug_msg = _msg.data[0];
			if (debug_msg == 'R') {		// routing table
#if defined(MY_REPEATER_FEATURE) && defined(MY_SENSOR_NETWORK)
				transportReportRoutingTable();
#endif
			} else if (debug_msg == 'V') {	// CPU voltage
				(void)_sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
				                       I_DEBUG).set(hwCPUVoltage()));
			} else if (debug_msg == 'F') {	// CPU frequency in 1/10Mhz
				(void)_sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
				                       I_DEBUG).set(hwCPUFrequency()));
			} else if (debug_msg == 'M') {	// free memory
				(void)_sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
				                       I_DEBUG).set(hwFreeMem()));
			} else if (debug_msg == 'E') {	// clear MySensors eeprom area and reboot
				(void)_sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL, I_DEBUG).set("OK"));
				for (uint16_t i = EEPROM_START; i<EEPROM_LOCAL_CONFIG_ADDRESS; i++) {
					hwWriteConfig(i, 0xFF);
				}
				setIndication(INDICATION_REBOOT);
				hwReboot();
			}
#endif
		} else {
			return false; // further processing required
		}
	} else {
		// sender is a node
		if (type == I_REGISTRATION_REQUEST) {
#if defined(MY_GATEWAY_FEATURE)
			// registration requests are exclusively handled by GW/Controller
#if !defined(MY_REGISTRATION_CONTROLLER)
			bool approveRegistration;

#if defined(MY_CORE_COMPATIBILITY_CHECK)
			approveRegistration = (_msg.getByte() >= MY_CORE_MIN_VERSION);
#else
			// auto registration if version compatible
			approveRegistration = true;
#endif

#if (F_CPU>16*1000000ul)
			// delay for fast GW and slow nodes
			delay(5);
#endif
			(void)_sendRoute(build(_msgTmp, _msg.getSender(), NODE_SENSOR_ID, C_INTERNAL,
			                       I_REGISTRATION_RESPONSE).set(approveRegistration));
#else
			return false;	// processing of this request via controller
#endif
#endif
		} else {
			return false; // further processing required
		}
	}
	return true; // if not GW or no further processing required
}


void saveState(const uint8_t pos, const uint8_t value)
{
	hwWriteConfig(EEPROM_LOCAL_CONFIG_ADDRESS+pos, value);
}
uint8_t loadState(const uint8_t pos)
{
	return hwReadConfig(EEPROM_LOCAL_CONFIG_ADDRESS+pos);
}


void wait(const uint32_t waitingMS)
{
#if defined(MY_DEBUG_VERBOSE_CORE)
	if (waitLock) {
		CORE_DEBUG(PSTR("!MCO:WAI:RC=%" PRIu8 "\n"), waitLock);	// recursive call detected
	}
	waitLock++;
#endif
	const uint32_t enteringMS = hwMillis();
	while (hwMillis() - enteringMS < waitingMS) {
		_process();
	}
#if defined(MY_DEBUG_VERBOSE_CORE)
	waitLock--;
#endif
}

bool wait(const uint32_t waitingMS, const mysensors_command_t cmd)
{
#if defined(MY_DEBUG_VERBOSE_CORE)
	if (waitLock) {
		CORE_DEBUG(PSTR("!MCO:WAI:RC=%" PRIu8 "\n"), waitLock);	// recursive call detected
	}
	waitLock++;
#endif
	const uint32_t enteringMS = hwMillis();
	// invalidate cmd
	//_msg.setCommand(!cmd);
	_msg.setCommand(C_INVALID_7);
	bool expectedResponse = false;
	while ((hwMillis() - enteringMS < waitingMS) && !expectedResponse) {
		_process();
		expectedResponse = (_msg.getCommand() == cmd);
	}
#if defined(MY_DEBUG_VERBOSE_CORE)
	waitLock--;
#endif
	return expectedResponse;
}

bool wait(const uint32_t waitingMS, const mysensors_command_t cmd, const uint8_t msgType)
{
#if defined(MY_DEBUG_VERBOSE_CORE)
	if (waitLock) {
		CORE_DEBUG(PSTR("!MCO:WAI:RC=%" PRIu8 "\n"), waitLock);	// recursive call detected
	}
	waitLock++;
#endif
	const uint32_t enteringMS = hwMillis();
	// invalidate cmd
	//_msg.setCommand(!cmd);
	_msg.setCommand(C_INVALID_7);
	bool expectedResponse = false;
	while ( (hwMillis() - enteringMS < waitingMS) && !expectedResponse ) {
		_process();
		expectedResponse = (_msg.getCommand() == cmd && _msg.getType() == msgType);
	}
#if defined(MY_DEBUG_VERBOSE_CORE)
	waitLock--;
#endif
	return expectedResponse;
}

void doYield(void)
{
	hwWatchdogReset();
	yield();
#if defined (MY_DEFAULT_TX_LED_PIN) || defined(MY_DEFAULT_RX_LED_PIN) || defined(MY_DEFAULT_ERR_LED_PIN)
	ledsProcess();
#endif
}

#if !defined(MY_SLEEP_HANDLER)
void sleepHandler(bool sleep)
{
	(void)sleep;
	// empty function, resolves AVR-specific GCC optimization bug (<5.5) if handler not used
	// see here: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=77326
}
#endif

int8_t _sleep(const uint32_t sleepingMS, const bool smartSleep, const uint8_t interrupt1,
              const uint8_t mode1, const uint8_t interrupt2, const uint8_t mode2)
{
	CORE_DEBUG(PSTR("MCO:SLP:MS=%" PRIu32 ",SMS=%" PRIu8 ",I1=%" PRIu8 ",M1=%" PRIu8 ",I2=%" PRIu8
	                ",M2=%" PRIu8 "\n"), sleepingMS, smartSleep,
	           interrupt1, mode1, interrupt2, mode2);
	// repeater feature: sleeping not possible
#if defined(MY_REPEATER_FEATURE)
	(void)smartSleep;
	(void)interrupt1;
	(void)mode1;
	(void)interrupt2;
	(void)mode2;

	CORE_DEBUG(PSTR("!MCO:SLP:REP\n"));	// sleeping not possible, repeater feature enabled
	wait(sleepingMS);
	return MY_SLEEP_NOT_POSSIBLE;
#else
	uint32_t sleepingTimeMS = sleepingMS;
#if defined(MY_SENSOR_NETWORK)
	// Do not sleep if transport not ready
	if (!isTransportReady()) {
		CORE_DEBUG(PSTR("!MCO:SLP:TNR\n"));	// sleeping not possible, transport not ready
		const uint32_t sleepEnterMS = hwMillis();
		uint32_t sleepDeltaMS = 0;
		while (!isTransportReady() && (sleepDeltaMS < sleepingTimeMS) &&
		        (sleepDeltaMS < MY_SLEEP_TRANSPORT_RECONNECT_TIMEOUT_MS)) {
			_process();
			sleepDeltaMS = hwMillis() - sleepEnterMS;
		}
		// sleep remainder
		if (sleepDeltaMS < sleepingTimeMS) {
			sleepingTimeMS -= sleepDeltaMS;		// calculate remaining sleeping time
			CORE_DEBUG(PSTR("MCO:SLP:MS=%" PRIu32 "\n"), sleepingTimeMS);
		} else {
			// no sleeping time left
			return MY_SLEEP_NOT_POSSIBLE;
		}
	}
	// OTA FW feature: do not sleep if FW update ongoing
#if defined(MY_OTA_FIRMWARE_FEATURE)
	while (isFirmwareUpdateOngoing() && sleepingTimeMS) {
		CORE_DEBUG(PSTR("!MCO:SLP:FWUPD\n"));	// sleeping not possible, FW update ongoing
		wait(1000ul);
		sleepingTimeMS = sleepingTimeMS >= 1000ul ? sleepingTimeMS - 1000ul : 1000ul;
	}
#endif // MY_OTA_FIRMWARE_FEATURE
	if (smartSleep) {
		// sleeping time left?
		if (sleepingTimeMS > 0 && sleepingTimeMS < ((uint32_t)MY_SMART_SLEEP_WAIT_DURATION_MS)) {
			wait(sleepingMS);
			CORE_DEBUG(PSTR("!MCO:SLP:NTL\n"));	// sleeping not possible, no time left
			return MY_SLEEP_NOT_POSSIBLE;
		}
		// notify controller about going to sleep, payload indicates smartsleep waiting time in MS
		(void)_sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
		                       I_PRE_SLEEP_NOTIFICATION).set((uint32_t)MY_SMART_SLEEP_WAIT_DURATION_MS));
		wait(MY_SMART_SLEEP_WAIT_DURATION_MS);		// listen for incoming messages
#if defined(MY_OTA_FIRMWARE_FEATURE)
		// check if during smart sleep waiting period a FOTA request was received
		if (isFirmwareUpdateOngoing()) {
			CORE_DEBUG(PSTR("!MCO:SLP:FWUPD\n"));	// sleeping not possible, FW update ongoing
			return MY_SLEEP_NOT_POSSIBLE;
		}
#endif // MY_OTA_FIRMWARE_FEATURE
	}
#else
	(void)smartSleep;
#endif // MY_SENSOR_NETWORK

#if defined(MY_SENSOR_NETWORK)
	transportDisable();
#endif
	setIndication(INDICATION_SLEEP);

#if defined (MY_DEFAULT_TX_LED_PIN) || defined(MY_DEFAULT_RX_LED_PIN) || defined(MY_DEFAULT_ERR_LED_PIN)
	// Wait until leds finish their blinking pattern
	while (ledsBlinking()) {
		doYield();
	}
#endif

	// Call the sleep handler to turn off peripherals optimally
	sleepHandler(true);

	int8_t result = MY_SLEEP_NOT_POSSIBLE;	// default
	if (interrupt1 != INTERRUPT_NOT_DEFINED && interrupt2 != INTERRUPT_NOT_DEFINED) {
		// both IRQs
		result = hwSleep(interrupt1, mode1, interrupt2, mode2, sleepingTimeMS);
	} else if (interrupt1 != INTERRUPT_NOT_DEFINED && interrupt2 == INTERRUPT_NOT_DEFINED) {
		// one IRQ
		result = hwSleep(interrupt1, mode1, sleepingTimeMS);
	} else if (interrupt1 == INTERRUPT_NOT_DEFINED && interrupt2 == INTERRUPT_NOT_DEFINED) {
		// no IRQ
		result = hwSleep(sleepingTimeMS);
	}

	// Call the sleep handler to turn on peripherals optimally
	sleepHandler(false);

	setIndication(INDICATION_WAKEUP);
	CORE_DEBUG(PSTR("MCO:SLP:WUP=%" PRIi8 "\n"), result);	// sleep wake-up
#if defined(MY_SENSOR_NETWORK)
	transportReInitialise();
#endif
	if (smartSleep) {
		// notify controller about waking up, payload indicates sleeping time in MS
		(void)_sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID, C_INTERNAL,
		                       I_POST_SLEEP_NOTIFICATION).set(sleepingTimeMS));
	}
	return result;
#endif
}

// sleep functions
int8_t sleep(const uint32_t sleepingMS, const bool smartSleep)
{
	return _sleep(sleepingMS, smartSleep);
}

int8_t sleep(const uint8_t interrupt, const uint8_t mode, const uint32_t sleepingMS,
             const bool smartSleep)
{
	return _sleep(sleepingMS, smartSleep, interrupt, mode);
}

int8_t sleep(const uint8_t interrupt1, const uint8_t mode1, const uint8_t interrupt2,
             const uint8_t mode2, const uint32_t sleepingMS, const bool smartSleep)
{
	return _sleep(sleepingMS, smartSleep, interrupt1, mode1, interrupt2, mode2);
}

// deprecated smartSleep() functions
int8_t smartSleep(const uint32_t sleepingMS)
{
	// compatibility
	return _sleep(sleepingMS, true);
}

int8_t smartSleep(const uint8_t interrupt, const uint8_t mode, const uint32_t sleepingMS)
{
	// compatibility
	return _sleep(sleepingMS, true, interrupt, mode);
}

int8_t smartSleep(const uint8_t interrupt1, const uint8_t mode1, const uint8_t interrupt2,
                  const uint8_t mode2, const uint32_t sleepingMS)
{
	// compatibility
	return _sleep(sleepingMS, true, interrupt1, mode1, interrupt2, mode2);
}

uint32_t getSleepRemaining(void)
{
	return hwGetSleepRemaining();
}


void _nodeLock(const char *str)
{
#ifdef MY_NODE_LOCK_FEATURE
	// Make sure EEPROM is updated to locked status
	hwWriteConfig(EEPROM_NODE_LOCK_COUNTER_ADDRESS, 0);
	while (1) {
		setIndication(INDICATION_ERR_LOCKED);
		CORE_DEBUG(PSTR("MCO:NLK:NODE LOCKED. TO UNLOCK, GND PIN %" PRIu8 " AND RESET\n"),
		           MY_NODE_UNLOCK_PIN);
		doYield();
		(void)_sendRoute(build(_msgTmp, GATEWAY_ADDRESS, NODE_SENSOR_ID,C_INTERNAL, I_LOCKED).set(str));
#if defined(MY_SENSOR_NETWORK)
		transportSleep();
		CORE_DEBUG(PSTR("MCO:NLK:TSL\n"));	// sleep transport
#endif
		setIndication(INDICATION_SLEEP);
		(void)hwSleep((uint32_t)1000*60*30); // Sleep for 30 min before resending LOCKED message
		setIndication(INDICATION_WAKEUP);
	}
#else
	(void)str;
#endif
}

void _checkNodeLock(void)
{
#ifdef MY_NODE_LOCK_FEATURE
	// Check if node has been locked down
	if (hwReadConfig(EEPROM_NODE_LOCK_COUNTER_ADDRESS) == 0) {
		// Node is locked, check if unlock pin is asserted, else hang the node
		hwPinMode(MY_NODE_UNLOCK_PIN, INPUT_PULLUP);
		// Make a short delay so we are sure any large external nets are fully pulled
		uint32_t enter = hwMillis();
		while (hwMillis() - enter < 2) {}
		if (hwDigitalRead(MY_NODE_UNLOCK_PIN) == 0) {
			// Pin is grounded, reset lock counter
			hwWriteConfig(EEPROM_NODE_LOCK_COUNTER_ADDRESS, MY_NODE_LOCK_COUNTER_MAX);
			// Disable pullup
			hwPinMode(MY_NODE_UNLOCK_PIN, INPUT);
			setIndication(INDICATION_ERR_LOCKED);
			CORE_DEBUG(PSTR("MCO:BGN:NODE UNLOCKED\n"));
		} else {
			// Disable pullup
			hwPinMode(MY_NODE_UNLOCK_PIN, INPUT);
			_nodeLock("LDB"); //Locked during boot
		}
	} else if (hwReadConfig(EEPROM_NODE_LOCK_COUNTER_ADDRESS) == 0xFF) {
		// Reset value
		hwWriteConfig(EEPROM_NODE_LOCK_COUNTER_ADDRESS, MY_NODE_LOCK_COUNTER_MAX);
	}
#endif
}
#if DOXYGEN

#endif
