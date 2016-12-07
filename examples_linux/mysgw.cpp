/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2016 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <iostream>
#include <cstdio>
#include <unistd.h>

// For more options run ./configure --help

// Config file
//#define MY_LINUX_CONFIG_FILE "/etc/mysensors.dat"

// How many clients should be able to connect to this gateway (default 1)
#define MY_GATEWAY_MAX_CLIENTS 10

// Serial config
// Enable this if you are using an Arduino connected to the USB
//#define MY_LINUX_SERIAL_PORT "/dev/ttyACM0"
// Enable this if you need to connect to a controller running on the same device
//#define MY_IS_SERIAL_PTY
// Choose a symlink name for the PTY device
//#define MY_LINUX_SERIAL_PTY "/dev/ttyMySensorsGateway"
// Grant access to the specified system group for the serial device
//#define MY_LINUX_SERIAL_GROUPNAME "tty"

// MQTT options
//#define MY_CONTROLLER_IP_ADDRESS 192, 168, 178, 68
//#define MY_PORT 1883
//#define MY_MQTT_CLIENT_ID "mysensors-1"
//#define MY_MQTT_PUBLISH_TOPIC_PREFIX "mygateway1-out"
//#define MY_MQTT_SUBSCRIBE_TOPIC_PREFIX "mygateway1-in"

// Enable these if your MQTT broker requires usenrame/password
//#define MY_MQTT_USER "username"
//#define MY_MQTT_PASSWORD "password"

// Flash leds on rx/tx/err
//#define MY_DEFAULT_ERR_LED_PIN 12  // Error LED pin
//#define MY_DEFAULT_RX_LED_PIN  16  // Receive LED pin
//#define MY_DEFAULT_TX_LED_PIN  18  // Transmit LED pin
// Inverse the blinking feature
//#define MY_WITH_LEDS_BLINKING_INVERSE

// Enable software signing
//#define MY_SIGNING_SOFT
// Enable signing related debug
//#define MY_DEBUG_VERBOSE_SIGNING
// Enable this to request signatures from nodes that in turn request signatures from gateway
//#define MY_SIGNING_REQUEST_SIGNATURES
// Enable this to have gateway require all nodes in the network to sign messages sent to it
// Note: MY_SIGNING_REQUEST_SIGNATURES must also be set
//#define MY_SIGNING_GW_REQUEST_SIGNATURES_FROM_ALL

// Enables RF24 encryption (all nodes and gateway must have this enabled, and all must be
// personalized with the same AES key)
//#define MY_RF24_ENABLE_ENCRYPTION

#include <MySensors.h>

#define ARDUINO 100
// This space is intended to be used to include arduino libraries

#undef ARDUINO

void setup()
{
	// Setup locally attached sensors
}

void presentation()
{
	// Present locally attached sensors here
}

void loop()
{
	// Send locally attached sensors data here
}
