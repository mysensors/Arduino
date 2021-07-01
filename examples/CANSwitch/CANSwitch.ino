/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * DESCRIPTION
 *
 * Interrupt driven binary switch example with dual interrupts
 * Author: Patrick 'Anticimex' Fallberg
 * Connect one button or door/window reed switch between
 * digital I/O pin 3 (BUTTON_PIN below) and GND and the other
 * one in similar fashion on digital I/O pin 2.
 * This example is designed to fit Arduino Nano/Pro Mini
 *
 */


// Enable debug prints to serial monitor
#define MY_DEBUG
//#define MY_DEBUG_VERBOSE_CAN

// Enable and select radio type attached
#define MY_CAN
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

#include <MySensors.h>

#define SKETCH_NAME "Binary Sensor"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "0"

//#define PRIMARY_CHILD_ID 3
#define SECONDARY_CHILD_ID 4

//#define PRIMARY_BUTTON_PIN 2   // Arduino Digital I/O pin for button/reed switch
#define SECONDARY_BUTTON_PIN 3 // Arduino Digital I/O pin for button/reed switch

//#if (PRIMARY_BUTTON_PIN < 2 || PRIMARY_BUTTON_PIN > 3)
//#error PRIMARY_BUTTON_PIN must be either 2 or 3 for interrupts to work
//#endif
#if (SECONDARY_BUTTON_PIN < 2 || SECONDARY_BUTTON_PIN > 3)
#error SECONDARY_BUTTON_PIN must be either 2 or 3 for interrupts to work
#endif
//#if (PRIMARY_BUTTON_PIN == SECONDARY_BUTTON_PIN)
//#error PRIMARY_BUTTON_PIN and BUTTON_PIN2 cannot be the same
//#endif
//#if (PRIMARY_CHILD_ID == SECONDARY_CHILD_ID)
//#error PRIMARY_CHILD_ID and SECONDARY_CHILD_ID cannot be the same
//#endif


// Change to V_LIGHT if you use S_LIGHT in presentation below
//MyMessage msg(PRIMARY_CHILD_ID, V_TRIPPED);
MyMessage msg2(SECONDARY_CHILD_ID, V_TRIPPED);

void setup()
{
	// Setup the buttons
//	pinMode(PRIMARY_BUTTON_PIN, INPUT_PULLUP);
	pinMode(SECONDARY_BUTTON_PIN, INPUT_PULLUP);
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

	// Register binary input sensor to sensor_node (they will be created as child devices)
	// You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage.
	// If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
//	present(PRIMARY_CHILD_ID, S_DOOR);
	present(SECONDARY_CHILD_ID, S_DOOR);
}

// Loop will iterate on changes on the BUTTON_PINs
void loop()
{
	uint8_t value;
	static uint8_t sentValue=2;
	static uint8_t sentValue2=2;

	// Short delay to allow buttons to properly settle
	sleep(5);

//	value = digitalRead(PRIMARY_BUTTON_PIN);

//	if (value != sentValue) {
		// Value has changed from last transmission, send the updated value
//		send(msg.set(value==HIGH));
//		sentValue = value;
//	}

	value = digitalRead(SECONDARY_BUTTON_PIN);

	if (value != sentValue2) {
		// Value has changed from last transmission, send the updated value
		send(msg2.set(value==HIGH));
		sentValue2 = value;
	}

	// Sleep until something happens with the sensor
	sleep(SECONDARY_BUTTON_PIN-2, CHANGE, 0);
}
