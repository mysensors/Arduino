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
 * @defgroup MySensorsgrp MySensors
 * @ingroup publics
 * @{
 * @brief The primary public API declaration for the MySensors library
 */

/**
 * @file MySensors.h
 *
 * @brief API declaration for MySensors
 *
 * Include this header into your sketch to include the MySensors library and harness the power of
 * all those sensors!
 */
#ifndef MySensors_h
#define MySensors_h

#ifdef __cplusplus
#include <Arduino.h>
#endif
#include <stdint.h>

#if defined(MY_DIAGNOSTICS)
#if !defined(ARDUINO_ARCH_AVR)
// more flash available
#define MY_DIAGNOSTICS_CRYPTO
#endif
#define MY_DEBUG_VERBOSE_TRANSPORT
#define MY_DEBUG_VERBOSE_TRANSPORT_HAL
#define MY_SPECIAL_DEBUG
#include "core/MyDiagnostics.h"
#endif

#include "MyConfig.h"
#include "core/MyHelperFunctions.cpp"
#include "core/MySplashScreen.h"
#include "core/MySensorsCore.h"

// OTA Debug, has to be defined before HAL
#if defined(MY_OTA_LOG_SENDER_FEATURE) || defined(MY_OTA_LOG_RECEIVER_FEATURE)
#include "core/MyOTALogging.h"
#endif

// HARDWARE
#include "hal/architecture/MyHwHAL.h"
#include "hal/crypto/MyCryptoHAL.h"
#if defined(ARDUINO_ARCH_ESP8266)
#include "hal/architecture/ESP8266/MyHwESP8266.cpp"
#include "hal/crypto/generic/MyCryptoGeneric.cpp"
#elif defined(ARDUINO_ARCH_ESP32)
#include "hal/architecture/ESP32/MyHwESP32.cpp"
#include "hal/crypto/ESP32/MyCryptoESP32.cpp"
#elif defined(ARDUINO_ARCH_AVR)
#include "hal/architecture/AVR/MyHwAVR.cpp"
#include "hal/crypto/AVR/MyCryptoAVR.cpp"
#elif defined(ARDUINO_ARCH_SAMD)
#include "drivers/extEEPROM/extEEPROM.cpp"
#include "hal/architecture/SAMD/MyHwSAMD.cpp"
#include "hal/crypto/generic/MyCryptoGeneric.cpp"
#elif defined(ARDUINO_ARCH_STM32F1)
#include "hal/architecture/STM32F1/MyHwSTM32F1.cpp"
#include "hal/crypto/generic/MyCryptoGeneric.cpp"
#elif defined(ARDUINO_ARCH_NRF5) || defined(ARDUINO_ARCH_NRF52)
#include "hal/architecture/NRF5/MyHwNRF5.cpp"
#include "hal/crypto/generic/MyCryptoGeneric.cpp"
#elif defined(__arm__) && defined(TEENSYDUINO)
#include "hal/architecture/Teensy3/MyHwTeensy3.cpp"
#include "hal/crypto/generic/MyCryptoGeneric.cpp"
#elif defined(__linux__)
#include "hal/architecture/Linux/MyHwLinuxGeneric.cpp"
#include "hal/crypto/generic/MyCryptoGeneric.cpp"
#else
#error Hardware abstraction not defined (unsupported platform)
#endif

#include "hal/architecture/MyHwHAL.cpp"

// commonly used macros, sometimes missing in arch definitions
#if !defined(_BV)
#define _BV(x) (1<<(x))	//!< _BV
#endif

#if !defined(min) && !defined(__linux__)
#define min(a,b) ((a)<(b)?(a):(b)) //!< min
#endif

#if !defined(max) && !defined(__linux__)
#define max(a,b) ((a)>(b)?(a):(b)) //!< max
#endif

#if !defined(MIN)
#define MIN min //!< MIN
#endif

#if !defined(MAX)
#define MAX max //!< MAX
#endif

// OTA Debug second part, depends on HAL
#if defined(MY_OTA_LOG_SENDER_FEATURE) || defined(MY_OTA_LOG_RECEIVER_FEATURE)
#include "core/MyOTALogging.cpp"
#endif

#if defined(MY_LEDS_BLINKING_FEATURE)
#error MY_LEDS_BLINKING_FEATURE is now removed from MySensors core,\
define MY_DEFAULT_ERR_LED_PIN, MY_DEFAULT_TX_LED_PIN or\
MY_DEFAULT_RX_LED_PIN in your sketch instead to enable LEDs
#endif

#if defined(MY_DEFAULT_RX_LED_PIN) || defined(MY_DEFAULT_TX_LED_PIN) || defined(MY_DEFAULT_ERR_LED_PIN)
#include "core/MyLeds.cpp"
#else
#include "core/MyLeds.h"
#endif

#include "core/MyIndication.cpp"


// INCLUSION MODE
#if defined(MY_INCLUSION_MODE_FEATURE)
#include "core/MyInclusionMode.cpp"
#endif


// SIGNING
#include "core/MySigning.cpp"
#if defined(MY_SIGNING_FEATURE)
// SIGNING COMMON FUNCTIONS
#if defined(MY_SIGNING_ATSHA204) && defined(MY_SIGNING_SOFT)
#error Only one signing engine can be activated
#endif
#if defined(MY_SIGNING_ATSHA204) && defined(__linux__)
#error No support for ATSHA204 on this platform
#endif

#if defined(MY_SIGNING_ATSHA204)
#include "core/MySigningAtsha204.cpp"
#include "drivers/ATSHA204/ATSHA204.cpp"
#elif defined(MY_SIGNING_SOFT)
#include "core/MySigningAtsha204Soft.cpp"
#endif
#endif

// FLASH
#if defined(MY_OTA_FIRMWARE_FEATURE)
#ifndef MCUBOOT_PRESENT
#if defined(MY_OTA_USE_I2C_EEPROM)
#include "drivers/I2CEeprom/I2CEeprom.cpp"
#else
#include "drivers/SPIFlash/SPIFlash.cpp"
#endif
#endif
#include "core/MyOTAFirmwareUpdate.cpp"
#endif

// GATEWAY - TRANSPORT
#if defined(MY_CONTROLLER_IP_ADDRESS) || defined(MY_CONTROLLER_URL_ADDRESS)
#define MY_GATEWAY_CLIENT_MODE	//!< gateway client mode
#endif

#if defined(MY_USE_UDP) && !defined(MY_GATEWAY_CLIENT_MODE)
#error You must specify MY_CONTROLLER_IP_ADDRESS or MY_CONTROLLER_URL_ADDRESS for UDP
#endif



// Set MQTT defaults if not set

#if !defined(MY_MQTT_PUBLISH_TOPIC_PREFIX)
#define MY_MQTT_PUBLISH_TOPIC_PREFIX "mygateway1-out"
#endif

#if !defined(MY_MQTT_SUBSCRIBE_TOPIC_PREFIX)
#define MY_MQTT_SUBSCRIBE_TOPIC_PREFIX "mygateway1-in"
#endif

#if !defined(MY_MQTT_CLIENT_ID)
#define MY_MQTT_CLIENT_ID "mysensors-1"
#endif

#if defined(MY_GATEWAY_MQTT_CLIENT)
#if defined(MY_SENSOR_NETWORK)
// We assume that a gateway having a radio also should act as repeater
#define MY_REPEATER_FEATURE
#endif

// GATEWAY - COMMON FUNCTIONS
// We support MQTT Client using W5100, ESP8266, GSM modems supported by TinyGSM library and Linux
#if !defined(MY_GATEWAY_CLIENT_MODE) && !defined(MY_GATEWAY_TINYGSM)
#error You must specify MY_CONTROLLER_IP_ADDRESS or MY_CONTROLLER_URL_ADDRESS
#endif

#if defined(MY_GATEWAY_TINYGSM) && !defined(MY_GATEWAY_MQTT_CLIENT)
// TinyGSM currently only supports MQTTClient mode.
#error MY_GATEWAY_TINYGSM only works with MY_GATEWAY_MQTT_CLIENT
#endif

#include "core/MyGatewayTransport.cpp"
#include "core/MyProtocol.cpp"

#if defined(MY_GATEWAY_TINYGSM)
#include "drivers/TinyGSM/TinyGsmClient.h"
#endif

#if defined(MY_GATEWAY_LINUX)
#include "hal/architecture/Linux/drivers/core/EthernetClient.h"
#include "hal/architecture/Linux/drivers/core/EthernetServer.h"
#include "hal/architecture/Linux/drivers/core/IPAddress.h"
#endif
#include "drivers/PubSubClient/PubSubClient.cpp"
#include "core/MyGatewayTransportMQTTClient.cpp"
#elif defined(MY_GATEWAY_FEATURE)
// GATEWAY - COMMON FUNCTIONS
#include "core/MyGatewayTransport.cpp"
#include "core/MyProtocol.cpp"

// GATEWAY - CONFIGURATION
#if defined(MY_SENSOR_NETWORK)
// We assume that a gateway having a radio also should act as repeater
#define MY_REPEATER_FEATURE
#endif

#if defined(MY_GATEWAY_ESP8266) || defined(MY_GATEWAY_ESP32)
// GATEWAY - ESP8266 / ESP32
#include "core/MyGatewayTransportEthernet.cpp"
#elif defined(MY_GATEWAY_LINUX)
// GATEWAY - Generic Linux
#if defined(MY_USE_UDP)
#error UDP mode is not available for Linux
#endif
#include "hal/architecture/Linux/drivers/core/EthernetClient.h"
#include "hal/architecture/Linux/drivers/core/EthernetServer.h"
#include "hal/architecture/Linux/drivers/core/IPAddress.h"
#include "core/MyGatewayTransportEthernet.cpp"
#elif defined(MY_GATEWAY_W5100)
// GATEWAY - W5100
#include "core/MyGatewayTransportEthernet.cpp"
#elif defined(MY_GATEWAY_ENC28J60)
// GATEWAY - ENC28J60
#if defined(MY_USE_UDP)
#error UDP mode is not available for ENC28J60
#endif
#include "core/MyGatewayTransportEthernet.cpp"
#elif defined(MY_GATEWAY_SERIAL)
// GATEWAY - SERIAL
#include "core/MyGatewayTransportSerial.cpp"
#endif
#endif

// TRANSPORT
#ifndef DOXYGEN
// count enabled transports
#if defined(MY_RADIO_RF24)
#define __RF24CNT 1		//!< __RF24CNT
#else
#define __RF24CNT 0		//!< __RF24CNT
#endif
#if defined(MY_RADIO_NRF5_ESB)
#define __NRF5ESBCNT 1 //!< __NRF5ESBCNT
#else
#define __NRF5ESBCNT 0 //!< __NRF5ESBCNT
#endif
#if defined(MY_RADIO_RFM69)
#define __RFM69CNT 1	//!< __RFM69CNT
#else
#define __RFM69CNT 0	//!< __RFM69CNT
#endif
#if defined(MY_RADIO_RFM95)
#define __RFM95CNT 1	//!< __RFM95CNT
#else
#define __RFM95CNT 0	//!< __RFM95CNT
#endif
#if defined(MY_RS485)
#define __RS485CNT 1	//!< __RS485CNT
#else
#define __RS485CNT 0	//!< __RS485CNT
#endif
#if defined(MY_PJON)
#define _PJONCNT 1	//!< _PJONCNT
#else
#define _PJONCNT 0	//!< _PJONCNT
#endif

#if (__RF24CNT + __NRF5ESBCNT + __RFM69CNT + __RFM95CNT + __RS485CNT + _PJONCNT > 1)
#error Only one forward link driver can be activated
#endif
#endif //DOXYGEN

// SANITY CHECK
#if defined(MY_REPEATER_FEATURE) || defined(MY_GATEWAY_FEATURE)
#define MY_TRANSPORT_SANITY_CHECK		//!< enable regular transport sanity checks
#endif

// TRANSPORT INCLUDES
#if defined(MY_RADIO_RF24) || defined(MY_RADIO_NRF5_ESB) || defined(MY_RADIO_RFM69) || defined(MY_RADIO_RFM95) || defined(MY_RS485) || defined (MY_PJON)
#include "hal/transport/MyTransportHAL.h"
#include "core/MyTransport.h"

// PARENT CHECK
#if defined(MY_PARENT_NODE_IS_STATIC) && (MY_PARENT_NODE_ID == AUTO)
#error Parent is static but no parent ID defined, set MY_PARENT_NODE_ID.
#endif

#if defined(MY_TRANSPORT_DONT_CARE_MODE)
#error MY_TRANSPORT_DONT_CARE_MODE is deprecated, set MY_TRANSPORT_WAIT_READY_MS instead!
#endif

// RAM ROUTING TABLE
#ifdef DOXYGEN
/**
 * @def MY_RAM_ROUTING_TABLE_ENABLED
 * @brief Automatically set if RAM routing table is enabled
 *
 * @see MY_RAM_ROUTING_TABLE_FEATURE
 */
#define MY_RAM_ROUTING_TABLE_ENABLED
#elif defined(MY_RAM_ROUTING_TABLE_FEATURE) && defined(MY_REPEATER_FEATURE)
// activate feature based on architecture
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_NRF5) || defined(ARDUINO_ARCH_STM32F1) || defined(TEENSYDUINO) || defined(__linux__)
#define MY_RAM_ROUTING_TABLE_ENABLED
#elif defined(ARDUINO_ARCH_AVR)
#if defined(__avr_atmega1280__) || defined(__avr_atmega1284__) || defined(__avr_atmega2560__)
// >4kb, enable it
#define MY_RAM_ROUTING_TABLE_ENABLED
#else
// memory limited, enable with care
// #define MY_RAM_ROUTING_TABLE_ENABLED
#endif // __avr_atmega1280__, __avr_atmega1284__, __avr_atmega2560__
#endif // ARDUINO_ARCH_AVR
#endif // DOXYGEN

// SOFTSERIAL
#if defined(MY_GSM_TX) != defined(MY_GSM_RX)
#error Both, MY_GSM_TX and MY_GSM_RX need to be defined when using SoftSerial
#endif

#if defined(MY_GATEWAY_TINYGSM) && !defined(SerialAT) && (!defined(MY_GSM_TX) || !defined(MY_GSM_RX))
#error You need to define either SerialAT or MY_GSM_RX and MY_GSM_TX pins
#endif

// POWER PIN
#ifndef DOXYGEN
#if defined(MY_RF24_POWER_PIN) || defined(MY_RFM69_POWER_PIN) || defined(MY_RFM95_POWER_PIN) || defined(MY_RADIO_NRF5_ESB)
#define RADIO_CAN_POWER_OFF (true)
#else
#define RADIO_CAN_POWER_OFF (false)
#endif
#endif

// Transport drivers
#if defined(MY_RADIO_RF24)
#include "hal/transport/RF24/driver/RF24.cpp"
#include "hal/transport/RF24/MyTransportRF24.cpp"
#elif defined(MY_RADIO_NRF5_ESB)
#if !defined(ARDUINO_ARCH_NRF5)
#error No support for nRF5 radio on this platform
#endif
#include "hal/transport/NRF5_ESB/driver/Radio.cpp"
#include "hal/transport/NRF5_ESB/driver/Radio_ESB.cpp"
#include "hal/transport/NRF5_ESB/MyTransportNRF5_ESB.cpp"
#elif defined(MY_RS485)
#if !defined(MY_RS485_HWSERIAL)
#if defined(__linux__)
#error You must specify MY_RS485_HWSERIAL for RS485 transport
#endif
#include "drivers/AltSoftSerial/AltSoftSerial.cpp"
#endif
#include "hal/transport/RS485/MyTransportRS485.cpp"
#elif defined(MY_RADIO_RFM69)
#if defined(MY_RFM69_NEW_DRIVER)
#include "hal/transport/RFM69/driver/new/RFM69_new.cpp"
#else
#include "hal/transport/RFM69/driver/old/RFM69_old.cpp"
#endif
#include "hal/transport/RFM69/MyTransportRFM69.cpp"
#elif defined(MY_RADIO_RFM95)
#include "hal/transport/RFM95/driver/RFM95.cpp"
#include "hal/transport/RFM95/MyTransportRFM95.cpp"
#elif defined(MY_PJON)
#include "hal/transport/PJON/driver/PJON.h"
#include "hal/transport/PJON/driver/PJONSoftwareBitBang.h"
#if (PJON_BROADCAST == 0)
#error "You must change PJON_BROADCAST to BROADCAST_ADDRESS (255u) and PJON_NOT_ASSIGNED to other one."
#endif
#include "hal/transport/PJON/MyTransportPJON.cpp"
#endif

#if (defined(MY_RF24_ENABLE_ENCRYPTION) && defined(MY_RADIO_RF24)) || (defined(MY_NRF5_ESB_ENABLE_ENCRYPTION) && defined(MY_RADIO_NRF5_ESB)) || (defined(MY_RFM69_ENABLE_ENCRYPTION) && defined(MY_RADIO_RFM69)) || (defined(MY_RFM95_ENABLE_ENCRYPTION) && defined(MY_RADIO_RFM95))
#define MY_TRANSPORT_ENCRYPTION //!< ïnternal flag
#endif

#include "hal/transport/MyTransportHAL.cpp"

// PASSIVE MODE
#if defined(MY_PASSIVE_NODE) && !defined(DOXYGEN)
#define MY_TRANSPORT_UPLINK_CHECK_DISABLED
#define MY_PARENT_NODE_IS_STATIC
#undef MY_REGISTRATION_FEATURE
#undef MY_SIGNING_FEATURE
#undef MY_OTA_FIRMWARE_FEATURE
#if defined(MY_GATEWAY_FEATURE) || defined(MY_REPEATER_FEATURE)
#error This node is configured as GW/repeater, MY_PASSIVE_NODE cannot be set simultaneously
#endif
#if (MY_NODE_ID == AUTO)
#error MY_PASSIVE_NODE configuration requires setting MY_NODE_ID
#endif
#endif

#include "core/MyTransport.cpp"
#endif

// Make sure to disable child features when parent feature is disabled
#if !defined(MY_SENSOR_NETWORK)
#undef MY_OTA_FIRMWARE_FEATURE
#undef MY_REPEATER_FEATURE
#undef MY_SIGNING_NODE_WHITELISTING
#undef MY_SIGNING_FEATURE
#endif

#if !defined(MY_GATEWAY_FEATURE)
#undef MY_INCLUSION_MODE_FEATURE
#undef MY_INCLUSION_BUTTON_FEATURE
#endif

#if !defined(MY_CORE_ONLY)
#if !defined(MY_GATEWAY_FEATURE) && !defined(MY_SENSOR_NETWORK)
#error No forward link or gateway feature activated. This means nowhere to send messages! Pretty pointless.
#endif
#endif

#include "core/MyCapabilities.h"
#include "core/MyMessage.cpp"
#include "core/MySplashScreen.cpp"
#include "core/MySensorsCore.cpp"

#if defined(MY_DIAGNOSTICS)
#include "core/MyDiagnostics.cpp"
#endif

// HW mains
#if defined(ARDUINO_ARCH_AVR)
#include "hal/architecture/AVR/MyMainAVR.cpp"
#elif defined(ARDUINO_ARCH_SAMD)
#include "hal/architecture/SAMD/MyMainSAMD.cpp"
#elif defined(ARDUINO_ARCH_ESP8266)
#include "hal/architecture/ESP8266/MyMainESP8266.cpp"
#elif defined(ARDUINO_ARCH_NRF5)
#include "hal/architecture/NRF5/MyMainNRF5.cpp"
#elif defined(ARDUINO_ARCH_ESP32)
#include "hal/architecture/ESP32/MyMainESP32.cpp"
#elif defined(__linux__)
#include "hal/architecture/Linux/MyMainLinuxGeneric.cpp"
#elif defined(ARDUINO_ARCH_STM32F1)
#include "hal/architecture/STM32F1/MyMainSTM32F1.cpp"
#elif defined(__arm__) && defined(TEENSYDUINO)
#include "hal/architecture/Teensy3/MyMainTeensy3.cpp"
#endif

#endif

/** @}*/
