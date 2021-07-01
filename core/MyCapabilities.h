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
 * @file MyCapabilities.h
 * @ingroup MyCapabilities
 */

#ifndef MyCapabilities_h
#define MyCapabilities_h
/**
 * @defgroup MyCapabilities Node capabilities indicator
 * @ingroup MyConfigGrp
 *
 * @brief MySensors capabilities indications.
 *
 * At node startup, a capabilities string is shown as part of the initialization logs.
 * This string indicate what configuration the node is running with.
 *
 * The string symbols are ordered in the following way:
 * | Setting   |       Reset       |       Radio       |         OTA        |       Node       |   Architecture   |      Signing     |     Buffering     |   Encryption
 * |-----------|-------------------|-------------------|--------------------|------------------|------------------|------------------|-------------------|-----------------
 * | Indicator | @ref MY_CAP_RESET | @ref MY_CAP_RADIO | @ref MY_CAP_OTA_FW | @ref MY_CAP_TYPE | @ref MY_CAP_ARCH | @ref MY_CAP_SIGN | @ref MY_CAP_RXBUF | @ref MY_CAP_ENCR
 *
 * @see MY_CAPABILITIES
 *
 * @{
 */

// Remote reset
/**
 * @def MY_CAP_RESET
 * @brief Indicate the remote reset setting.
 *
 * @see MY_DISABLE_REMOTE_RESET
 *
 * | Setting    | Indicator
 * |------------|----------
 * | Enabled    | R
 * | Disabled   | N
 */
#if defined(MY_DISABLE_REMOTE_RESET)
#define MY_CAP_RESET "N"
#else
#define MY_CAP_RESET "R"
#endif

// OTA firmware update feature
/**
 * @def MY_CAP_OTA_FW
 * @brief Indicate the OTA update setting.
 *
 * @see MY_OTA_FIRMWARE_FEATURE
 *
 * | Setting    | Indicator
 * |------------|----------
 * | Enabled    | O
 * | Disabled   | N
 */
#if defined(MY_OTA_FIRMWARE_FEATURE)
#define MY_CAP_OTA_FW "O"
#else
#define MY_CAP_OTA_FW "N"
#endif

// Transport
/**
 * @def MY_CAP_RADIO
 * @brief Indicate the type of transport selected.
 *
 * @see MY_RADIO_RF24, MY_RADIO_NRF5_ESB, MY_RADIO_RFM69, MY_RFM69_NEW_DRIVER, MY_RADIO_RFM95, MY_RS485
 *
 * | Radio        | Indicator
 * |--------------|----------
 * | nRF24/nRF5   | N
 * | %RFM69 (old) | R
 * | %RFM69 (new) | P
 * | RFM95        | L
 * | RS485        | S
 * | None         | -
 */
#if defined(MY_RADIO_RF24) || defined(MY_RADIO_NRF5_ESB)
#define MY_CAP_RADIO "N"
#elif defined(MY_RADIO_RFM69)
#if !defined(MY_RFM69_NEW_DRIVER)
// old RFM69 driver
#define MY_CAP_RADIO "R"
#else
// new RFM69 driver
#define MY_CAP_RADIO "P"
#endif
#elif defined(MY_RADIO_RFM95)
#define MY_CAP_RADIO "L"
#elif defined(MY_RS485)
#define MY_CAP_RADIO "S"
#else
#define MY_CAP_RADIO "-"
#endif

// Node type
/**
 * @def MY_CAP_TYPE
 * @brief Indicate the type of node.
 *
 * @see MY_GATEWAY_FEATURE, MY_REPEATER_FEATURE, MY_PASSIVE_NODE
 *
 * | Node type | Indicator
 * |-----------|----------
 * | Gateway   | G
 * | Repeater  | R
 * | Passive   | P
 * | Node      | N
 */
#if defined(MY_GATEWAY_FEATURE)
#define MY_CAP_TYPE "G"
#elif defined(MY_REPEATER_FEATURE)
#define MY_CAP_TYPE "R"
#elif defined(MY_PASSIVE_NODE)
#define MY_CAP_TYPE "P"
#else
#define MY_CAP_TYPE "N"
#endif

// Architecture
/**
 * @def MY_CAP_ARCH
 * @brief Indicate the architecture.
 *
 * @see ARDUINO_ARCH_SAMD, ARDUINO_ARCH_NRF5, ARDUINO_ARCH_ESP8266, ARDUINO_ARCH_ESP32, ARDUINO_ARCH_AVR, ARDUINO_ARCH_STM32F1, TEENSYDUINO
 *
 * | Architecture | Indicator
 * |--------------|----------
 * | SAMD         | S
 * | nRF5         | N
 * | ESP8266      | E
 * | ESP32        | F
 * | AVR          | A
 * | STM32F1      | M
 * | TEENSY       | T
 * | Linux        | L
 * | Unknown      | -
 */
#if defined(ARDUINO_ARCH_SAMD)
#define MY_CAP_ARCH "S"
#elif defined(ARDUINO_ARCH_NRF5)
#define MY_CAP_ARCH "N"
#elif defined(ARDUINO_ARCH_ESP8266)
#define MY_CAP_ARCH "E"
#elif defined(ARDUINO_ARCH_ESP32)
#define MY_CAP_ARCH "F"
#elif defined(ARDUINO_ARCH_AVR)
#define MY_CAP_ARCH "A"
#elif defined(ARDUINO_ARCH_STM32F1)
#define MY_CAP_ARCH "F"
#elif defined(ARDUINO_ARCH_STM32)
#define MY_CAP_ARCH "f"
#elif defined(__arm__) && defined(TEENSYDUINO)
#define MY_CAP_ARCH "T"
#elif defined(__linux__)
#define MY_CAP_ARCH "L"
#else
#define MY_CAP_ARCH "-"
#endif

// Signing
/**
 * @def MY_CAP_SIGN
 * @brief Indicate the signing backend used.
 *
 * @see MY_SIGNING_ATSHA204, MY_SIGNING_SOFT
 *
 * | Signing backend | Indicator
 * |-----------------|----------
 * | ATSHA204        | A
 * | Software        | S
 * | No signing      | -
 */
#if defined(MY_SIGNING_ATSHA204)
#define MY_CAP_SIGN "A"
#elif defined(MY_SIGNING_SOFT)
#define MY_CAP_SIGN "S"
#else
#define MY_CAP_SIGN "-"
#endif

// RX queue
/**
 * @def MY_CAP_RXBUF
 * @brief Indicate the rx message buffer setting.
 *
 * @see MY_RX_MESSAGE_BUFFER_FEATURE
 *
 * | Setting    | Indicator
 * |------------|----------
 * | Enabled    | Q
 * | Disabled   | -
 */
#if defined(MY_RX_MESSAGE_BUFFER_FEATURE)
#define MY_CAP_RXBUF "Q"
#else
#define MY_CAP_RXBUF "-"
#endif

// Radio encryption
/**
 * @def MY_CAP_ENCR
 * @brief Indicate the encryption setting.
 *
 * @see MY_ENCRYPTION_FEATURE
 *
 * | Setting    | Indicator
 * |------------|----------
 * | Enabled    | X
 * | Disabled   | -
 */
#if defined(MY_ENCRYPTION_FEATURE)
#define MY_CAP_ENCR "X"
#else
#define MY_CAP_ENCR "-"
#endif


/**
 * @def MY_CAPABILITIES
 * @brief This is the resulting capabilities string.
 *
 * @see MY_CAP_RESET, MY_CAP_RADIO, MY_CAP_OTA_FW, MY_CAP_TYPE, MY_CAP_ARCH, MY_CAP_SIGN, MY_CAP_RXBUF, MY_CAP_ENCR
 */
#define MY_CAPABILITIES MY_CAP_RESET MY_CAP_RADIO MY_CAP_OTA_FW MY_CAP_TYPE MY_CAP_ARCH MY_CAP_SIGN MY_CAP_RXBUF MY_CAP_ENCR

/** @}*/ // End of MyCapabilities group
#endif /* MyCapabilities_h */
