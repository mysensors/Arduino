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
 */
/**
 * @ingroup MySigninggrp
 * @{
 * @file SecurityPersonalizer.ino
 * @brief Security personalization sketch
 *
 * REVISION HISTORY
 *  - See git log (git log libraries/MySensors/examples/SecurityPersonalizer/SecurityPersonalizer.ino)
 */

/**
 * @example SecurityPersonalizer.ino
 * This sketch will personalize either none-volatile memory or ATSHA204A for security functions
 * available in the MySensors library.<br>
 * Details on personalization procedure is given in @ref personalization.<br>
 * This sketch will when executed without modifications also print a guided workflow on the UART.
 */

#include "sha204_library.h"
#include "sha204_lib_return_codes.h"
/** @brief Make use of the MySensors framework without invoking the entire system */
#define MY_CORE_ONLY
#include <MySensors.h>

/************************************ User defined key data ***************************************/

/** @brief The user-defined HMAC key to use unless @ref GENERATE_HMAC_KEY is set */
#define MY_HMAC_KEY 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

/** @brief The user-defined AES key to store in EEPROM unless @ref GENERATE_AES_KEY is set */
#define MY_AES_KEY 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

/** @brief The user-defined soft serial to use for soft signing unless @ref GENERATE_SOFT_SERIAL is set */
#define MY_SOFT_SERIAL 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF

/***************************** Flags for guided personalization flow ******************************/

/**
 * @def GENERATE_KEYS_ATSHA204A
 * @brief Default settings for generating keys using ATSHA204A
 *
 * @note The generated keys displayed in the serial log with this setting needs to be written down
 *       and transferred to all nodes this gateway will communicate with. This is mandatory for ALL
 *       nodes for encryption (AES key). For signing (HMAC key) it is only required for nodes that
 *       use signing. Typically you set the values for @ref MY_HMAC_KEY and @ref MY_AES_KEY.
 */
//#define GENERATE_KEYS_ATSHA204A

/**
 * @def GENERATE_KEYS_SOFT
 * @brief Default settings for generating keys using software
 *
 * @b Important<br>
 * You will need to ensure @ref MY_SIGNING_SOFT_RANDOMSEED_PIN is set to an unconnected analog pin
 * in order to provide entropy to the software RNG if your hardware has no HWRNG.
 *
 * @note The generated keys displayed in the serial log with this setting needs to be written down
 *       and transferred to all nodes this gateway will communicate with. This is mandatory for ALL
 *       nodes for encryption (AES key). For signing (HMAC key) it is only required for nodes that
 *       use signing. Typically you set the values for @ref MY_HMAC_KEY and @ref MY_AES_KEY.
 */
//#define GENERATE_KEYS_SOFT

/**
 * @def PERSONALIZE_ATSHA204A
 * @brief Default settings for personalizing an ATSHA204A
 *
 * It is assumed that you have updated @ref MY_HMAC_KEY and @ref MY_AES_KEY with the keys displayed
 * when executing this sketch with @ref GENERATE_KEYS_ATSHA204A or @ref GENERATE_KEYS_SOFT defined.
 */
//#define PERSONALIZE_ATSHA204A

/**
 * @def PERSONALIZE_SOFT
 * @brief Default settings for personalizing EEPROM for software signing
 *
 * It is assumed that you have updated @ref MY_HMAC_KEY and @ref MY_AES_KEY with the keys displayed
 * when executing this sketch with @ref GENERATE_KEYS_ATSHA204A or @ref GENERATE_KEYS_SOFT defined.
 */
//#define PERSONALIZE_SOFT

/**
 * @def PERSONALIZE_SOFT_RANDOM_SERIAL
 * @brief This is an alternative to @ref PERSONALIZE_SOFT which will also store a randomly generated
 * serial to EEPROM in addition to the actions performed by @ref PERSONALIZE_SOFT. Take note of the
 * generated soft serial as it will be needed if you plan to use whitelisting. It should be
 * unique for each node.
 *
 * @note This is only needed for targets that lack unique device IDs. The sketch will inform you if
 *       there is a need for generating a random serial or not. Check the "Hardware security
 *       peripherals" listing. If a target has a unique device ID and a serial in EEPROM, the serial
 *       in EEPROM will be used. If erased (replaced with FF:es) the unique device ID will be used
 *       instead.
 */
//#define PERSONALIZE_SOFT_RANDOM_SERIAL

/*************************** The settings below are for advanced users ****************************/
/**
 * @def USE_SOFT_SIGNING
 * @brief Uncomment this to generate keys by software and store them to EEPROM instead of ATSHA204A
 */
//#define USE_SOFT_SIGNING

/**
 * @def LOCK_ATSHA204A_CONFIGURATION
 * @brief Uncomment this to enable locking the ATSHA204A configuration zone
 *
 * It is still possible to change the key, and this also enable random key generation.
 * @warning BE AWARE THAT THIS PREVENTS ANY FUTURE CONFIGURATION CHANGE TO THE CHIP
 */
//#define LOCK_ATSHA204A_CONFIGURATION

/**
 * @def SKIP_UART_CONFIRMATION
 * @brief Uncomment this for boards that lack UART
 *
 * This will disable additional confirmation for actions that are non-reversible.
 *
 * @b Important<br> For ATSHA204A, no confirmation will be required for locking any zones with this
 * configuration! Also, if you generate keys on a board without UART, you have no way of determining
 * what the key is unless it is stored in EEPROM.
 */
//#define SKIP_UART_CONFIRMATION

/**
 * @def GENERATE_HMAC_KEY
 * @brief Uncomment this to generate a random HMAC key using ATSHA204A or software depending on
 *        @ref USE_SOFT_SIGNING
 * @note If not enabled, key defined by @ref MY_HMAC_KEY will be used instead.
 */
//#define GENERATE_HMAC_KEY

/**
 * @def STORE_HMAC_KEY
 * @brief Uncomment this to store HMAC key to ATSHA204A or EEPROM depending on @ref USE_SOFT_SIGNING
 */
//#define STORE_HMAC_KEY

/**
 * @def GENERATE_AES_KEY
 * @brief Uncomment this to generate a random AES key using ATSHA204A or software depending on
 * @ref USE_SOFT_SIGNING
 * @note If not enabled, key defined by @ref MY_AES_KEY will be used instead.
 */
//#define GENERATE_AES_KEY

/**
 * @def STORE_AES_KEY
 * @brief Uncomment this to store AES key to EEPROM
 */
//#define STORE_AES_KEY

/**
 * @def GENERATE_SOFT_SERIAL
 * @brief Uncomment this to generate a random serial number for software signing
 * @note If not enabled, serial defined by @ref MY_SOFT_SERIAL will be used instead.
 */
//#define GENERATE_SOFT_SERIAL

/**
 * @def STORE_SOFT_SERIAL
 * @brief Uncomment this to store the serial number to EEPROM
 */
//#define STORE_SOFT_SERIAL

/**
 * @def PRINT_DETAILED_ATSHA204A_CONFIG
 * @brief Uncomment to print the detailed ATSHA204A configuration
 */
//#define PRINT_DETAILED_ATSHA204A_CONFIG

/**
 * @def RESET_EEPROM_PERSONALIZATION
 * @brief Uncomment to reset the personalization data in EEPROM to 0xFF:es
 */
//#define RESET_EEPROM_PERSONALIZATION

/********************* Guided mode flag configurations (don't change these) ***********************/
#ifdef GENERATE_KEYS_ATSHA204A
#define LOCK_ATSHA204A_CONFIGURATION // We have to lock configuration to enable random number generation
#define GENERATE_HMAC_KEY // Generate random HMAC key
#define GENERATE_AES_KEY // Generate random AES key
#define SKIP_UART_CONFIRMATION // This is an automated mode
#endif

#ifdef GENERATE_KEYS_SOFT
#define USE_SOFT_SIGNING // Use software backend
#define GENERATE_HMAC_KEY // Generate random HMAC key
#define GENERATE_AES_KEY // Generate random AES key
#define SKIP_UART_CONFIRMATION // This is an automated mode
#endif

#ifdef PERSONALIZE_ATSHA204A
#define LOCK_ATSHA204A_CONFIGURATION // We have to lock configuration to enable random number generation
#define STORE_HMAC_KEY // Store the HMAC key
#define STORE_AES_KEY // Store the AES key
#define SKIP_UART_CONFIRMATION // This is an automated mode
#endif

#ifdef PERSONALIZE_SOFT_RANDOM_SERIAL
#define GENERATE_SOFT_SERIAL // Generate a soft serial number
#define PERSONALIZE_SOFT // Do the rest as PERSONALIZE_SOFT
#endif

#ifdef PERSONALIZE_SOFT
#define USE_SOFT_SIGNING // Use software backend
#define STORE_HMAC_KEY // Store the HMAC key
#define STORE_AES_KEY // Store the AES key
#define STORE_SOFT_SERIAL // Store the soft serial number
#define SKIP_UART_CONFIRMATION // This is an automated mode
#endif

#if defined(GENERATE_HMAC_KEY) || defined(GENERATE_AES_KEY) || defined(GENERATE_SOFT_SERIAL)
#define GENERATE_SOMETHING
#endif

#if defined(MY_LOCK_MCU)
#undefine MY_LOCK_MCU  // The Sketch after SecurityPersonaliter should lock the MCU
#endif

/********************************** Preprocessor sanitychecks *************************************/
#if defined(GENERATE_SOFT_SERIAL) && !defined(USE_SOFT_SIGNING)
#error Cannot generate soft serial using ATSHA204A, use USE_SOFT_SINGING for this
#endif
#if defined(STORE_SOFT_SERIAL) && !defined(USE_SOFT_SIGNING)
#error Cannot store soft serial to ATSHA204A, use USE_SOFT_SINGING for this
#endif
#if defined(PRINT_DETAILED_ATSHA204A_CONFIG) && defined(USE_SOFT_SIGNING)
#error Cannot print ATSHA204A config using software signing flag, disable USE_SOFT_SINGING for this
#endif
#if defined(LOCK_ATSHA204A_CONFIGURATION) && defined(USE_SOFT_SIGNING)
#error Cannot lock ATSHA204A config using software signing flag, disable USE_SOFT_SINGING for this
#endif
#ifdef GENERATE_KEYS_ATSHA204A
#ifdef USE_SOFT_SIGNING
#error You cannot select soft signing if you want to generate keys using ATSHA204A
#endif
#ifdef STORE_HMAC_KEY
#error Disable STORE_SOFT_KEY, you should not store keys in this mode
#endif
#ifdef STORE_SOFT_SERIAL
#error Disable STORE_SOFT_SERIAL, you should not store serial in this mode
#endif
#ifdef STORE_AES_KEY
#error Disable STORE_AES_KEY, you should not store keys in this mode
#endif
#if defined(GENERATE_KEYS_SOFT) ||\
		defined (PERSONALIZE_ATSHA204A) ||\
		defined (PERSONALIZE_SOFT) ||\
		defined (PERSONALIZE_SOFT_RANDOM_SERIAL)
#error You can not enable GENERATE_KEYS_ATSHA204A together with other guided modes
#endif
#endif // GENERATE_KEYS_ATSHA204A
#ifdef GENERATE_KEYS_SOFT
#ifdef STORE_HMAC_KEY
#error Disable STORE_SOFT_KEY, you should not store keys in this mode
#endif
#ifdef STORE_SOFT_SERIAL
#error Disable STORE_SOFT_SERIAL, you should not store serial in this mode
#endif
#ifdef STORE_AES_KEY
#error Disable STORE_AES_KEY, you should not store keys in this mode
#endif
#ifndef MY_SIGNING_SOFT_RANDOMSEED_PIN
#error You have to set MY_SIGNING_SOFT_RANDOMSEED_PIN to a suitable value in this mode
#endif
#if defined(GENERATE_KEYS_ATSHA204A) ||\
		defined (PERSONALIZE_ATSHA204A) ||\
		defined (PERSONALIZE_SOFT) ||\
		defined (PERSONALIZE_SOFT_RANDOM_SERIAL)
#error You can not enable GENERATE_KEYS_SOFT together with other guided modes
#endif
#endif // GENERATE_KEYS_SOFT
#ifdef PERSONALIZE_ATSHA204A
#ifdef USE_SOFT_SIGNING
#error You cannot select soft signing if you want to personalize an ATSHA204A
#endif
#if defined(GENERATE_KEYS_ATSHA204A) ||\
		defined (GENERATE_KEYS_SOFT) ||\
		defined (PERSONALIZE_SOFT) ||\
		defined (PERSONALIZE_SOFT_RANDOM_SERIAL)
#error You can not enable PERSONALIZE_ATSHA204A together with other guided modes
#endif
#ifdef RESET_EEPROM_PERSONALIZATION
#error You cannot reset EEPROM personalization when personalizing a device
#endif
#endif // PERSONALIZE_ATSHA204A
#ifdef PERSONALIZE_SOFT
#if defined(GENERATE_KEYS_ATSHA204A) ||\
		defined (GENERATE_KEYS_SOFT) ||\
		defined (PERSONALIZE_ATSHA204A)
#error You can not enable PERSONALIZE_SOFT together with other guided modes
#endif
#ifdef RESET_EEPROM_PERSONALIZATION
#error You cannot reset EEPROM personalization when personalizing a device
#endif
#endif // PERSONALIZE_SOFT
#ifdef PERSONALIZE_SOFT_RANDOM_SERIAL
#if defined(GENERATE_KEYS_SOFT) ||\
		defined (PERSONALIZE_ATSHA204A) ||\
		defined (GENERATE_KEYS_ATSHA204A)
#error You can only enable one of the guided modes at a time
#endif
#ifdef RESET_EEPROM_PERSONALIZATION
#error You cannot reset EEPROM personalization when personalizing a device
#endif
#endif // PERSONALIZE_SOFT_RANDOM_SERIAL

#if !defined(GENERATE_KEYS_ATSHA204A) &&\
		!defined(GENERATE_KEYS_SOFT) &&\
		!defined(PERSONALIZE_ATSHA204A) &&\
		!defined(PERSONALIZE_SOFT) &&\
		!defined(PERSONALIZE_SOFT_RANDOM_SERIAL) &&\
		!defined(USE_SOFT_SIGNING) &&\
		!defined(LOCK_ATSHA204A_CONFIGURATION) &&\
		!defined(SKIP_UART_CONFIRMATION) &&\
		!defined(GENERATE_HMAC_KEY) &&\
		!defined(STORE_HMAC_KEY) &&\
		!defined(GENERATE_SOFT_SERIAL) &&\
		!defined(STORE_SOFT_SERIAL) &&\
		!defined(GENERATE_AES_KEY) &&\
		!defined(STORE_AES_KEY) &&\
		!defined(PRINT_DETAILED_ATSHA204A_CONFIG) &&\
		!defined(RESET_EEPROM_PERSONALIZATION)
/** @brief Set when there are no config flags defined */
#define NO_SETTINGS_DEFINED
#endif

#if defined(GENERATE_KEYS_ATSHA204A) ||\
		defined(GENERATE_KEYS_SOFT) ||\
		defined(PERSONALIZE_ATSHA204A) ||\
		defined(PERSONALIZE_SOFT) ||\
		defined(PERSONALIZE_SOFT_RANDOM_SERIAL)
/** @brief Set when there is a guided mode flag defined */
#define GUIDED_MODE
#endif

/************************************* Function declarations ***************************************/
static void halt(bool success);
#ifdef GENERATE_SOMETHING
static bool generate_random_data(uint8_t* data, size_t sz);
#endif
static void generate_keys(void);
static void store_keys(void);
static void print_hex_buffer(uint8_t* data, size_t sz);
static void print_c_friendly_hex_buffer(uint8_t* data, size_t sz);
#ifdef STORE_HMAC_KEY
static bool store_hmac_key_data(uint8_t* data, size_t sz);
#endif
#ifdef STORE_AES_KEY
static bool store_aes_key_data(uint8_t* data, size_t sz);
#endif
#ifdef STORE_SOFT_SERIAL
static bool store_soft_serial_data(uint8_t* data, size_t sz);
#endif
#ifndef USE_SOFT_SIGNING
static void init_atsha204a_state(void);
#ifdef LOCK_ATSHA204A_CONFIGURATION
static void	lock_atsha204a_config(void);
static uint16_t write_atsha204a_config_and_get_crc(void);
#endif
static bool get_atsha204a_serial(uint8_t* data);
#ifdef STORE_HMAC_KEY
static bool write_atsha204a_key(uint8_t* key);
#endif
#endif // not USE_SOFT_SIGNING
static void print_greeting(void);
static void print_ending(void);
static void	probe_and_print_peripherals(void);
static void print_eeprom_data(void);
static void print_whitelisting_entry(void);
#ifdef PRINT_DETAILED_ATSHA204A_CONFIG
static void dump_detailed_atsha204a_configuration(void);
#endif
#ifdef RESET_EEPROM_PERSONALIZATION
static void reset_eeprom(void);
#endif
static void write_eeprom_checksum(void);

/**************************************** File local data *****************************************/
#if defined(GENERATE_HMAC_KEY) || defined(STORE_HMAC_KEY)
/** @brief The data to store as HAMC key in ATSHA204A or EEPROM */
static uint8_t user_hmac_key[32] = {MY_HMAC_KEY};
#endif

#if defined(GENERATE_SOFT_SERIAL) || defined(STORE_SOFT_SERIAL)
/** @brief The data to store as soft serial in EEPROM */
static uint8_t user_soft_serial[9] = {MY_SOFT_SERIAL};
#endif

#if defined(GENERATE_AES_KEY) || defined(STORE_AES_KEY)
/* @brief The data to store as AES key in EEPROM */
static uint8_t user_aes_key[16] = {MY_AES_KEY};
#endif

#ifndef USE_SOFT_SIGNING
const int sha204Pin = MY_SIGNING_ATSHA204_PIN; //!< The IO pin to use for ATSHA204A
atsha204Class sha204(sha204Pin);  //!< atsha204Class
static uint8_t tx_buffer[SHA204_CMD_SIZE_MAX];
static uint8_t rx_buffer[SHA204_RSP_SIZE_MAX];
static uint8_t ret_code;
static uint8_t lockConfig = 0;
static uint8_t lockValue = 0;
#endif
static bool has_device_unique_id = false;
static const uint8_t reset_buffer[32] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/******************************************* Functions ********************************************/

/** @brief Sketch setup code (all personalization is done here as it is a run-once sketch) */
void setup()
{
	// Delay startup a bit for serial consoles to catch up
	uint32_t enter = hwMillis();
	while (hwMillis() - enter < (uint32_t)500);
#ifdef USE_SOFT_SIGNING
	// initialize pseudo-RNG
	hwRandomNumberInit();
#endif

	while(!Serial); // For USB enabled devices, wait for USB enumeration before continuing

	print_greeting();

#ifndef USE_SOFT_SIGNING
	init_atsha204a_state();
	// Lock configuration now if requested to enable RNG in ATSHA
#ifdef LOCK_ATSHA204A_CONFIGURATION
	lock_atsha204a_config();
#endif
#endif
	// Generate the requested keys (if any)
	generate_keys();

#ifdef RESET_EEPROM_PERSONALIZATION
	// If requested, reset EEPROM before storing keys
	reset_eeprom();
#endif

	// Store the keys (if configured to do so)
	store_keys();

	// Write a checksum on the EEPROM data
	write_eeprom_checksum();

	// Print current EEPROM
	print_eeprom_data();
	print_whitelisting_entry();
	Serial.println();

	print_ending();
	halt(true);
}

/** @brief Sketch execution code (unused) */
void loop()
{
}

/** @brief Print a notice and halt the execution */
static void halt(bool success)
{
	Serial.println();
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("|                                  Execution result                                  |"));
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	if (success) {
		Serial.println(F(
		                   "| SUCCESS                                                                            |"));
	} else {
		Serial.print(F(
		                 "| FAILURE "));
#ifdef USE_SOFT_SIGNING
		Serial.println(F("                                                                           |"));
#else
		if (ret_code != SHA204_SUCCESS) {
			Serial.print(F("(last ATSHA204A return code: 0x"));
			if (ret_code < 0x10) {
				Serial.print('0'); // Because Serial.print does not 0-pad HEX
			}
			Serial.print(ret_code, HEX);
			Serial.println(F(")                                         |"));
		} else {
			Serial.println(F("                                                                           |"));
		}
#endif
	}
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	while(1) {
		doYield();
	}
}

#ifdef GENERATE_SOMETHING
/**
 * @brief Generate random data of specified size and store in provided buffer
 * @param data Buffer to store bytes in (buffer is assumed to be of required size)
 * @param sz Number of random bytes to generate
 * @returns false if failed
 */
static bool generate_random_data(uint8_t* data, size_t sz)
{
#if defined(USE_SOFT_SIGNING) && ! defined(MY_HW_HAS_GETENTROPY)
	for (size_t i = 0; i < sz; i++) {
		data[i] = random(256) ^ micros();
		uint32_t enter = hwMillis();
		while (hwMillis() - enter < (uint32_t)2);
	}
	return true;
#elif defined(USE_SOFT_SIGNING) && defined(MY_HW_HAS_GETENTROPY)
	hwGetentropy(&data, sz);
#else
	ret_code = sha204.sha204m_random(tx_buffer, rx_buffer, RANDOM_SEED_UPDATE);
	if ((ret_code != SHA204_SUCCESS) || (lockConfig != 0x00)) {
		return false;
	} else {
		memcpy(data, rx_buffer+SHA204_BUFFER_POS_DATA, sz);
		return true;
	}
#endif // not USE_SOFT_SIGNING
}
#endif // GENERATE_SOMETHING

/** @brief Generate keys depending on configuration */
static void generate_keys(void)
{
#ifdef GENERATE_SOMETHING
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("|                                   Key generation                                   |"));
	Serial.println(
	    F("+--------+--------+------------------------------------------------------------------+"));
	Serial.println(
	    F("| Key ID | Status | Key                                                              |"));
	Serial.println(
	    F("+--------+--------+------------------------------------------------------------------+"));
#endif
#ifdef GENERATE_HMAC_KEY
	Serial.print(F("| HMAC   | "));
	if (!generate_random_data(user_hmac_key, 32)) {
		memset(user_hmac_key, 0xFF, 32);
		Serial.print(F("FAILED | "));
	} else {
		Serial.print(F("OK     | "));
	}
	print_hex_buffer(user_hmac_key, 32);
	Serial.println(F(" |"));
#endif
#ifdef GENERATE_AES_KEY
	Serial.print(F("| AES    | "));
	if (!generate_random_data(user_aes_key, 16)) {
		memset(user_aes_key, 0xFF, 16);
		Serial.print(F("FAILED | "));
	} else {
		Serial.print(F("OK     | "));
	}
	print_hex_buffer(user_aes_key, 16);
	Serial.println(F("                                 |"));
#endif
#ifdef GENERATE_SOFT_SERIAL
	Serial.print(F("| SERIAL | "));
	if (has_device_unique_id) {
		Serial.println(F("N/A    | MCU has a unique serial which will be used instead.              |"));
	} else {
		if (!generate_random_data(user_soft_serial, 9)) {
			memset(user_soft_serial, 0xFF, 9);
			Serial.print(F("FAILED | "));
		} else {
			Serial.print(F("OK     | "));
		}
		print_hex_buffer(user_soft_serial, 9);
		Serial.println(F("                                               |"));
	}
#endif
#if defined(GENERATE_SOMETHING) && !defined(PERSONALIZE_SOFT_RANDOM_SERIAL)
	Serial.println(
	    F("+--------+--------+------------------------------------------------------------------+"));
	Serial.println();
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("|                                  Key copy section                                  |"));
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
#ifdef GENERATE_HMAC_KEY
	Serial.print(F("#define MY_HMAC_KEY "));
	print_c_friendly_hex_buffer(user_hmac_key, 32);
	Serial.println();
#endif
#ifdef GENERATE_AES_KEY
	Serial.print(F("#define MY_AES_KEY "));
	print_c_friendly_hex_buffer(user_aes_key, 16);
	Serial.println();
#endif
#ifdef GENERATE_SOFT_SERIAL
	Serial.print(F("#define MY_SOFT_SERIAL "));
	print_c_friendly_hex_buffer(user_soft_serial, 9);
	Serial.println();
#endif
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println();
#elif defined(PERSONALIZE_SOFT_RANDOM_SERIAL)
	Serial.println(
	    F("+--------+--------+------------------------------------------------------------------+"));
	Serial.println();
#endif
}

#ifdef RESET_EEPROM_PERSONALIZATION
/** @brief Reset EEPROM */
static void	reset_eeprom(void)
{
	uint8_t validation_buffer[32];
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("|                                   EEPROM reset                                     |"));
	Serial.println(
	    F("+--------+---------------------------------------------------------------------------+"));
	Serial.println(
	    F("| Key ID | Status                                                                    |"));
	Serial.println(
	    F("+--------+---------------------------------------------------------------------------+"));
	Serial.print(F("| HMAC   | "));
	hwWriteConfigBlock((void*)reset_buffer, (void*)EEPROM_SIGNING_SOFT_HMAC_KEY_ADDRESS, 32);
	// Validate data written
	hwReadConfigBlock((void*)validation_buffer, (void*)EEPROM_SIGNING_SOFT_HMAC_KEY_ADDRESS, 32);
	if (memcmp(validation_buffer, reset_buffer,	32) != 0) {
		Serial.println(F("FAILED                                                                    |"));
	} else {
		Serial.println(F("OK                                                                        |"));
	}
	Serial.print(F("| AES    | "));
	hwWriteConfigBlock((void*)reset_buffer, (void*)EEPROM_RF_ENCRYPTION_AES_KEY_ADDRESS, 16);
	// Validate data written
	hwReadConfigBlock((void*)validation_buffer, (void*)EEPROM_RF_ENCRYPTION_AES_KEY_ADDRESS, 16);
	if (memcmp(validation_buffer, reset_buffer,	16) != 0) {
		Serial.println(F("FAILED                                                                    |"));
	} else {
		Serial.println(F("OK                                                                        |"));
	}
	Serial.print(F("| SERIAL | "));
	hwWriteConfigBlock((void*)reset_buffer, (void*)EEPROM_SIGNING_SOFT_SERIAL_ADDRESS, 9);
	// Validate data written
	hwReadConfigBlock((void*)validation_buffer, (void*)EEPROM_SIGNING_SOFT_SERIAL_ADDRESS, 9);
	if (memcmp(validation_buffer, reset_buffer,	9) != 0) {
		Serial.println(F("FAILED                                                                    |"));
	} else {
		Serial.println(F("OK                                                                        |"));
	}
	Serial.println(
	    F("+--------+---------------------------------------------------------------------------+"));
}
#endif // RESET_EEPROM_PERSONALIZATION

static void write_eeprom_checksum(void)
{
	uint8_t buffer[SIZE_SIGNING_SOFT_HMAC_KEY + SIZE_RF_ENCRYPTION_AES_KEY + SIZE_SIGNING_SOFT_SERIAL];
	uint8_t hash[32];
	uint8_t checksum;
	hwReadConfigBlock((void*)buffer, (void*)EEPROM_SIGNING_SOFT_HMAC_KEY_ADDRESS,
	                  SIZE_SIGNING_SOFT_HMAC_KEY);
	hwReadConfigBlock((void*)&buffer[SIZE_SIGNING_SOFT_HMAC_KEY],
	                  (void*)EEPROM_RF_ENCRYPTION_AES_KEY_ADDRESS, SIZE_RF_ENCRYPTION_AES_KEY);
	hwReadConfigBlock((void*)&buffer[SIZE_SIGNING_SOFT_HMAC_KEY + SIZE_RF_ENCRYPTION_AES_KEY],
	                  (void*)EEPROM_SIGNING_SOFT_SERIAL_ADDRESS, SIZE_SIGNING_SOFT_SERIAL);
	hwReadConfigBlock((void*)&checksum, (void*)EEPROM_PERSONALIZATION_CHECKSUM_ADDRESS,
	                  SIZE_PERSONALIZATION_CHECKSUM);

	SHA256(hash, buffer, sizeof(buffer));
	hwWriteConfigBlock((void*)&hash[0], (void*)EEPROM_PERSONALIZATION_CHECKSUM_ADDRESS,
	                   SIZE_PERSONALIZATION_CHECKSUM);
}

/** @brief Store keys depending on configuration */
static void store_keys(void)
{
#if defined(STORE_HMAC_KEY) || defined(STORE_AES_KEY) || defined(STORE_SOFT_SERIAL)
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("|                                    Key storage                                     |"));
	Serial.println(
	    F("+--------+--------+------------------------------------------------------------------+"));
	Serial.println(
	    F("| Key ID | Status | Key                                                              |"));
	Serial.println(
	    F("+--------+--------+------------------------------------------------------------------+"));
#endif
#ifdef STORE_HMAC_KEY
	Serial.print(F("| HMAC   | "));
	if (!store_hmac_key_data(user_hmac_key)) {
		Serial.print(F("FAILED | "));
	} else {
		Serial.print(F("OK     | "));
	}
	print_hex_buffer(user_hmac_key, 32);
	Serial.println(F(" |"));
#endif
#ifdef STORE_AES_KEY
	Serial.print(F("| AES    | "));
	if (!store_aes_key_data(user_aes_key)) {
		Serial.print(F("FAILED | "));
	} else {
		Serial.print(F("OK     | "));
	}
	print_hex_buffer(user_aes_key, 16);
	Serial.println(F("                                 |"));
#endif
#ifdef STORE_SOFT_SERIAL
	Serial.print(F("| SERIAL | "));
	if (has_device_unique_id) {
		memset(user_soft_serial, 0xFF, 9);
	}
	if (!store_soft_serial_data(user_soft_serial)) {
		Serial.print(F("FAILED | "));
	} else {
		Serial.print(F("OK     | "));
	}
	if (has_device_unique_id) {
		Serial.println(F("EEPROM reset. MCU has a unique serial which will be used instead.|"));
	} else {
		print_hex_buffer(user_soft_serial, 9);
		Serial.println(F("                                               |"));
	}
#endif
#if defined(STORE_HMAC_KEY) || defined(STORE_AES_KEY) || defined(STORE_SOFT_SERIAL)
	Serial.println(
	    F("+--------+--------+------------------------------------------------------------------+"));
	Serial.println();
#endif
}

/**
 * @brief Print provided buffer as HEX data on serial console
 * @param data Buffer to print
 * @param sz Number of bytes to print
 */
static void print_hex_buffer(uint8_t* data, size_t sz)
{
	for (size_t i=0; i<sz; i++) {
		if (data[i] < 0x10) {
			Serial.print('0'); // Because Serial.print does not 0-pad HEX
		}
		Serial.print(data[i], HEX);
	}
}

/**
 * @brief Print provided buffer as C friendly copy+paste HEX data on serial console
 * @param data Buffer to print
 * @param sz Number of bytes to print
 */
static void print_c_friendly_hex_buffer(uint8_t* data, size_t sz)
{
	for (size_t i=0; i<sz; i++) {
		Serial.print("0x");
		if (data[i] < 0x10) {
			Serial.print('0'); // Because Serial.print does not 0-pad HEX
		}
		Serial.print(data[i], HEX);
		if (i < sz-1) {
			Serial.print(',');
		}
	}
}

#ifdef STORE_HMAC_KEY
/**
 * @brief Store the provided data as HMAC key to ATSHA204A or EEPROM depending on @ref USE_SOFT_SIGNING flag
 * @param data Key to store
 * @returns false if failed
 */
static bool store_hmac_key_data(uint8_t* data)
{
#ifdef USE_SOFT_SIGNING
	static uint8_t validation_buffer[32];
	hwWriteConfigBlock((void*)data, (void*)EEPROM_SIGNING_SOFT_HMAC_KEY_ADDRESS, 32);
	// Validate data written
	hwReadConfigBlock((void*)validation_buffer, (void*)EEPROM_SIGNING_SOFT_HMAC_KEY_ADDRESS, 32);
	if (memcmp(validation_buffer, data,	32) != 0) {
		return false;
	} else {
		return true;
	}
#else
	// It will not be possible to write the key if the configuration zone is unlocked
	if (lockConfig == 0x00) {
		// Write the key to the appropriate slot in the data zone
		if (!write_atsha204a_key(data)) {
			return false;
		} else {
			return true;
		}
	} else {
		return false;
	}
#endif
}
#endif

#ifdef STORE_AES_KEY
/**
 * @brief Store the provided data as AES key to EEPROM
 * @param data Key to store
 * @returns false if failed
 */
static bool store_aes_key_data(uint8_t* data)
{
	static uint8_t validation_buffer[16];
	hwWriteConfigBlock((void*)data, (void*)EEPROM_RF_ENCRYPTION_AES_KEY_ADDRESS, 16);
	// Validate data written
	hwReadConfigBlock((void*)validation_buffer, (void*)EEPROM_RF_ENCRYPTION_AES_KEY_ADDRESS, 16);
	if (memcmp(validation_buffer, data,	16) != 0) {
		return false;
	} else {
		return true;
	}
}
#endif // STORE_AES_KEY

#ifdef STORE_SOFT_SERIAL
/**
 * @brief Store the provided data as serial number to EEPROM
 * @param data Serial to store
 * @returns false if failed
 */
static bool store_soft_serial_data(uint8_t* data)
{
	static uint8_t validation_buffer[9];
	hwWriteConfigBlock((void*)data, (void*)EEPROM_SIGNING_SOFT_SERIAL_ADDRESS, 9);
	// Validate data written
	hwReadConfigBlock((void*)validation_buffer, (void*)EEPROM_SIGNING_SOFT_SERIAL_ADDRESS, 9);
	if (memcmp(validation_buffer, data,	9) != 0) {
		return false;
	} else {
		return true;
	}
}
#endif // STORE_SOFT_SERIAL

#ifndef USE_SOFT_SIGNING
/**
 * @brief Initialize and read states in ATSHA204A
 */
static void init_atsha204a_state(void)
{
	// Read out lock config bits to determine if locking is possible
	ret_code = sha204.sha204m_read(tx_buffer, rx_buffer, SHA204_ZONE_CONFIG, 0x15<<2);
	if (ret_code != SHA204_SUCCESS) {
		halt(false);
	} else {
		lockConfig = rx_buffer[SHA204_BUFFER_POS_DATA+3];
		lockValue = rx_buffer[SHA204_BUFFER_POS_DATA+2];
	}
}

#ifdef LOCK_ATSHA204A_CONFIGURATION
static void	lock_atsha204a_config(void)
{
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("|                           ATSHA204A configuration locking                          |"));
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	if (lockConfig != 0x00) {
		uint16_t crc;
		(void)crc;

		// Write config and get CRC for the updated config
		crc = write_atsha204a_config_and_get_crc();

		// List current configuration before attempting to lock
#ifdef PRINT_DETAILED_ATSHA204A_CONFIG
		Serial.println(
		    F("|                             New ATSHA204A Configuration                            |"));
		dump_detailed_atsha204a_configuration();
#endif // PRINT_DETAILED_ATSHA204A_CONFIG

#ifndef SKIP_UART_CONFIRMATION
		// Purge serial input buffer
		while (Serial.available()) {
			Serial.read();
		}
		Serial.println(
		    F("| * Send SPACE character now to lock the configuration...                            |"));

		while (Serial.available() == 0);
		if (Serial.read() == ' ')
#endif //not SKIP_UART_CONFIRMATION
		{
			Serial.print(F("| * Locking configuration..."));

			// Correct sequence, resync chip
			ret_code = sha204.sha204c_resync(SHA204_RSP_SIZE_MAX, rx_buffer);
			if (ret_code != SHA204_SUCCESS && ret_code != SHA204_RESYNC_WITH_WAKEUP) {
				Serial.println(
				    F("+------------------------------------------------------------------------------------+"));
				halt(false);
			}

			// Lock configuration zone
			ret_code = sha204.sha204m_execute(SHA204_LOCK, SHA204_ZONE_CONFIG,
			                                  crc, 0, NULL, 0, NULL, 0, NULL,
			                                  LOCK_COUNT, tx_buffer, LOCK_RSP_SIZE, rx_buffer);
			if (ret_code != SHA204_SUCCESS) {
				Serial.println(F("Failed                                                   |"));
				Serial.println(
				    F("+------------------------------------------------------------------------------------+"));
				halt(false);
			} else {
				Serial.println(F("Done                                                     |"));

				// Update lock flags after locking
				ret_code = sha204.sha204m_read(tx_buffer, rx_buffer, SHA204_ZONE_CONFIG, 0x15<<2);
				if (ret_code != SHA204_SUCCESS) {
					Serial.println(
					    F("+------------------------------------------------------------------------------------+"));
					halt(false);
				} else {
					lockConfig = rx_buffer[SHA204_BUFFER_POS_DATA+3];
					lockValue = rx_buffer[SHA204_BUFFER_POS_DATA+2];
				}
			}
		}
#ifndef SKIP_UART_CONFIRMATION
		else {
			Serial.println(
			    F("| * Unexpected answer. Skipping locking.                                             |"));
			Serial.println(
			    F("+------------------------------------------------------------------------------------+"));
		}
#endif //not SKIP_UART_CONFIRMATION
	} else {
		Serial.println(
		    F("| * Skipping configuration write and lock (configuration already locked).            |"));
		Serial.println(
		    F("+------------------------------------------------------------------------------------+"));
	}
	Serial.println();
}

/**
 * @brief Write default configuration and return CRC of the configuration bits
 * @returns CRC over the configuration bits
 */
static uint16_t write_atsha204a_config_and_get_crc(void)
{
	uint16_t crc = 0;
	uint8_t config_word[4];

	// We will set default settings from datasheet on all slots. This means that we can use slot 0 for the key
	// as that slot will not be readable (key will therefore be secure) and slot 8 for the payload digest
	// calculationon as that slot can be written in clear text even when the datazone is locked.
	// Other settings which are not relevant are kept as is.

	for (int i=0; i < 88; i += 4) {
		bool do_write = true;
		if (i == 20) {
			config_word[0] = 0x8F;
			config_word[1] = 0x80;
			config_word[2] = 0x80;
			config_word[3] = 0xA1;
		} else if (i == 24) {
			config_word[0] = 0x82;
			config_word[1] = 0xE0;
			config_word[2] = 0xA3;
			config_word[3] = 0x60;
		} else if (i == 28) {
			config_word[0] = 0x94;
			config_word[1] = 0x40;
			config_word[2] = 0xA0;
			config_word[3] = 0x85;
		} else if (i == 32) {
			config_word[0] = 0x86;
			config_word[1] = 0x40;
			config_word[2] = 0x87;
			config_word[3] = 0x07;
		} else if (i == 36) {
			config_word[0] = 0x0F;
			config_word[1] = 0x00;
			config_word[2] = 0x89;
			config_word[3] = 0xF2;
		} else if (i == 40) {
			config_word[0] = 0x8A;
			config_word[1] = 0x7A;
			config_word[2] = 0x0B;
			config_word[3] = 0x8B;
		} else if (i == 44) {
			config_word[0] = 0x0C;
			config_word[1] = 0x4C;
			config_word[2] = 0xDD;
			config_word[3] = 0x4D;
		} else if (i == 48) {
			config_word[0] = 0xC2;
			config_word[1] = 0x42;
			config_word[2] = 0xAF;
			config_word[3] = 0x8F;
		} else if (i == 52 || i == 56 || i == 60 || i == 64) {
			config_word[0] = 0xFF;
			config_word[1] = 0x00;
			config_word[2] = 0xFF;
			config_word[3] = 0x00;
		} else if (i == 68 || i == 72 || i == 76 || i == 80) {
			config_word[0] = 0xFF;
			config_word[1] = 0xFF;
			config_word[2] = 0xFF;
			config_word[3] = 0xFF;
		} else {
			// All other configs are untouched
			ret_code = sha204.sha204m_read(tx_buffer, rx_buffer, SHA204_ZONE_CONFIG, i);
			if (ret_code != SHA204_SUCCESS) {
				Serial.println(
				    F("+------------------------------------------------------------------------------------+"));
				halt(false);
			}
			// Set config_word to the read data
			config_word[0] = rx_buffer[SHA204_BUFFER_POS_DATA+0];
			config_word[1] = rx_buffer[SHA204_BUFFER_POS_DATA+1];
			config_word[2] = rx_buffer[SHA204_BUFFER_POS_DATA+2];
			config_word[3] = rx_buffer[SHA204_BUFFER_POS_DATA+3];
			do_write = false;
		}

		// Update crc with CRC for the current word
		crc = sha204.calculateAndUpdateCrc(4, config_word, crc);

		// Write config word
		if (do_write) {
			ret_code = sha204.sha204m_execute(SHA204_WRITE, SHA204_ZONE_CONFIG,
			                                  i >> 2, 4, config_word, 0, NULL, 0, NULL,
			                                  WRITE_COUNT_SHORT, tx_buffer, WRITE_RSP_SIZE, rx_buffer);
			if (ret_code != SHA204_SUCCESS) {
				Serial.println(
				    F("+------------------------------------------------------------------------------------+"));
				halt(false);
			}
		}
	}
	return crc;
}
#endif

static bool get_atsha204a_serial(uint8_t* data)
{
	ret_code = sha204.getSerialNumber(rx_buffer);
	if (ret_code != SHA204_SUCCESS) {
		return false;
	} else {
		memcpy(data, rx_buffer, 9);
		return true;
	}
}

#ifdef STORE_HMAC_KEY
/**
 * @brief Write provided key to slot 0
 * @param key The key data to write
 * @returns false if failed
 */
static bool write_atsha204a_key(uint8_t* key)
{
	// Write key to slot 0
	ret_code = sha204.sha204m_execute(SHA204_WRITE, SHA204_ZONE_DATA | SHA204_ZONE_COUNT_FLAG,
	                                  0, SHA204_ZONE_ACCESS_32, key, 0, NULL, 0, NULL,
	                                  WRITE_COUNT_LONG, tx_buffer, WRITE_RSP_SIZE, rx_buffer);
	if (ret_code != SHA204_SUCCESS) {
		return false;
	} else {
		return true;
	}
}
#endif // STORE_HMAC_KEY
#endif // not USE_SOFT_SIGNING

/** @brief Print a greeting on serial console */
static void print_greeting(void)
{
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("|                           MySensors security personalizer                          |"));
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println();
#ifdef NO_SETTINGS_DEFINED
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("| You are running without any configuration flags set.                               |"));
	Serial.println(
	    F("| No changes will be made to ATSHA204A or EEPROM except for the EEPROM checksum      |"));
	Serial.println(
	    F("| which will be updated.                                                             |"));
	Serial.println(
	    F("|                                                                                    |"));
	Serial.println(
	    F("| If you want to personalize your device, you have two options.                      |"));
	Serial.println(
	    F("|                                                                                    |"));
	Serial.println(
	    F("| 1. a. Enable either GENERATE_KEYS_ATSHA204A or GENERATE_KEYS_SOFT                  |"));
	Serial.println(
	    F("|       This will generate keys for ATSHA204A or software signing.                   |"));
	Serial.println(
	    F("|    b. Execute the sketch. You will be guided through the steps below under         |"));
	Serial.println(
	    F("|       WHAT TO DO NEXT?                                                             |"));
	Serial.println(
	    F("|    c. Copy the generated keys and replace the topmost definitions in this file.    |"));
	Serial.println(
	    F("|    d. Save the sketch and then disable the flag you just enabled.                  |"));
	Serial.println(
	    F("|    e. Enable PERSONALIZE_ATSHA204A to personalize the ATSHA204A device.            |"));
	Serial.println(
	    F("|       or                                                                           |"));
	Serial.println(
	    F("|       Enable PERSONALIZE_SOFT to personalize the EEPROM for software signing.      |"));
	Serial.println(
	    F("|       If you want to use whitelisting you need to pick a unique serial number      |"));
	Serial.println(
	    F("|       for each device you run the sketch on and fill in MY_SOFT_SERIAL.            |"));
	Serial.println(
	    F("|       or                                                                           |"));
	Serial.println(
	    F("|       Enable PERSONALIZE_SOFT_RANDOM_SERIAL to personalzie the EEPROM and          |"));
	Serial.println(
	    F("|       include a new random serial number every time the sketch is executed.        |"));
	Serial.println(
	    F("|       Take note of each saved serial number if you plan to use whitelisting.       |"));
	Serial.println(
	    F("|    f. Execute the sketch on each device you want to personalize that is supposed   |"));
	Serial.println(
	    F("|       to communicate securely.                                                     |"));
	Serial.println(
	    F("|                                                                                    |"));
	Serial.println(
	    F("| 2. Enable any configuration flag as you see fit.                                   |"));
	Serial.println(
	    F("|    It is assumed that you know what you are doing.                                 |"));
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println();
#else
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("|                               Configuration settings                               |"));
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
#if defined(GENERATE_KEYS_ATSHA204A)
	Serial.println(
	    F("| * Guided key generation for ATSHA204A using ATSHA024A                              |"));
#endif
#if defined(GENERATE_KEYS_SOFT)
	Serial.println(
	    F("| * Guided key generation for EEPROM using software                                  |"));
#endif
#if defined(PERSONALIZE_ATSHA204A)
	Serial.println(
	    F("| * Guided personalization/storage of keys in ATSHA204A                              |"));
#endif
#if defined(PERSONALIZE_SOFT)
	Serial.println(
	    F("| * Guided personalization/storage of keys in EEPROM                                 |"));
#endif
#if defined(PERSONALIZE_SOFT_RANDOM_SERIAL)
	Serial.println(
	    F("| * Guided storage and generation of random serial in EEPROM                         |"));
#endif
#if defined(USE_SOFT_SIGNING)
	Serial.println(
	    F("| * Software based personalization (no ATSHA204A usage whatsoever)                   |"));
#else
	Serial.println(
	    F("| * ATSHA204A based personalization                                                  |"));
#endif
#if defined(LOCK_ATSHA204A_CONFIGURATION)
	Serial.println(
	    F("| * Will lock ATSHA204A configuration                                                |"));
#endif
#if defined(SKIP_UART_CONFIRMATION)
	Serial.println(
	    F("| * Will not require any UART confirmations                                          |"));
#endif
#if defined(GENERATE_HMAC_KEY)
	Serial.print(
	    F("| * Will generate HMAC key using "));
#if defined(USE_SOFT_SIGNING)
	Serial.println(
	    F("software                                            |"));
#else
	Serial.println(
	    F("ATSHA204A                                           |"));
#endif
#endif
#if defined(STORE_HMAC_KEY)
	Serial.print(F("| * Will store HMAC key to "));
#if defined(USE_SOFT_SIGNING)
	Serial.println(
	    F("EEPROM                                                    |"));
#else
	Serial.println(
	    F("ATSHA204A                                                 |"));
#endif
#endif
#if defined(GENERATE_AES_KEY)
	Serial.print(
	    F("| * Will generate AES key using "));
#if defined(USE_SOFT_SIGNING)
	Serial.println(
	    F("software                                             |"));
#else
	Serial.println(
	    F("ATSHA204A                                            |"));
#endif
#endif
#if defined(STORE_AES_KEY)
	Serial.println(
	    F("| * Will store AES key to EEPROM                                                     |"));
#endif
#if defined(GENERATE_SOFT_SERIAL)
	Serial.println(
	    F("| * Will generate soft serial using software                                         |"));
#endif
#if defined(STORE_SOFT_SERIAL)
	Serial.println(
	    F("| * Will store soft serial to EEPROM                                                 |"));
#endif
#if defined(PRINT_DETAILED_ATSHA204A_CONFIG)
	Serial.println(
	    F("| * Will print detailed ATSHA204A configuration                                      |"));
#endif
#if defined(RESET_EEPROM_PERSONALIZATION)
	Serial.println(
	    F("| * Will reset EEPROM personalization data                                           |"));
#endif
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println();
#endif // not NO_SETTINGS_DEFINED
	probe_and_print_peripherals();
}

static void print_ending(void)
{
#if defined(GUIDED_MODE) || defined(NO_SETTINGS_DEFINED)
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("|                                  WHAT TO DO NEXT?                                  |"));
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
#ifdef NO_SETTINGS_DEFINED
	Serial.println(
	    F("| To proceed with the personalization, enable GENERATE_KEYS_ATSHA204A or             |"));
	Serial.println(
	    F("| GENERATE_KEYS_SOFT depending on what type of signing backend you plan to use.      |"));
	Serial.println(
	    F("| Both options will generate an AES key for encryption if you plan to use that.      |"));
	Serial.println(
	    F("| Recompile, upload and run the sketch again for further instructions.               |"));
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
#endif
#ifdef GENERATE_KEYS_ATSHA204A
	Serial.println(
	    F("| To proceed with the personalization, copy the keys shown in the Key copy section,  |"));
	Serial.println(
	    F("| and replace the corresponding definitions in the top of the sketch, then disable   |"));
	Serial.println(
	    F("| GENERATE_KEYS_ATSHA204A and enable PERSONALIZE_ATSHA204A.                          |"));
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
#endif
#ifdef GENERATE_KEYS_SOFT
	Serial.println(
	    F("| To proceed with the personalization, copy the keys shown in the Key copy section,  |"));
	Serial.println(
	    F("| and replace the corresponding definitions in the top of the sketch, then disable   |"));
	Serial.println(
	    F("| GENERATE_KEYS_SOFT and enable PERSONALIZE_SOFT or PERSONALIZE_SOFT_RANDOM_SERIAL.  |"));
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
#endif
#if defined(PERSONALIZE_ATSHA204A) ||\
	defined(PERSONALIZE_SOFT) ||\
	defined(PERSONALIZE_SOFT_RANDOM_SERIAL)
	Serial.println(
	    F("| This device has now been personalized. Run this sketch with its current settings   |"));
	Serial.println(
	    F("| on all the devices in your network that have security enabled.                     |"));
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
#endif
#endif // GUIDED_MODE or NO_SETTINGS_DEFINED
}

static void	probe_and_print_peripherals(void)
{
	unique_id_t uniqueID;

	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("|                           Hardware security peripherals                            |"));
	Serial.println(
	    F("+--------------+--------------+--------------+------------------------------+--------+"));
	Serial.println(
	    F("| Device       | Status       | Revision     | Serial number                | Locked |"));
	Serial.println(
	    F("+--------------+--------------+--------------+------------------------------+--------+"));
#if defined(ARDUINO_ARCH_AVR)
	Serial.print(F("| AVR          | DETECTED     | N/A          | "));
#elif defined(ARDUINO_ARCH_ESP8266)
	Serial.print(F("| ESP8266      | DETECTED     | N/A          | "));
#elif defined(ARDUINO_ARCH_SAMD)
	Serial.print(F("| SAMD         | DETECTED     | N/A          | "));
#elif defined(ARDUINO_ARCH_STM32F1)
	Serial.print(F("| STM32F1      | DETECTED     | N/A          | "));
#elif defined(ARDUINO_ARCH_STM32)
	Serial.print(F("| STM32        | DETECTED     | N/A          | "));
#elif defined(__linux__)
	Serial.print(F("| Linux        | DETECTED     | N/A          | "));
#else
	Serial.print(F("| Unknown      | DETECTED     | N/A          | "));
#endif
	if (hwUniqueID(&uniqueID)) {
		has_device_unique_id = true;
		print_hex_buffer(uniqueID, 9);
		Serial.println(F("           | N/A    |"));
	} else {
		Serial.println(F("N/A (generation required)    | N/A    |"));
	}
	Serial.println(
	    F("+--------------+--------------+--------------+------------------------------+--------+"));
#ifndef USE_SOFT_SIGNING
	Serial.print(F("| ATSHA204A    | "));
	ret_code = sha204.sha204c_wakeup(rx_buffer);
	if (ret_code != SHA204_SUCCESS) {
		ret_code = SHA204_SUCCESS; // Reset retcode to avoid false negative execution result
		Serial.println(F("NOT DETECTED | N/A          | N/A                          | N/A    |"));
	} else {
		uint8_t buffer[9];
		Serial.print(F("DETECTED     | "));
		ret_code = sha204.sha204m_dev_rev(tx_buffer, rx_buffer);
		if (ret_code != SHA204_SUCCESS) {
			Serial.print(F("FAILED       | "));
		} else {
			print_hex_buffer(&rx_buffer[SHA204_BUFFER_POS_DATA], 4);
			Serial.print(F("     | "));
		}
		if (!get_atsha204a_serial(buffer)) {
			memset(buffer, 0xFF, 9);
			Serial.print(F("FAILED                       | "));
		} else {
			print_hex_buffer(buffer, 9);
			Serial.print(F("           | "));
		}
		ret_code = sha204.sha204m_read(tx_buffer, rx_buffer, SHA204_ZONE_CONFIG, 0x15<<2);
		if (ret_code != SHA204_SUCCESS) {
			Serial.println("FAILED |");
		} else {
			if (rx_buffer[SHA204_BUFFER_POS_DATA+3] == 0x00) {
				Serial.println("YES    |");
			} else {
				Serial.println("NO     |");
			}
		}
	}
	Serial.println(
	    F("+--------------+--------------+--------------+------------------------------+--------+"));
#ifdef PRINT_DETAILED_ATSHA204A_CONFIG
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("|                          Current ATSHA204A Configuration                           |"));
	dump_detailed_atsha204a_configuration();
#endif
#endif // not USE_SOFT_SIGNING
	Serial.println();
}

static void print_eeprom_data(void)
{
	uint8_t buffer[32];

	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("|                                       EEPROM                                       |"));
	Serial.println(
	    F("+--------+--------+------------------------------------------------------------------+"));
	Serial.println(
	    F("| Key ID | Status | Key                                                              |"));
	Serial.println(
	    F("+--------+--------+------------------------------------------------------------------+"));
	Serial.print(F("| HMAC   | "));
	hwReadConfigBlock((void*)buffer, (void*)EEPROM_SIGNING_SOFT_HMAC_KEY_ADDRESS, 32);
	if (!memcmp(buffer, reset_buffer, 32)) {
		Serial.print(F("RESET  | "));
	} else {
		Serial.print(F("OK     | "));
	}
	print_hex_buffer(buffer, 32);
	Serial.println(F(" |"));
	Serial.print(F("| AES    | "));
	hwReadConfigBlock((void*)buffer, (void*)EEPROM_RF_ENCRYPTION_AES_KEY_ADDRESS, 16);
	if (!memcmp(buffer, reset_buffer, 16)) {
		Serial.print(F("RESET  | "));
	} else {
		Serial.print(F("OK     | "));
	}
	print_hex_buffer(buffer, 16);
	Serial.println(F("                                 |"));
	Serial.print(F("| SERIAL | "));
	hwReadConfigBlock((void*)buffer, (void*)EEPROM_SIGNING_SOFT_SERIAL_ADDRESS, 9);
	if (!memcmp(buffer, reset_buffer, 9)) {
		if (has_device_unique_id) {
			Serial.println(F("N/A    | Device unique serial, not stored in EEPROM                       |"));
		} else {
			Serial.print(F("RESET  | "));
			print_hex_buffer(buffer, 9);
			Serial.println(F("                                               |"));
		}
	} else {
		Serial.print(F("OK     | "));
		print_hex_buffer(buffer, 9);
		Serial.println(F("                                               |"));
	}
	Serial.println(
	    F("+--------+--------+------------------------------------------------------------------+"));
	Serial.println();
}

static void print_whitelisting_entry(void)
{
	uint8_t buffer[9];
#ifdef USE_SOFT_SIGNING
	unique_id_t uniqueID;
	hwReadConfigBlock((void*)buffer, (void*)EEPROM_SIGNING_SOFT_SERIAL_ADDRESS, 9);
	if (!memcmp(buffer, reset_buffer, 9)) {
		// Serial reset in EEPROM, check for unique ID
		if (hwUniqueID(&uniqueID)) {
			memcpy(buffer, uniqueID, 9);
		}
	}
#else
	// If ATSHA204A is used, use that serial here
	if (!get_atsha204a_serial(buffer)) {
		memset(buffer, 0xFF, 9);
	}
#endif
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.println(
	    F("|                      This nodes whitelist entry on other nodes                     |"));
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
	Serial.print(F("{.nodeId = <ID of this node>,.serial = {"));
	print_c_friendly_hex_buffer(buffer, 9);
	Serial.println(F("}}"));
	Serial.println(
	    F("+------------------------------------------------------------------------------------+"));
}

#ifdef PRINT_DETAILED_ATSHA204A_CONFIG
/** @brief Dump current configuration to UART */
static void dump_detailed_atsha204a_configuration(void)
{
	Serial.println(
	    F("+---------------------------------------------------------------++-------------------+"));
	Serial.println(
	    F("|                           Fieldname                           ||       Data        |"));
	Serial.println(
	    F("+-------------------------------+-------------------------------++---------+---------+"));
	for (int i=0; i < 88; i += 4) {
		ret_code = sha204.sha204m_read(tx_buffer, rx_buffer, SHA204_ZONE_CONFIG, i);
		if (ret_code != SHA204_SUCCESS) {
			Serial.println(
			    F("+------------------------------------------------------------------------------------+"));
			halt(false);
		}
		if (i == 0x00) {
			Serial.print(F("|            SN[0:1]            |            SN[2:3]            || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j == 1) {
					Serial.print(F(" | "));
				} else if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+-------------------------------+-------------------------------++---------+---------+"));
		} else if (i == 0x04) {
			Serial.print(F("|                            Revnum                             || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+---------------------------------------------------------------++-------------------+"));
		} else if (i == 0x08) {
			Serial.print(F("|                            SN[4:7]                            || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+---------------+---------------+---------------+---------------++----+----+----+----+"));
		} else if (i == 0x0C) {
			Serial.print(F("|     SN[8]     |  Reserved13   |   I2CEnable   |  Reserved15   || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j < 3) {
					Serial.print(F(" | "));
				} else {
					Serial.println(F(" |"));
				}
			}
			Serial.println(
			    F("+---------------+---------------+---------------+---------------++----+----+----+----+"));
		} else if (i == 0x10) {
			Serial.print(F("|  I2CAddress   |  TempOffset   |    OTPmode    | SelectorMode  || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j < 3) {
					Serial.print(F(" | "));
				} else {
					Serial.println(F(" |"));
				}
			}
			Serial.println(
			    F("+---------------+---------------+---------------+---------------++----+----+----+----+"));
		} else if (i == 0x14) {
			Serial.print(F("|          SlotConfig00         |         SlotConfig01          || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j == 1) {
					Serial.print(F(" | "));
				} else if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+-------------------------------+-------------------------------++---------+---------+"));
		} else if (i == 0x18) {
			Serial.print(F("|          SlotConfig02         |         SlotConfig03          || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j == 1) {
					Serial.print(F(" | "));
				} else if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+-------------------------------+-------------------------------++---------+---------+"));
		} else if (i == 0x1C) {
			Serial.print(F("|          SlotConfig04         |         SlotConfig05          || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j == 1) {
					Serial.print(F(" | "));
				} else if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+-------------------------------+-------------------------------++---------+---------+"));
		} else if (i == 0x20) {
			Serial.print(F("|          SlotConfig06         |         SlotConfig07          || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j == 1) {
					Serial.print(F(" | "));
				} else if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+-------------------------------+-------------------------------++---------+---------+"));
		} else if (i == 0x24) {
			Serial.print(F("|          SlotConfig08         |         SlotConfig09          || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j == 1) {
					Serial.print(F(" | "));
				} else if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+-------------------------------+-------------------------------++---------+---------+"));
		} else if (i == 0x28) {
			Serial.print(F("|          SlotConfig0A         |         SlotConfig0B          || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j == 1) {
					Serial.print(F(" | "));
				} else if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+-------------------------------+-------------------------------++---------+---------+"));
		} else if (i == 0x2C) {
			Serial.print(F("|          SlotConfig0C         |         SlotConfig0D          || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j == 1) {
					Serial.print(F(" | "));
				} else if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+-------------------------------+-------------------------------++---------+---------+"));
		} else if (i == 0x30) {
			Serial.print(F("|          SlotConfig0E         |         SlotConfig0F          || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j == 1) {
					Serial.print(F(" | "));
				} else if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+---------------+---------------+---------------+---------------++----+----+----+----+"));
		} else if (i == 0x34) {
			Serial.print(F("|   UseFlag00   | UpdateCount00 |   UseFlag01   | UpdateCount01 || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j < 3) {
					Serial.print(F(" | "));
				} else {
					Serial.println(F(" |"));
				}
			}
			Serial.println(
			    F("+---------------+---------------+---------------+---------------++----+----+----+----+"));
		} else if (i == 0x38) {
			Serial.print(F("|   UseFlag02   | UpdateCount02 |   UseFlag03   | UpdateCount03 || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j < 3) {
					Serial.print(F(" | "));
				} else {
					Serial.println(F(" |"));
				}
			}
			Serial.println(
			    F("+---------------+---------------+---------------+---------------++----+----+----+----+"));
		} else if (i == 0x3C) {
			Serial.print(F("|   UseFlag04   | UpdateCount04 |   UseFlag05   | UpdateCount05 || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j < 3) {
					Serial.print(F(" | "));
				} else {
					Serial.println(F(" |"));
				}
			}
			Serial.println(
			    F("+---------------+---------------+---------------+---------------++----+----+----+----+"));
		} else if (i == 0x40) {
			Serial.print(F("|   UseFlag06   | UpdateCount06 |   UseFlag07   | UpdateCount07 || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j < 3) {
					Serial.print(F(" | "));
				} else {
					Serial.println(F(" |"));
				}
			}
			Serial.println(
			    F("+---------------+---------------+---------------+---------------++----+----+----+----+"));
		} else if (i == 0x44) {
			Serial.print(F("|                        LastKeyUse[0:3]                        || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+---------------------------------------------------------------++-------------------+"));
		} else if (i == 0x48) {
			Serial.print(F("|                        LastKeyUse[4:7]                        || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+---------------------------------------------------------------++-------------------+"));
		} else if (i == 0x4C) {
			Serial.print(F("|                        LastKeyUse[8:B]                        || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+---------------------------------------------------------------++-------------------+"));
		} else if (i == 0x50) {
			Serial.print(F("|                        LastKeyUse[C:F]                        || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j < 3) {
					Serial.print(F("   "));
				}
			}
			Serial.println(F(" |"));
			Serial.println(
			    F("+---------------+---------------+---------------+---------------++----+----+----+----+"));
		} else if (i == 0x54) {
			Serial.print(F("|   UserExtra   |    Selector   |   LockValue   |  LockConfig   || "));
			for (int j=0; j<4; j++) {
				if (rx_buffer[SHA204_BUFFER_POS_DATA+j] < 0x10) {
					Serial.print('0'); // Because Serial.print does not 0-pad HEX
				}
				Serial.print(rx_buffer[SHA204_BUFFER_POS_DATA+j], HEX);
				if (j < 3) {
					Serial.print(F(" | "));
				} else {
					Serial.println(F(" |"));
				}
			}
			Serial.println(
			    F("+---------------+---------------+---------------+---------------++----+----+----+----+"));
		}
	}
}
#endif // PRINT_DETAILED_ATSHA204A_CONFIG

// Doxygen specific constructs, not included when built normally
// This is used to enable disabled macros/definitions to be included in the documentation as well.
#if DOXYGEN
#define GENERATE_KEYS_ATSHA204A
#define GENERATE_KEYS_SOFT
#define PERSONALIZE_ATSHA204A
#define PERSONALIZE_SOFT
#define PERSONALIZE_SOFT_RANDOM_SERIAL
#define USE_SOFT_SIGNING
#define LOCK_ATSHA204A_CONFIGURATION
#define SKIP_UART_CONFIRMATION
#define GENERATE_HMAC_KEY
#define STORE_HMAC_KEY
#define GENERATE_AES_KEY
#define STORE_AES_KEY
#define GENERATE_SOFT_SERIAL
#define STORE_SOFT_SERIAL
#define PRINT_DETAILED_ATSHA204A_CONFIG
#define RESET_EEPROM_PERSONALIZATION
#endif

/** @}*/