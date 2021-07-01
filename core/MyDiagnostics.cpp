/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in RAM or EEPROM which keeps track of the
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
 */


#include "MyDiagnostics.h"

static char inputBuffer[15];
static char inputCmd;
static uint8_t inputBufferPosition;
static String inputParameter;

void diagnosticsFlushSerial(void)
{
	delay(100);
	while (MY_SERIALDEVICE.available()) {
		(void)MY_SERIALDEVICE.read();
	}
}

void diagnosticsSerialInput(void)
{
	bool cmdReceived = false;
	inputBufferPosition = 0;
	while (!cmdReceived) {
		if (MY_SERIALDEVICE.available()) {
			const char inputChr = MY_SERIALDEVICE.read();
			if (inputChr == '\n' || inputBufferPosition == sizeof(inputBuffer) - 1) {
				cmdReceived = true;
			} else {
				inputBuffer[inputBufferPosition++] = inputChr;
			}
		}
	}
	diagnosticsFlushSerial();
	// null termination
	inputBuffer[inputBufferPosition] = 0;
	inputParameter = String(&inputBuffer[1]);
	inputCmd = toUpperCase(inputBuffer[0]);
}

void PRINT(const char *fmt, ...)
{
	char fmtBuffer[MY_SERIAL_OUTPUT_SIZE];
	va_list args;
	va_start(args, fmt);
	vsnprintf_P(fmtBuffer, sizeof(fmtBuffer), fmt, args);
	va_end(args);
	MY_SERIALDEVICE.print(fmtBuffer);
}

void PrintHex8(const uint8_t *data, const uint16_t length)
{
	for (uint16_t i = 0; i < length; i++) {
		PRINT(PSTR("%02" PRIX8 " "), data[i]);
		if ((((i + 1) % 16 == 0) && (i < length)) || (i == length - 1)) {
			MY_SERIALDEVICE.println();
		}
	}
}

void diagnosticsPrintSeparationLine(void)
{
	for (uint8_t i = 0; i < 50; i++) {
		MY_SERIALDEVICE.print("=");
	}
	MY_SERIALDEVICE.println();
}

void diagnosticsMySensorsEEPROMDump(void)
{
	uint8_t buffer[256];
	PRINT(PSTR("> MYS E2P START: 0x%04" PRIX16 "\n"), EEPROM_START);

	MY_SERIALDEVICE.print(F("> NODE_ID="));
	hwReadConfigBlock(buffer, EEPROM_NODE_ID_ADDRESS, SIZE_NODE_ID);
	PrintHex8(buffer, SIZE_NODE_ID);

	MY_SERIALDEVICE.print(F("> PAR_ID="));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_PARENT_NODE_ID_ADDRESS,
	                  SIZE_PARENT_NODE_ID);
	PrintHex8(buffer, SIZE_PARENT_NODE_ID);

	MY_SERIALDEVICE.print(F("> D_GW="));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_DISTANCE_ADDRESS, SIZE_DISTANCE);
	PrintHex8(buffer, SIZE_DISTANCE);

	MY_SERIALDEVICE.println(F("> RTE TABLE:"));
	hwReadConfigBlock(buffer, reinterpret_cast<void *> EEPROM_ROUTES_ADDRESS, SIZE_ROUTES);
	PrintHex8(buffer, SIZE_ROUTES);

	MY_SERIALDEVICE.println(F("> CTRL_CFG:"));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_CONTROLLER_CONFIG_ADDRESS,
	                  SIZE_CONTROLLER_CONFIG);
	PrintHex8(buffer, SIZE_CONTROLLER_CONFIG);

	MY_SERIALDEVICE.print(F("> PERS_CRC="));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_PERSONALIZATION_CHECKSUM_ADDRESS,
	                  SIZE_PERSONALIZATION_CHECKSUM);
	PrintHex8(buffer, SIZE_PERSONALIZATION_CHECKSUM);

	MY_SERIALDEVICE.print(F("> FW_TYPE="));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_FIRMWARE_TYPE_ADDRESS,
	                  SIZE_PERSONALIZATION_CHECKSUM);
	PrintHex8(buffer, SIZE_PERSONALIZATION_CHECKSUM);

	MY_SERIALDEVICE.print(F("> FW_VERS="));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_FIRMWARE_VERSION_ADDRESS,
	                  SIZE_FIRMWARE_VERSION);
	PrintHex8(buffer, SIZE_FIRMWARE_VERSION);

	MY_SERIALDEVICE.print(F("> FW_BLOCKS="));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_FIRMWARE_BLOCKS_ADDRESS,
	                  SIZE_FIRMWARE_BLOCKS);
	PrintHex8(buffer, SIZE_FIRMWARE_BLOCKS);

	MY_SERIALDEVICE.print(F("> FW_CRC="));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_FIRMWARE_CRC_ADDRESS, SIZE_FIRMWARE_CRC);
	PrintHex8(buffer, SIZE_FIRMWARE_CRC);

	MY_SERIALDEVICE.println(F("> SGN_REQ_TABLE:"));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_SIGNING_REQUIREMENT_TABLE_ADDRESS,
	                  SIZE_SIGNING_REQUIREMENT_TABLE);
	PrintHex8(buffer, SIZE_SIGNING_REQUIREMENT_TABLE);

	MY_SERIALDEVICE.println(F("> WL_REQ_TABLE:"));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_WHITELIST_REQUIREMENT_TABLE_ADDRESS,
	                  SIZE_WHITELIST_REQUIREMENT_TABLE);
	PrintHex8(buffer, SIZE_WHITELIST_REQUIREMENT_TABLE);

	MY_SERIALDEVICE.println(F("> SGN_SOFT_KEY:"));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_SIGNING_SOFT_HMAC_KEY_ADDRESS,
	                  SIZE_SIGNING_SOFT_HMAC_KEY);
	PrintHex8(buffer, SIZE_SIGNING_SOFT_HMAC_KEY);

	MY_SERIALDEVICE.println(F("> SGN_SOFT_SER:"));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_SIGNING_SOFT_SERIAL_ADDRESS,
	                  SIZE_SIGNING_SOFT_SERIAL);
	PrintHex8(buffer, SIZE_SIGNING_SOFT_SERIAL);

	MY_SERIALDEVICE.println(F("> AES_KEY:"));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_RF_ENCRYPTION_AES_KEY_ADDRESS,
	                  SIZE_RF_ENCRYPTION_AES_KEY);
	PrintHex8(buffer, SIZE_RF_ENCRYPTION_AES_KEY);

	MY_SERIALDEVICE.print(F("> NL_CNT="));
	hwReadConfigBlock(buffer, reinterpret_cast<void *>EEPROM_NODE_LOCK_COUNTER_ADDRESS,
	                  SIZE_NODE_LOCK_COUNTER);
	PrintHex8(buffer, SIZE_NODE_LOCK_COUNTER);

	PRINT(PSTR("> USER E2P >= 0x%04" PRIX16 "\n"), EEPROM_LOCAL_CONFIG_ADDRESS);
}

void diagnosticsClearMySensorsEEPROMConfig(void)
{
	for (uint16_t i = EEPROM_START; i < EEPROM_START + EEPROM_LOCAL_CONFIG_ADDRESS; i++) {
		hwWriteConfig(i, 0xFF);
		if (hwReadConfig(i) != 0xFF) {
			PRINT(PSTR("!ERR POS 0x02%" PRIX8 "\n"), i);
		}
	}
	MY_SERIALDEVICE.println(F("> E2P CLR"));
}

void diagnosticsClearMySensorsRoutingTable(void)
{
	for (uint16_t i = 0; i < SIZE_ROUTES; i++) {
		hwWriteConfig(EEPROM_ROUTES_ADDRESS + i, 0xFF);
		if (hwReadConfig(EEPROM_ROUTES_ADDRESS + i) != 0xFF) {
			PRINT(PSTR("!ERR POS 0x02%" PRIu8 "\n"), i);
		}
	}
	MY_SERIALDEVICE.println(F("> RTE TABLE CLR"));
}

void diagnosticsClearMySensorsTransportSettings(void)
{
	hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 0xFF);
	hwWriteConfig(EEPROM_PARENT_NODE_ID_ADDRESS, 0xFF);
	hwWriteConfig(EEPROM_DISTANCE_ADDRESS, 0xFF);
	MY_SERIALDEVICE.println(F("> TSP CFG CLR"));
}

void diagnosticsEEPROMTest(void)
{
	MY_SERIALDEVICE.println(F("EEPROM test:"));
	uint16_t success = 0;
	for (uint16_t i = 0; i < (EEPROM_LOCAL_CONFIG_ADDRESS - EEPROM_START); i++) {
		if (i % 80 == 0) {
			MY_SERIALDEVICE.println();
			MY_SERIALDEVICE.print(i + EEPROM_START, HEX);
			MY_SERIALDEVICE.print(": ");
		} else {
			MY_SERIALDEVICE.print(".");
		}
		const uint8_t originalContent = hwReadConfig(i + EEPROM_START);
		hwWriteConfig(i + EEPROM_START, 0xAA);
		if (hwReadConfig(i + EEPROM_START) == 0xAA) {
			success++;
		} else {
			PRINT(PSTR("!ERR POS 0x02%" PRIu8 "\n"), i + EEPROM_START);
		}
		hwWriteConfig(i + EEPROM_START, 0x55);
		if (hwReadConfig(i + EEPROM_START) == 0x55) {
			success++;
		}
		// write back original byte
		hwWriteConfig(i + EEPROM_START, originalContent);
		if (hwReadConfig(i + EEPROM_START) == originalContent) {
			success++;
		} else {
			PRINT(PSTR("!ERR POS 0x02%" PRIu8 "\n"), i + EEPROM_START);
		}
	}
	MY_SERIALDEVICE.print(F("\n>E2P check: "));
	if (success == (EEPROM_LOCAL_CONFIG_ADDRESS - EEPROM_START) * 3) {
		MY_SERIALDEVICE.println(F("pass"));
	} else {
		MY_SERIALDEVICE.println(F("failed!"));
	}

}

void diagnosticsEEPROMMenu(void)
{
	while (true) {
		diagnosticsPrintSeparationLine();
		MY_SERIALDEVICE.println(F("EEPROM:\n\n"
		                          "[D] Dump\n"
		                          "[T] Test\n"
		                          "[C] CLR\n"
		                          "[R] CLR TSP RTE\n"
		                          "[S] CLR TSP CFG\n"
		                          "[X] Exit"
		                         ));
		diagnosticsPrintSeparationLine();
		diagnosticsFlushSerial();
		diagnosticsSerialInput();
		if (inputCmd == 'D') {
			diagnosticsMySensorsEEPROMDump();
		} else if (inputCmd == 'T') {
			diagnosticsEEPROMTest();
		} else if (inputCmd == 'C') {
			diagnosticsClearMySensorsEEPROMConfig();
		} else if (inputCmd== 'R') {
			diagnosticsClearMySensorsRoutingTable();
		} else if (inputCmd == 'S') {
			diagnosticsClearMySensorsTransportSettings();
		} else if (inputCmd == 'X') {
			return;
		}
	}
}

#if defined(MY_DIAGNOSTICS_CRYPTO)
bool diagnosticsCryptoMenu(void)
{
	MY_SERIALDEVICE.println(F("Testing:"));
	const uint8_t test_data[64] = { 0x76, 0x49, 0xab, 0xac, 0x81, 0x19, 0xb2, 0x46, 0xce, 0xe9, 0x8e, 0x9b, 0x12, 0xe9, 0x19, 0x7d,
	                                0x50, 0x86, 0xcb, 0x9b, 0x50, 0x72, 0x19, 0xee, 0x95, 0xdb, 0x11, 0x3a, 0x91, 0x76, 0x78, 0xb2,
	                                0x73, 0xbe, 0xd6, 0xb8, 0xe3, 0xc1, 0x74, 0x3b, 0x71, 0x16, 0xe6, 0x9e, 0x22, 0x22, 0x95, 0x16,
	                                0x3f, 0xf1, 0xca, 0xa1, 0x68, 0x1f, 0xac, 0x09, 0x12, 0x0e, 0xca, 0x30, 0x75, 0x86, 0xe1, 0xa7
	                              };
	const uint8_t test_psk[16] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
	uint8_t aes_iv[16];
	for (uint8_t i = 0; i < sizeof(aes_iv); i++) {
		aes_iv[i] = i;
	}

	AES128CBCInit(test_psk);
#if defined(CRYPTO_OUTPUT)
	MY_SERIALDEVICE.println(F("AES128CBC input:"));
	PrintHex8(test_data, sizeof(test_data));
	MY_SERIALDEVICE.println(F("AES128CBC key:"));
	PrintHex8(test_psk, sizeof(test_psk));
	MY_SERIALDEVICE.println(F("AES128CBC IV:"));
	PrintHex8(aes_iv, sizeof(aes_iv));
#endif
	uint8_t temp_iv[16];
	uint8_t temp_data[64];
	(void)memcpy((void *)temp_iv, (const void *)aes_iv, sizeof(aes_iv));
	(void)memcpy((void *)temp_data, (const void *)test_data, sizeof(temp_data));
	AES128CBCEncrypt(temp_iv, temp_data, sizeof(test_data));
	MY_SERIALDEVICE.print(F("- AES128 CBC encryption: "));
	const uint8_t aes_ciphertext[64] = {
		0x46,0xE3,0x35,0xB8,0xEA,0x11,0xBC,0xC5,0xB4,0xEB,0x7F,0x49,0xD1,0x14,0xFF,0x43,0x28,0x22,0x15,
		0xAD,0x3A,0xCF,0xF1,0x6B,0xE1,0x9B,0x6F,0x71,0x1A,0xA1,0x3B,0x89,0x69,0xFD,0x9F,0xB7,0x98,0x2A,
		0x37,0x03,0xE8,0x16,0x14,0x3F,0x89,0x62,0x56,0x0F,0xDA,0x85,0xAD,0x94,0xD3,0x4E,0x54,0x18,0x2A,
		0x52,0x5C,0x2B,0x28,0xFA,0x0E,0xAB
	};
	// result verified here: http://extranet.cryptomathic.com/aescalc/index
	if (memcmp(temp_data, &aes_ciphertext, sizeof(aes_ciphertext)) == 0) {
		MY_SERIALDEVICE.println(F("OK"));
	} else {
		MY_SERIALDEVICE.println(F("FAIL!"));
	};
#if defined(CRYPTO_OUTPUT)
	PrintHex8(temp_data, sizeof(test_data));
#endif
	(void)memcpy(temp_iv, aes_iv, sizeof(aes_iv));
	AES128CBCDecrypt(temp_iv, temp_data, sizeof(temp_data));
	MY_SERIALDEVICE.print(F("- AES128 CBC decryption: "));
	if (memcmp((const void *)test_data, (const void *)temp_data, sizeof(temp_data)) == 0) {
		MY_SERIALDEVICE.println(F("OK"));
	} else {
		MY_SERIALDEVICE.println(F("FAIL!"));
	};
#if defined(CRYPTO_OUTPUT)
	PrintHex8(temp_data, sizeof(temp_data));
#endif
	MY_SERIALDEVICE.print(F("- SHA256: "));
	uint8_t dest[64];
	SHA256((uint8_t *)dest, (const uint8_t *)test_data, sizeof(test_data));
#if defined(CRYPTO_OUTPUT)
	MY_SERIALDEVICE.println(F("SHA256 input:"));
	PrintHex8(test_data, sizeof(test_data));
	MY_SERIALDEVICE.println(F("SHA256 output:"));
	PrintHex8(dest, 32);
#endif
	// result verified here: http://extranet.cryptomathic.com/hashcalc/index
	const uint8_t sha256result[32] = { 0x51,0x3f,0xa7,0x82,0x3d,0xc3,0x05,0x3d,0xc6,0x43,0xa4,0x4b,0x8f,0xb8,0xdd,0x62,
	                                   0x36,0x0b,0x00,0x44,0xf1,0xab,0x69,0x65,0xf8,0x36,0x29,0xd2,0xb1,0x64,0xbf,0x14
	                                 };
	//PRINT(PSTR("SHA256 test: "));
	if (memcmp(dest, &sha256result, sizeof(sha256result)) == 0) {
		MY_SERIALDEVICE.println(F("OK"));
	} else {
		MY_SERIALDEVICE.println(F("FAIL!"));
	};

	MY_SERIALDEVICE.print(F("- HMAC SHA256: "));
#if defined(CRYPTO_OUTPUT)
	MY_SERIALDEVICE.println(F("HMAC input:"));
	PrintHex8(test_data, sizeof(test_data));
	MY_SERIALDEVICE.println(F("HMAC key:"));
	PrintHex8(test_psk, sizeof(test_psk));
#endif
	SHA256HMAC(dest, test_psk, sizeof(test_psk), test_data, sizeof(test_data));
#if defined(CRYPTO_OUTPUT)
	MY_SERIALDEVICE.println(F("HMAC output:"));
	PrintHex8(dest, sizeof(dest));
#endif
	// result verified here: http://extranet.cryptomathic.com/hmaccalc/index
	const uint8_t hmacresult[32] = { 0xcc,0xa7,0x5f,0x5d,0xd5,0xeb,0x50,0x34,0x02,0x53,0x12,0x17,0x40,0x72,0xaf,0x29,
	                                 0xe6,0xc9,0xb5,0xb1,0x9b,0x26,0x8b,0x23,0x0f,0x5c,0xeb,0x50,0x24,0x63,0xc2,0x33
	                               };
	//PRINT(PSTR("HMAC test: "));
	if (memcmp(dest, &hmacresult, sizeof(sha256result)) == 0) {
		MY_SERIALDEVICE.println(F("OK"));
	} else {
		MY_SERIALDEVICE.println(F("FAIL!"));
	};
	MY_SERIALDEVICE.println(F("> MUL speed:"));
	uint32_t startMS, stopMS;
	uint32_t cnt;
	MY_SERIALDEVICE.print(F("- 8bit MUL: "));
	startMS = hwMillis();
	uint8_t u8 = 1;
	cnt = 0xFFFFF;
	while (cnt--) {
		u8 *= 3;
	}
	stopMS = hwMillis();
	if (u8 == 171) {
		PRINT(PSTR("OK, %" PRIu32 " ms\n"), stopMS - startMS);
	} else {
		MY_SERIALDEVICE.println(F("FAIL!"));
	}

	MY_SERIALDEVICE.print(F("- 16bit MUL: "));
	startMS = hwMillis();
	uint16_t u16 = 1;
	cnt = 0xFFFFF;
	while (cnt--) {
		u16 *= 3;
	}
	stopMS = hwMillis();
	if (u16 == 43691) {
		PRINT(PSTR("OK, %" PRIu32 " ms\n"), stopMS - startMS);
	} else {
		MY_SERIALDEVICE.println(F("FAIL!"));
	}

	MY_SERIALDEVICE.print(F("- 32bit MUL: "));
	startMS = hwMillis();
	uint32_t u32 = 1;
	cnt = 0xFFFFF;
	while (cnt--) {
		u32 *= 3;
	}
	stopMS = hwMillis();
	if (u32 == 3664423595) {
		PRINT(PSTR("OK, %" PRIu32 " ms\n"), stopMS - startMS);
	} else {
		MY_SERIALDEVICE.println(F("FAIL!"));
	}
	return false;
}
#endif

#if defined(ARDUINO_ARCH_AVR)
void diagnosticsWatchdogTest(void)
{
	MY_SERIALDEVICE.println(F("Set WDT to 4s\n"));
	hwWatchdogReset();
	wdt_enable(WDTO_4S);
	for (uint8_t timer = 0; timer < 10; timer++) {
		MY_SERIALDEVICE.print(timer);
		delay(1000);
	}
	MY_SERIALDEVICE.println(F("WDT failed!\n"));
}
#endif

#if defined(MY_RADIO_RFM95)
void diagnosticsRFM95Menu(void)
{

}
#endif

#if defined(MY_RADIO_RFM69) && defined(MY_RFM69_NEW_DRIVER)
void diagnosticsRFM69Menu(void)
{
	RFM69_initialise(RFM69_868MHZ);
	while (true) {
		diagnosticsFlushSerial();
		diagnosticsPrintSeparationLine();
		MY_SERIALDEVICE.println(F("RFM69:\n"));
		PRINT(PSTR("SPI: MOSI=%" PRIu8 ", MISO=%" PRIu8 ", SCK=%" PRIu8 ", CS=%" PRIu8 ", IRQ=%" PRIu8
		           "\n"),
		      MOSI, MISO, SCK, MY_RFM69_CS_PIN, MY_RFM69_IRQ_PIN);
		PRINT(PSTR("RF: ID=%" PRIu8 ", FREQ=%" PRIu32 ", POW=%" PRIu8 "\n"),
		      RFM69_getAddress(), RFM69_getFrequency(), RFM69_getTxPowerLevel());

		MY_SERIALDEVICE.println(F(
		                            "[I] Init\n"
		                            "[D] Dump REG\n"
		                            "[Ax] ADDR=x\n"
		                            "[Fx] FREQ=x\n"
		                            "[Wx] POW=X\n"
		                            "[L] SLP\n"
		                            "[B] STDBY\n"
		                            "[O] CAR on\n"
		                            "[Q] CAR off\n"
		                            "[R] RX\n"
		                            "[Tx] TX to x\n"
		                            "[P] Poll STAT\n"
		                            "[X] Exit"
		                        ));
		diagnosticsPrintSeparationLine();
		diagnosticsSerialInput();
		if (inputCmd == 'I') {
			RFM69_initialise(RFM69_868MHZ);
		} else if (inputCmd == 'A') {
			RFM69_setAddress(inputParameter.toInt());
		} else if (inputCmd == 'F') {
			RFM69_setFrequency(inputParameter.toInt());
		} else if (inputCmd == 'W') {
			RFM69_setTxPowerLevel(inputParameter.toInt());
		} else if (inputCmd == 'L') {
			RFM69_sleep();
		} else if (inputCmd == 'B') {
			RFM69_standBy();
		} else if (inputCmd == 'R') {
			(void)RFM69_setRadioMode(RFM69_RADIO_MODE_RX);
		} else if (inputCmd == 'T') {
			uint8_t buffer[] = { 'T','E','S','T','R','F','M','6','9' };
			RFM69_sendWithRetry(inputParameter.toInt(), buffer, sizeof(buffer), true);
		} else if (inputCmd == 'P') {
			diagnosticsPrintSeparationLine();
			MY_SERIALDEVICE.println(F("Press any key to exit"));
			diagnosticsPrintSeparationLine();
			diagnosticsFlushSerial();
			while (!MY_SERIALDEVICE.available()) {
				PRINT(PSTR("IRQF1=0x%02" PRIX8 ", IRQF2=0x%02" PRIX8 ", IRQF=%" PRIu8 "\n"),
				      RFM69_readReg(RFM69_REG_IRQFLAGS1), RFM69_readReg(RFM69_REG_IRQFLAGS2), RFM69_irq);
				delay(300);
			}
			MY_SERIALDEVICE.println(F("Exiting..."));
		} else if (inputCmd == 'O') {
			(void)RFM69_setRadioMode(RFM69_RADIO_MODE_TX);
		} else if (inputCmd == 'Q') {
			(void)RFM69_setRadioMode(RFM69_RADIO_MODE_STDBY);
		} else if (inputCmd == 'D') {
			uint8_t i = 0;
			do {
				PRINT(PSTR("Reg 0x%02" PRIX8 " = 0x%02" PRIX8 "\n"), i, RFM69_readReg(i));
			} while (i++ != 0xFF);
		} else if (inputCmd == 'X') {
			return;
		}
	}
}
#endif

#if defined(MY_RADIO_RF24)
void diagnosticsRF24Menu(void)
{
	RF24_initialize();
	while (true) {
		diagnosticsFlushSerial();
		diagnosticsPrintSeparationLine();
		MY_SERIALDEVICE.println(F("RF24:\n"));
		PRINT(PSTR("SPI: MOSI=%" PRIu8 ", MISO=%" PRIu8 ", SCK=%" PRIu8 ", CS=%" PRIu8 ", CE=%" PRIu8 "\n"),
		      MOSI, MISO, SCK, MY_RF24_CS_PIN, MY_RF24_CE_PIN);
		PRINT(PSTR("RF: ADDR=%" PRIu8 ", CH=%" PRIu8 ", POW=%" PRIu8 ", CFG=%" PRIu8 "\n"),
		      RF24_getNodeID(),
		      RF24_getChannel(), RF24_getRawTxPowerLevel(), RF24_getRFConfiguration());

		MY_SERIALDEVICE.println(F("[I] Init\n"
		                          "[D] Dump REG\n"
		                          "[Ax] ADDR=x\n"
		                          "[Cx] CH=x\n"
		                          "[Wx] POW=X\n"
		                          "[L] SLP\n"
		                          "[B] STDBY\n"
		                          "[O] CAR on\n"
		                          "[Q] CAR off\n"
		                          "[R] RX\n"
		                          "[Tx] TX to x\n"
		                          "[P] Poll STAT\n"
		                          "[S] Scan CHs\n"
		                          "[X] Exit"
		                         ));
		diagnosticsPrintSeparationLine();
		diagnosticsSerialInput();
		if (inputCmd == 'I') {
			RF24_initialize();
		} else if (inputCmd == 'A') {
			RF24_setNodeAddress(inputParameter.toInt());
		} else if (inputCmd == 'C') {
			RF24_setChannel(inputParameter.toInt());
		} else if (inputCmd == 'W') {
			RF24_setTxPowerLevel(inputParameter.toInt());
		} else if (inputCmd == 'L') {
			RF24_sleep();
		} else if (inputCmd == 'B') {
			RF24_standBy();
		} else if (inputCmd == 'R') {
			RF24_startListening();
		} else if (inputCmd == 'T') {
			uint8_t buffer[] = { 'T','E','S','T','R','F','2','4' };
			RF24_sendMessage(inputParameter.toInt(), buffer, sizeof(buffer), false);
		} else if (inputCmd == 'P') {
			diagnosticsPrintSeparationLine();
			MY_SERIALDEVICE.println(F("Press any key to exit"));
			diagnosticsPrintSeparationLine();
			diagnosticsFlushSerial();
			while (!MY_SERIALDEVICE.available()) {
				PRINT(PSTR("status=%02" PRIX8 "\n"), RF24_getStatus());
				delay(300);
			}
			MY_SERIALDEVICE.println(F("Exiting..."));
		} else if (inputCmd == 'O') {
			RF24_enableConstantCarrierWave();
		} else if (inputCmd == 'Q') {
			RF24_disableConstantCarrierWave();
		} else if (inputCmd == 'D') {
			//uint8_t buffer[16];
			for (uint8_t i = 0; i < 0x20; i++) {
				PRINT(PSTR("Reg 0x%02" PRIX8 " = 0x%02" PRIX8 "\n"), i, RF24_readByteRegister(i));
				/*
				(void)RF24_readMultiByteRegister(i, buffer, sizeof(buffer));
				PrintHex8(buffer, sizeof(buffer));
				for (uint8_t cnt = 0; cnt < sizeof(buffer); cnt++) {
					buffer[cnt] = 0xFF;
				}
				(void)RF24_writeMultiByteRegister(i, buffer, sizeof(buffer));
				(void)RF24_readMultiByteRegister(i, buffer, sizeof(buffer));
				PrintHex8(buffer, sizeof(buffer));

				for (uint8_t cnt = 0; cnt < sizeof(buffer); cnt++) {
					buffer[cnt] = 0x00;
				}
				(void)RF24_writeMultiByteRegister(i, buffer, sizeof(buffer));
				(void)RF24_readMultiByteRegister(i, buffer, sizeof(buffer));
				PrintHex8(buffer, sizeof(buffer));
				*/
			}
		} else if (inputCmd == 'S') {

			MY_SERIALDEVICE.println(F("Press any key to exit"));
			diagnosticsFlushSerial();

			const uint8_t num_channels = 126;

			for(uint8_t i = 0; i < num_channels; i++) {
				PRINT(PSTR("%" PRIX8), i >> 4);
			}
			MY_SERIALDEVICE.println();

			for (uint8_t i = 0; i < num_channels; i++) {
				PRINT(PSTR("%" PRIX8), i & 0xf);
			}

			MY_SERIALDEVICE.println();


			while (!MY_SERIALDEVICE.available()) {

				uint8_t values[num_channels];
				// disable ACK on all pipes
				RF24_setAutoACK(0);
				// clear result array
				(void)memset((void *)values, 0, sizeof(values));
				for (uint8_t rep_counter = 0; rep_counter < 100; rep_counter++) {
					for (uint8_t channel = 0; channel < num_channels; channel++) {
						RF24_setChannel(channel);
						RF24_startListening();
						delayMicroseconds(130 + 40);
						// Carrier detected?
						if (RF24_getReceivedPowerDetector()) {
							values[channel] = values[channel] + 1;
						}
						RF24_stopListening();
					}
				}

				for (uint8_t i = 0; i < num_channels; i++) {
					PRINT(PSTR("%" PRIX8), min(0xf, values[i]));
				}

				MY_SERIALDEVICE.println();
			}
		} else if (inputCmd == 'X') {
			return;
		}
	}

}
#endif

#if defined(MY_SENSOR_NETWORK)
void diagnosticsTSMStatus(void)
{
	PRINT(PSTR("%" PRIu32 " TSM,%" PRIu8 ",%" PRIu8 ",%" PRIu8 ",%" PRIu32 ",%" PRIu32 ",%" PRIu8 ",%"
	           PRIu8 ",%" PRIu8 ",%" PRIu8 ",%" PRIu8 ",%" PRIu8 ",%" PRIu8 ",%" PRIu8 "\n"),
	      hwMillis(),
	      transportSanityCheck(),
	      getNodeId(),
	      getParentNodeId(),
	      _transportSM.stateEnter,
	      _transportSM.lastUplinkCheck,
	      _transportSM.findingParentNode,
	      _transportSM.uplinkOk,
	      _transportSM.pingActive,
	      _transportSM.transportActive,
	      _transportSM.stateRetries,
	      _transportSM.failedUplinkTransmissions,
	      _transportSM.failureCounter,
	      _transportSM.pingResponse);
}
#endif

void diagnosticsTransportSM(void)
{
	while (true) {
		diagnosticsPrintSeparationLine();
		MY_SERIALDEVICE.println(F("TSP SM:\n"));
#if defined(MY_SENSOR_NETWORK)
		PRINT(PSTR("ADDR=%" PRIu8 ",PAR=%" PRIu8 ",DGW=%" PRIu8 ",TSP=%" PRIu8 "\n"), getNodeId(),
		      getDistanceGW(), getParentNodeId(),
		      isTransportReady());
		MY_SERIALDEVICE.println(F("[I] Init TSP\n"
		                          "[S] Step TSM\n"
		                          "[R] Run TSM"
		                         ));
#endif
		MY_SERIALDEVICE.println(F("[X] Exit"));
		diagnosticsPrintSeparationLine();
		diagnosticsFlushSerial();
		diagnosticsSerialInput();
		if (inputCmd == 'I') {
#if defined(MY_SENSOR_NETWORK)
			transportInitialise();
#endif
		} else if (inputCmd == 'S') {
#if defined(MY_SENSOR_NETWORK)
			transportProcess();
			diagnosticsTSMStatus();
#endif
		} else if (inputCmd == 'R') {
#if defined(MY_SENSOR_NETWORK)
			MY_SERIALDEVICE.println(F("[U] CKU\n"
			                          "[F] FPAR\n"
			                          "[E] TSP ERR\n"
			                          "[I] INIT\n"
			                          "[Cx] PNG x\n"
			                          "[Nx] ID=x\n"
			                          "[Px] PAR=x\n"
			                          "[Tx] TX x\n"
			                          "[Sx] Sleep x ms\n"
			                          "[X] EXIT\n"
			                         ));
			uint32_t lastTimer = 0;
			bool exitSignal = false;
			while (!exitSignal) {
				if (MY_SERIALDEVICE.available()) {
					diagnosticsSerialInput();
					if (inputCmd == 'U') {
						transportCheckUplink();
					} else if (inputCmd == 'F') {
						transportSwitchSM(stParent);
					} else if (inputCmd == 'E') {
						transportSwitchSM(stFailure);
					} else if (inputCmd == 'I') {
						transportInitialise();
					} else if (inputCmd == 'C') {
						_transportSM.pingActive = false;
						transportPingNode(inputParameter.toInt());
					} else if (inputCmd == 'N') {
						const uint8_t nodeID = inputParameter.toInt();
						_transportConfig.nodeId = nodeID;
						transportHALSetAddress(nodeID);
						// Write ID to EEPROM
						hwWriteConfig(EEPROM_NODE_ID_ADDRESS, nodeID);
					} else if (inputCmd == 'P') {
						_transportConfig.parentNodeId = inputParameter.toInt();
					} else if (inputCmd == 'T') {
						transportSendRoute(build(_msgTmp, inputParameter.toInt(), NODE_SENSOR_ID, C_SET, V_VAR1,
						                         false).set((uint32_t)0xDEADBEAF));
					} else if (inputCmd == 'S') {
						(void)sleep((uint32_t)inputParameter.toInt(), false);
					} else if (inputCmd == 'X') {
						exitSignal = true;
					}

				}
				transportProcess();
				if (hwMillis() - lastTimer > 1000ul) {
					lastTimer = hwMillis();
					diagnosticsTSMStatus();
				}
			}
#endif
		} else if (inputCmd == 'X') {
			return;
		} else {
			MY_SERIALDEVICE.println(F("!CMD"));
		}
	}
}

void diagnosticsMCUMenu(void)
{
#if defined(ARDUINO_ARCH_ESP8266)
	MY_SERIALDEVICE.println(F("ARCH: ESP8266"));
#elif defined(ARDUINO_ARCH_ESP32)
	MY_SERIALDEVICE.println(F("ARCH: ESP32"));
#elif defined(ARDUINO_ARCH_AVR)
	MY_SERIALDEVICE.println(F("ARCH: AVR"));
#elif defined(ARDUINO_ARCH_SAMD)
	MY_SERIALDEVICE.println(F("ARCH: SAMD"));
#elif defined(ARDUINO_ARCH_STM32F1)
	MY_SERIALDEVICE.println(F("ARCH: STM32F1"));
#elif defined(ARDUINO_ARCH_NRF5) || defined(ARDUINO_ARCH_NRF52)
	MY_SERIALDEVICE.println(F("ARCH: NRF5"));
#elif defined(__arm__) && defined(TEENSYDUINO)
	MY_SERIALDEVICE.println(F("ARCH: Teensyduino"));
#elif defined(__linux__)
	MY_SERIALDEVICE.println(F("ARCH: Linux"));
#else
	MY_SERIALDEVICE.println(F("ARCH: Unknown"));
#endif
#if defined(ARDUINO_ARCH_AVR)
	PRINT(PSTR("AVR fuses: L:%02" PRIX8 ",H:%02" PRIX8 ",E:%02" PRIX8 ",LK:%02" PRIX8
	           "\n"),
	      boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS),
	      boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS),
	      boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS),
	      boot_lock_fuse_bits_get(GET_LOCK_BITS));
#endif
	PRINT(PSTR("T_CPU: %" PRIi8 " C\n"), hwCPUTemperature());
	PRINT(PSTR("V_CPU: %" PRIu16 " mV\n"), hwCPUVoltage());
	MY_SERIALDEVICE.print(F("F_CPU: "));
	MY_SERIALDEVICE.print(hwCPUFrequency() / 10.0);
	MY_SERIALDEVICE.println(F(" MHz"));
	MY_SERIALDEVICE.print(F("CPU ID: "));
	unique_id_t ID;
	const bool result = hwUniqueID(&ID);
	PrintHex8(ID, sizeof(ID));
	PRINT(PSTR("UID unique: %s\n"), result ? "true" : "false");
#if defined(MY_HW_HAS_GETENTROPY)
	MY_SERIALDEVICE.println(F("RNG: True"));
#else
	MY_SERIALDEVICE.println(F("RNG: Pseudo"));
#endif
#if defined(ARDUINO_ARCH_ESP32)
	PRINT(PSTR("Chip rev: %" PRIu8 "\n"), ESP.getChipRevision());
	PRINT(PSTR("Cycles: %" PRIu32 "\n"), ESP.getCycleCount());
	PRINT(PSTR("SDK: %s\n"), ESP.getSdkVersion());
	PRINT(PSTR("EFUSE: %16" PRIX64 "\n"), ESP.getEfuseMac());
	PRINT(PSTR("Total HEAP size: %" PRIu32 "\n"), ESP.getHeapSize());
	PRINT(PSTR("Free HEAP size: %" PRIu32 "\n"), ESP.getFreeHeap());
	PRINT(PSTR("Min HEAP level: %" PRIu32 "\n"), ESP.getMinFreeHeap());
	PRINT(PSTR("Max HEAP alloc: %" PRIu32 "\n"), ESP.getMaxAllocHeap());
	PRINT(PSTR("PSRAM size: %" PRIu32 "\n"), ESP.getPsramSize());
	PRINT(PSTR("Free PSRAM: %" PRIu32 "\n"), ESP.getFreePsram());
	PRINT(PSTR("Min PSRAM level: %" PRIu32 "\n"), ESP.getMinFreePsram());
	PRINT(PSTR("Max PSRAM alloc: %" PRIu32 "\n"), ESP.getMaxAllocPsram());
	PRINT(PSTR("Flash size: %" PRIu32 "\n"), ESP.getFlashChipSize());
	PRINT(PSTR("Flash speed: %" PRIu32 "\n"), ESP.getFlashChipSpeed());
	PRINT(PSTR("Sketch size: %" PRIu32 "\n"), ESP.getSketchSize());
	PRINT(PSTR("Free sketch space: %" PRIu32 "\n"), ESP.getFreeSketchSpace());
#endif

#if defined(ARDUINO_ARCH_ESP8266)
	PRINT(PSTR("Chip id: %08" PRIX32 "\n"), ESP.getChipId());
	PRINT(PSTR("Cycles: %" PRIu32 "\n"), ESP.getCycleCount());
	PRINT(PSTR("SDK: %s\n"), ESP.getSdkVersion());
	PRINT(PSTR("Free HEAP size: %" PRIu32 "\n"), ESP.getFreeHeap());
	PRINT(PSTR("HEAP fragmentation: %" PRIu8 "\n"), ESP.getHeapFragmentation());
	PRINT(PSTR("Max block alloc: %" PRIu32 "\n"), ESP.getMaxFreeBlockSize());
	PRINT(PSTR("Flash id: %08" PRIX32 "\n"), ESP.getFlashChipId());
	PRINT(PSTR("Flash size: %" PRIu32 "\n"), ESP.getFlashChipSize());
	PRINT(PSTR("Flash speed: %" PRIu32 "\n"), ESP.getFlashChipSpeed());
	PRINT(PSTR("Sketch size: %" PRIu32 "\n"), ESP.getSketchSize());
	PRINT(PSTR("Free sketch space: %" PRIu32 "\n"), ESP.getFreeSketchSpace());
#endif
	while (true) {

		diagnosticsPrintSeparationLine();
		MY_SERIALDEVICE.println(F("MCU:\n\n"
		                          "[Dx] Read PIN\n"
		                          "[Sx] Set PIN\n"
		                          "[Rx] Reset PIN\n"
#if defined(ARDUINO_ARCH_AVR)
		                          "[W] WDT\n"
#endif
		                          "[Px] Sleep x ms\n"
		                          "[X] Exit\n"
		                         ));
		diagnosticsPrintSeparationLine();
		diagnosticsFlushSerial();
		diagnosticsSerialInput();
		if (inputCmd == 'D') {
			hwPinMode(inputParameter.toInt(), INPUT);
			PRINT(PSTR("PIN %" PRIu8 " = %" PRIu8 "\n"), inputParameter.toInt(),
			      hwDigitalRead(inputParameter.toInt()));
		} else if (inputCmd == 'S') {
			PRINT(PSTR("SET PIN %" PRIu8 "\n"), inputParameter.toInt());
			hwPinMode(inputParameter.toInt(), OUTPUT);
			hwDigitalWrite(inputParameter.toInt(), HIGH);
		} else if (inputCmd == 'R') {
			PRINT(PSTR("CLR PIN %" PRIu8 "\n"), inputParameter.toInt());
			hwPinMode(inputParameter.toInt(), OUTPUT);
			hwDigitalWrite(inputParameter.toInt(), LOW);
		} else if (inputCmd == 'W') {
#if defined(ARDUINO_ARCH_AVR)
			diagnosticsWatchdogTest();
#endif
		} else if (inputCmd == 'P') {
			PRINT(PSTR("Sleeping %" PRIu32 "ms\n"), inputParameter.toInt());
			transportSleep();
			hwSleep((uint32_t)inputParameter.toInt());
			PRINT(PSTR("waking up\n"));
			transportStandBy();
		} else if (inputCmd == 'X') {
			return;
		} else {
			MY_SERIALDEVICE.println(F("!CMD"));
		}
	}
}

void diagnosticsMainMenu(void)
{
	while (true) {
		diagnosticsPrintSeparationLine();
		MY_SERIALDEVICE.println(F("Main:\n\n"
		                          "[M] MCU\n"
		                          "[E] EEPROM\n"
		                          "[C] CRYPTO\n"
		                          "[R] Reboot\n"
		                          "[I] Info\n"

		                          "[T] TSP SM\n"
#if defined(MY_RADIO_RF24)
		                          "[2] RF24\n"
#endif
#if defined(MY_RADIO_RFM69) && defined(MY_RFM69_NEW_DRIVER)
		                          "[6] RFM69\n"
#endif
#if defined(MY_RADIO_RFM95)
		                          "[9] RFM95\n"
#endif
		                         ));
		diagnosticsPrintSeparationLine();
		diagnosticsFlushSerial();
		diagnosticsSerialInput();
		if (inputCmd == 'T') {
			diagnosticsTransportSM();
		} else if (inputCmd == 'E') {
			diagnosticsEEPROMMenu();
		} else if (inputCmd == 'C') {
#if defined(MY_DIAGNOSTICS_CRYPTO)
			diagnosticsCryptoMenu();
#else
			MY_SERIALDEVICE.println(F("> Define MY_DIAGNOSTICS_CRYPTO to enable"));
#endif
		} else if (inputCmd == 'M') {
			diagnosticsMCUMenu();
		} else if (inputCmd == '2') {
#if defined(MY_RADIO_RF24)
			diagnosticsRF24Menu();
#endif
		} else if (inputCmd == '6') {
#if defined(MY_RADIO_RFM69)
			diagnosticsRFM69Menu();
#endif
		} else if (inputCmd == '9') {
#if defined(MY_RADIO_RFM95)
			diagnosticsRFM95Menu();
#endif
		} else if (inputCmd == 'R') {
			hwReboot();
		} else if (inputCmd == 'I') {
			MY_SERIALDEVICE.println(F("Press any key to exit\n"));
			hwRandomNumberInit();
			while (!MY_SERIALDEVICE.available()) {
				PRINT(PSTR("> T_CPU=%" PRIi8 ", V_CPU=%" PRIu16 ", RNG=%" PRIu8 "\n"),
				      hwCPUTemperature(), hwCPUVoltage(), random(256));
				doYield();
				delay(100);
			}
		} else {
			MY_SERIALDEVICE.println(F("!CMD"));
		}
	}
}


void diagnosticsRun(void)
{
	MY_SERIALDEVICE.println(F("\nMySensors HW diagnostics v1.0"));
	diagnosticsPrintSeparationLine();
	PRINT(PSTR("LIB: MySensors %s\n"), MYSENSORS_LIBRARY_VERSION);
	PRINT(PSTR("REL: %" PRIu8 "\n"), MYSENSORS_LIBRARY_VERSION_PRERELEASE_NUMBER);
	PRINT(PSTR("VER: %" PRIx32 "\n"), MYSENSORS_LIBRARY_VERSION_INT);
	PRINT(PSTR("CAP: %s\n"), MY_CAPABILITIES);
	diagnosticsMainMenu();
}
