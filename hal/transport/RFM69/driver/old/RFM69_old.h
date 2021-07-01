// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright Felix Rusu (2014), felix@lowpowerlab.com
// http://lowpowerlab.com/
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#ifndef RFM69_h
#define RFM69_h

#if !defined(RFM69_SPI)
#define RFM69_SPI hwSPI	//!< default SPI
#endif

#define RFM69_MAX_DATA_LEN       (61u) // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)

#if defined(ARDUINO_ARCH_AVR)
#if defined(__AVR_ATmega32U4__)
#define DEFAULT_RFM69_IRQ_PIN			(3)													//!< DEFAULT_RFM69_IRQ_PIN
#else
#define DEFAULT_RFM69_IRQ_PIN			(2)													//!< DEFAULT_RFM69_IRQ_PIN
#endif
#define DEFAULT_RFM69_IRQ_NUM			digitalPinToInterrupt(MY_RFM69_IRQ_PIN)		//!< DEFAULT_RFM69_IRQ_NUM
#elif defined(ARDUINO_ARCH_ESP8266)
#define DEFAULT_RFM69_IRQ_PIN			(5)													//!< DEFAULT_RFM69_IRQ_PIN
#define DEFAULT_RFM69_IRQ_NUM			digitalPinToInterrupt(MY_RFM69_IRQ_PIN)		//!< DEFAULT_RFM69_IRQ_NUM
#elif defined(ARDUINO_ARCH_ESP32)
#define DEFAULT_RFM69_IRQ_PIN			(16)												//!< DEFAULT_RFM69_IRQ_PIN
#define DEFAULT_RFM69_IRQ_NUM			digitalPinToInterrupt(DEFAULT_RFM69_IRQ_PIN)		//!< DEFAULT_RFM69_IRQ_NUM
#elif defined(ARDUINO_ARCH_SAMD)
#define DEFAULT_RFM69_IRQ_PIN			(2)													//!< DEFAULT_RFM69_IRQ_PIN
#define DEFAULT_RFM69_IRQ_NUM			digitalPinToInterrupt(MY_RFM69_IRQ_PIN)		//!< DEFAULT_RFM69_IRQ_NUM
#elif defined(LINUX_ARCH_RASPBERRYPI)
#define DEFAULT_RFM69_IRQ_PIN			(22)												//!< DEFAULT_RFM69_IRQ_PIN
#define DEFAULT_RFM69_IRQ_NUM			DEFAULT_RFM69_IRQ_PIN								//!< DEFAULT_RFM69_IRQ_NUM
#elif defined(ARDUINO_ARCH_STM32)
#define DEFAULT_RFM69_IRQ_PIN			(PA3)												//!< DEFAULT_RFM69_IRQ_PIN
#define DEFAULT_RFM69_IRQ_NUM			DEFAULT_RFM69_IRQ_PIN								//!< DEFAULT_RFM69_IRQ_NUM
#elif defined(ARDUINO_ARCH_STM32F1)
#define DEFAULT_RFM69_IRQ_PIN			(PA3)												//!< DEFAULT_RFM69_IRQ_PIN
#define DEFAULT_RFM69_IRQ_NUM			DEFAULT_RFM69_IRQ_PIN								//!< DEFAULT_RFM69_IRQ_NUM
#elif defined(TEENSYDUINO)
#define DEFAULT_RFM69_IRQ_PIN			(8)													//!< DEFAULT_RFM69_IRQ_PIN
#define DEFAULT_RFM69_IRQ_NUM			digitalPinToInterrupt(MY_RFM69_IRQ_PIN)		//!< DEFAULT_RFM69_IRQ_NUM
#else
#define DEFAULT_RFM69_IRQ_PIN			(2)													//!< DEFAULT_RFM69_IRQ_PIN
#define DEFAULT_RFM69_IRQ_NUM			(2)													//!< DEFAULT_RFM69_IRQ_NUM
#endif

#define DEFAULT_RFM69_CS_PIN			(SS)												//!< DEFAULT_RFM69_CS_PIN

// SPI clock divier for non-transaction implementations
#if (MY_RFM69_SPI_SPEED >= F_CPU / 2)
#define RFM69_CLOCK_DIV SPI_CLOCK_DIV2			//!< SPI clock divider 2
#elif (MY_RFM69_SPI_SPEED >= F_CPU / 4)
#define RFM69_CLOCK_DIV SPI_CLOCK_DIV4			//!< SPI clock divider 4
#elif (MY_RFM69_SPI_SPEED >= F_CPU / 8)
#define RFM69_CLOCK_DIV SPI_CLOCK_DIV8			//!< SPI clock divider 8
#elif (MY_RFM69_SPI_SPEED >= F_CPU / 16)
#define RFM69_CLOCK_DIV SPI_CLOCK_DIV16			//!< SPI clock divider 16
#elif (MY_RFM69_SPI_SPEED >= F_CPU / 32)
#define RFM69_CLOCK_DIV SPI_CLOCK_DIV32			//!< SPI clock divider 32
#elif (MY_RFM69_SPI_SPEED >= F_CPU / 64)
#define RFM69_CLOCK_DIV SPI_CLOCK_DIV64			//!< SPI clock divider 64
#elif (MY_RFM69_SPI_SPEED >= F_CPU / 128)
#define RFM69_CLOCK_DIV SPI_CLOCK_DIV128		//!< SPI clock divider 128
#else
#define RFM69_CLOCK_DIV SPI_CLOCK_DIV256		//!< SPI clock divider 256
#endif

// powerup delay
#define RFM69_POWERUP_DELAY_MS			(100u)		//!< Power up delay, allow VCC to settle, transport to become fully operational

#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RFM69_MODE_SLEEP         0 // XTAL OFF
#define RFM69_MODE_STANDBY       1 // XTAL ON
#define RFM69_MODE_SYNTH         2 // PLL ON
#define RFM69_MODE_RX            3 // RX MODE
#define RFM69_MODE_TX            4 // TX MODE

// available frequency bands
#define RFM69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RFM69_433MHZ            43
#define RFM69_868MHZ            86
#define RFM69_915MHZ            91

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RFM69_BROADCAST_ADDR 255
#define RFM69_CSMA_LIMIT_MS 1000
#define RFM69_TX_LIMIT_MS   1000

#define RFM69_FXOSC			(32*1000000ul)				//!< The crystal oscillator frequency of the module, 32MHz
#define RFM69_FSTEP			(RFM69_FXOSC / 524288ul)	//!< The Frequency Synthesizer step

// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40

/** RFM69 class */
class RFM69
{
public:
	static volatile uint8_t DATA[RFM69_MAX_DATA_LEN]; //!< recv/xmit buf, including hdr & crc bytes
	static volatile uint8_t DATALEN; //!< DATALEN
	static volatile uint8_t SENDERID;  //!< SENDERID
	static volatile uint8_t TARGETID; //!<  should match _address
	static volatile uint8_t PAYLOADLEN; //!< PAYLOADLEN
	static volatile uint8_t ACK_REQUESTED; //!< ACK_REQUESTED
	static volatile uint8_t
	ACK_RECEIVED; //!< Should be polled immediately after sending a packet with ACK requestwith ACK request
	static volatile int16_t RSSI; //!<  most accurate RSSI during reception (closest to the reception)
	static volatile uint8_t _mode; //!<  should be protected?

	/**
	 * @brief Constructor
	 *
	 * @param slaveSelectPin ChipSelect pin.
	 * @param interruptPin Interrupt pin.
	 * @param isRFM69HW Set to @c true to indicate RFM69HW variant.
	 * @param interruptNum Interrupt number.
	 */
	// cppcheck-suppress uninitMemberVar
	RFM69(uint8_t slaveSelectPin=MY_RFM69_CS_PIN, uint8_t interruptPin=MY_RFM69_IRQ_PIN,
	      bool isRFM69HW=false,
	      uint8_t interruptNum=digitalPinToInterrupt(MY_RFM69_IRQ_PIN))
	{
		_slaveSelectPin = slaveSelectPin;
		_interruptPin = interruptPin;
		_interruptNum = interruptNum;
		_mode = RFM69_MODE_STANDBY;
		_promiscuousMode = false;
		_powerLevel = 31;
		_isRFM69HW = isRFM69HW;
		_address = RFM69_BROADCAST_ADDR;
#if !defined(SPI_HAS_TRANSACTION)
#if defined (SPCR) && defined (SPSR)
		_SPCR = 0;
		_SPSR = 0;
#endif
#if defined (SREG)
		_SREG = 0;
#endif
#endif // SPI_HAS_TRANSACTION
	}

	bool initialize(uint8_t freqBand, uint8_t ID, uint8_t networkID=1); //!< initialize
	void setAddress(uint8_t addr); //!< setAddress
	void setNetwork(uint8_t networkID); //!< setNetwork
	bool canSend(); //!< canSend
	virtual void send(uint8_t toAddress, const void* buffer, uint8_t bufferSize,
	                  bool requestACK=false); //!< send
	virtual bool sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize,
	                           uint8_t retries=5, uint8_t retryWaitTime=
	                               200); //!< sendWithRetry (40ms roundtrip req for 61byte packets, adjusted)
	virtual bool receiveDone(); //!< receiveDone
	bool ACKReceived(uint8_t fromNodeID); //!< ACKReceived
	bool ACKRequested(); //!< ACKRequested
	virtual void sendACK(const void* buffer = "", uint8_t bufferSize=0); //!< sendACK
	uint32_t getFrequency(); //!< getFrequency
	void setFrequency(uint32_t freqHz); //!< setFrequency
	void encrypt(const char* key); //!< encrypt
	void setCS(uint8_t newSPISlaveSelect); //!< setCS
	int16_t readRSSI(bool forceTrigger=false); //!< readRSSI
	void promiscuous(bool onOff=true); //!< promiscuous
	virtual void setHighPower(bool onOFF=
	                              true); //!< setHighPower (have to call it after initialize for RFM69HW)
	virtual void setPowerLevel(uint8_t level); //!< setPowerLevel (reduce/increase transmit power level)
	void sleep(void);			//!< sleep
	void standBy(void);			//!< standBy
	void powerDown(void);		//!< powerDown
	void powerUp(void);			//!< powerUp
	void reset(void);			//!< reset
	bool sanityCheck(void);	//!< sanityCheck
	uint8_t readTemperature(uint8_t calFactor=0); //!< readTemperature (get CMOS temperature (8bit))
	void rcCalibration(); //!< rcCalibration (calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy])

	// allow hacking registers by making these public
	uint8_t readReg(uint8_t addr); //!< readReg
	void writeReg(uint8_t addr, uint8_t val); //!< writeReg
	void readAllRegs(); //!< readAllRegs
protected:
	static void isr0(); //!< isr0
	void virtual interruptHandler(); //!< interruptHandler
	virtual void interruptHook(uint8_t CTLbyte); //!< interruptHook
	virtual void sendFrame(uint8_t toAddress, const void* buffer, uint8_t size, bool requestACK=false,
	                       bool sendACK=false); //!< sendFrame

	static RFM69* selfPointer; //!< selfPointer
	uint8_t _slaveSelectPin; //!< _slaveSelectPin
	uint8_t _interruptPin; //!< _interruptPin
	uint8_t _interruptNum; //!< _interruptNum
	uint8_t _address; //!< _address
	bool _promiscuousMode; //!< _promiscuousMode
	uint8_t _powerLevel; //!< _powerLevel
	bool _isRFM69HW; //!< _isRFM69HW
#if defined (SPCR) && defined (SPSR)
	uint8_t _SPCR; //!< _SPCR
	uint8_t _SPSR; //!< _SPSR
#endif
#if defined (SREG)
	uint8_t _SREG; //!< _SREG
#endif

	virtual void receiveBegin(); //!< receiveBegin
	virtual void setMode(uint8_t mode); //!< setMode
	virtual void setHighPowerRegs(bool onOff); //!< setHighPowerRegs
	virtual void select(); //!< select
	virtual void unselect(); //!< unselect
};

#endif
