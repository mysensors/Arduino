bool _initFilters();
bool transportInit(void);

void _cleanSlot(uint8_t slot);

uint8_t _findCanPacketSlot();

uint8_t _findCanPacketSlot(long unsigned int from, long unsigned int currentPart,
                           long unsigned int messageId);

bool transportSend(const uint8_t to, const void* data, const uint8_t len, const bool noACK);

bool transportDataAvailable(void);

uint8_t transportReceive(void* data);

void transportSetAddress(const uint8_t address);

uint8_t transportGetAddress(void);

bool transportSanityCheck(void);

void transportPowerDown(void);

void transportPowerUp(void);

void transportSleep(void);

void transportStandBy(void);

int16_t transportGetSendingRSSI(void);

int16_t transportGetReceivingRSSI(void);

int16_t transportGetSendingSNR(void);

int16_t transportGetReceivingSNR(void);

int16_t transportGetTxPowerPercent(void);

int16_t transportGetTxPowerLevel(void);

bool transportSetTxPowerPercent(const uint8_t powerPercent);

