// from 8bits (A)
// to 8bits (B)
// current part 4 bits (C)
// total part count 4bits (D)
// 3 bits message_id (E)
// 1 bit require ack (F)
// 1 bit is ack (G)
// header model (29 bits)
// G FEEE DDDD CCCC BBBB BBBB AAAA AAAA

#include "hal/transport/CAN/driver/mcp_can.h"
#include "hal/transport/CAN/driver/mcp_can.cpp"
#include "MyTransportCAN.h"
#if defined(MY_DEBUG_VERBOSE_CAN)
#define CAN_DEBUG(x,...)	DEBUG_OUTPUT(x, ##__VA_ARGS__)	//!< Debug print
#else
#define CAN_DEBUG(x,...)	//!< DEBUG null
#endif
MCP_CAN CAN0(CAN_CS);
bool canInitialized=false;

//input buffer for raw data (from library).
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];


unsigned char _nodeId;

//buffer element
typedef struct {
    uint8_t len;
    uint8_t data[MAX_MESSAGE_SIZE];
    uint8_t address;
    uint8_t lastReceivedPart;
    bool locked;
    uint8_t age;
    uint8_t packetId;
    bool ready;
} CAN_Packet;

CAN_Packet packets[CAN_BUF_SIZE];

//filter incoming messages (MCP2515 feature)
void _initFilters() {
    if (!canInitialized)
        return;
    CAN0.setMode(MODE_CONFIG);
    CAN0.init_Mask(0,1,0x0000FF00);                // Init first mask. Only destination address will be used to filter messages
    CAN0.init_Filt(0,1,BROADCAST_ADDRESS<<8);      // Init first filter. Accept broadcast messages.
    CAN0.init_Filt(1,1,_nodeId<<8);                // Init second filter. Accept messages send to this node.
//second mask and filters need to be set. Otherwise all messages would be accepted.
    CAN0.init_Mask(1,1,0xFFFFFFFF);                // Init second mask.
    CAN0.init_Filt(2,1,0xFFFFFFFF);                // Init third filter.
    CAN0.init_Filt(3,1,0xFFFFFFFF);                // Init fourth filter.
    CAN0.init_Filt(4,1,0xFFFFFFFF);                // Init fifth filter.
    CAN0.init_Filt(5,1,0xFFFFFFFF);                // Init sixth filter.
    CAN0.setMode(MCP_NORMAL);
    hwPinMode(CAN_INT, INPUT);
}
bool transportInit(void)
{
    CAN_DEBUG(PSTR("CAN:INIT:CS=%" PRIu8 ",INT=%" PRIu8 ",SPE=%" PRIu8 ",CLO=%" PRIu8 "\n"), CAN_CS, CAN_INT, CAN_SPEED, CAN_CLOCK);

    if(CAN0.begin(MCP_STDEXT, CAN_SPEED, CAN_CLOCK) != CAN_OK) {
        canInitialized=false;
        return false;
    }
    canInitialized=true;
    for (uint8_t i = 0; i < CAN_BUF_SIZE; i++) {
        _cleanSlot(i);
    }
    _initFilters();
    return true;

}

//clear single slot in buffer.
void _cleanSlot(uint8_t slot) {
    packets[slot].locked=false;
    packets[slot].len=0;
    packets[slot].address=0;
    packets[slot].lastReceivedPart=0;
    packets[slot].age=0;
    packets[slot].packetId=0;
    packets[slot].ready=false;
}

//find empty slot in buffer
uint8_t _findCanPacketSlot() {
    uint8_t slot=CAN_BUF_SIZE;
    uint8_t i;
    for (i = 0; i < CAN_BUF_SIZE; i++) {
        if(packets[i].locked) {
            packets[i].age++;
        } else {
            slot=i;
        }
    }
    if(slot < CAN_BUF_SIZE)
        return slot;
    //if empty slot not found. Clear oldest message.
    slot=0;
    for (i = 1; i < CAN_BUF_SIZE; i++) {
        if(packets[i].age>packets[slot].age) {
            slot=i;
        }
    }
    _cleanSlot(slot);
    //log error (message dropped)
    return slot;
}

//find slot with previous data parts.
uint8_t _findCanPacketSlot(long unsigned int from,long unsigned int currentPart,long unsigned int messageId){
    uint8_t slot=CAN_BUF_SIZE;
    uint8_t i;
    for (i = 0; i < CAN_BUF_SIZE; i++) {
        if(packets[i].locked && packets[i].address==from && packets[i].packetId==messageId && packets[i].lastReceivedPart==currentPart+1) {
            slot=i;
        }
    }
    if (slot == CAN_BUF_SIZE) {
        //log error. Received message id not found in buffer.
    }
    return slot;
}

bool transportSend(const uint8_t to, const void* data, const uint8_t len, const bool noACK)
{
    (void)noACK;	// some ack is provided by CAN itself. TODO implement application layer ack.
    const char *datap = static_cast<char const *>(data);
    //calculate number of frames
    uint8_t noOfFrames = len / 8;
    if (len % 8 != 0) {
        noOfFrames++;
    }
//set left most bit to 1 as this indicates extended frame
    uint8_t h1 = 0x80;
    //TODO increment msg_id for new message.
                //    uint8_t msg_id = 7;
                //add msg_id. 7 is maximum allowed value (3bits)
                //    h1 += (msg_id & 0x07);
                //    if (!noACK) {
                        //set require ACK bit - to be implemented
                //        h1 = h1 | 0x08;
                //    }

    //set total number of frames
    uint8_t h2 = noOfFrames;
    //shift left to create space for current frame number
    h2=h2 << 4;
    uint8_t i = 0;
    CAN_DEBUG(PSTR("CAN:SND:LN=%" PRIu8 ",NOF=%" PRIu8 "\n"), len, noOfFrames);

    for (i = 0; i < noOfFrames; i++) {
        uint32_t canId = h1;
        canId=canId << 8;
        //reset current frame number
        h2 = h2 & 0xF0;
        h2 += i;
        canId += h2;
        canId=canId << 8;
        canId += to;
        canId=canId << 8;
        canId += _nodeId;
        uint8_t partLen;
        if (len<=8){
            partLen=len;
        } else if (i * 8 <= len) {
            partLen = 8;
        } else {
            partLen = len % 8;
        }
        uint8_t buff[8];
        uint8_t j=0;
//        memcpy(buff,datap[i*8+j],partlen);
        for (j = 0; j < partLen; j++) {
            buff[j]=datap[i*8+j];
        }
        CAN_DEBUG(PSTR("CAN:SND:LN=%" PRIu8 ",DTA0=%" PRIu8 ",DTA1=%" PRIu8 ",DTA2=%" PRIu8 ",DTA3=%" PRIu8 ",DTA4=%" PRIu8 ",DTA5=%" PRIu8 ",DTA6=%" PRIu8 ",DTA7=%" PRIu8 "\n"), partLen, buff[0], buff[1], buff[2],buff[3],buff[4],buff[5],buff[6],buff[7]);

        CAN_DEBUG(PSTR("CAN:SND:LN=%" PRIu8 ",CANH=%" PRIu32 "\n"), partLen, canId);

        byte sndStat = CAN0.sendMsgBuf(canId, partLen, buff);
        if (sndStat == CAN_OK) {
            CAN_DEBUG(PSTR("CAN:SND:OK\n"));
            return true;
        } else {
            CAN_DEBUG(PSTR("!CAN:SND:FAIL\n"));
            return false;
        }
    }
}
bool transportDataAvailable(void)
{
    if(!hwDigitalRead(CAN_INT))                         // If CAN_INT pin is low, read receive buffer
    {
        CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
        long unsigned int from=(rxId & 0x000000FF);
        long unsigned int to=(rxId & 0x0000FF00)>>8;
        long unsigned int currentPart=(rxId & 0x000F0000)>>16;
        long unsigned int totalPartCount=(rxId & 0x00F00000)>>20;
        long unsigned int messageId=(rxId & 0x07000000)>>24;
        CAN_DEBUG(PSTR("CAN:RCV:CANH=%" PRIu32 ",FROM=%" PRIu8 ",TO=%" PRIu8 ",CURR=%" PRIu8 ",TOTAL=%" PRIu8 ",ID=%" PRIu8 "\n"), rxId, from, to, currentPart,totalPartCount,messageId);

        uint8_t slot;
        if(currentPart==0){
            slot=_findCanPacketSlot();
            packets[slot].locked=true;
        } else {
            slot=_findCanPacketSlot(from,currentPart,messageId);
        }
        if (slot != CAN_BUF_SIZE) {
            memcpy(packets[slot].data + packets[slot].len, rxBuf, len);
            packets[slot].lastReceivedPart++;
            packets[slot].len += len;
            CAN_DEBUG(PSTR("CAN:RCV:SLOT=%" PRIu8 ",PART=%" PRIu8 "\n"), slot, packets[slot].lastReceivedPart);
            if (packets[slot].lastReceivedPart == totalPartCount) {
                packets[slot].ready = true;
                CAN_DEBUG(PSTR("CAN:RCV:SLOT=%" PRIu8 " complete\n"), slot);
                return true;
            }

        }
    }
    return false;
}
uint8_t transportReceive(void* data)
{
    uint8_t slot=CAN_BUF_SIZE;
    uint8_t i;
    for (i = 0; i < CAN_BUF_SIZE; i++) {
        if(packets[i].ready) {
            slot=i;
        }
    }
    if (slot < CAN_BUF_SIZE) {
        memcpy(data,packets[slot].data,packets[slot].len);
        i=packets[slot].len;
        _cleanSlot(slot);
        return i;
    } else {
        return (0);
    }
}
void transportSetAddress(const uint8_t address)
{
    _nodeId = address;
}

uint8_t transportGetAddress(void)
{
    return _nodeId;
}
bool transportSanityCheck(void)
{
    // not implemented yet
    return true;
}
void transportPowerDown(void)
{
    // Nothing to shut down here
}

void transportPowerUp(void)
{
    // not implemented
}

void transportSleep(void)
{
    // not implemented
}

void transportStandBy(void)
{
    // not implemented
}

int16_t transportGetSendingRSSI(void)
{
    // not implemented
    return INVALID_RSSI;
}

int16_t transportGetReceivingRSSI(void)
{
    // not implemented
    return INVALID_RSSI;
}

int16_t transportGetSendingSNR(void)
{
    // not implemented
    return INVALID_SNR;
}

int16_t transportGetReceivingSNR(void)
{
    // not implemented
    return INVALID_SNR;
}

int16_t transportGetTxPowerPercent(void)
{
    // not implemented
    return static_cast<int16_t>(100);
}

int16_t transportGetTxPowerLevel(void)
{
    // not implemented
    return static_cast<int16_t>(100);
}

bool transportSetTxPowerPercent(const uint8_t powerPercent)
{
    // not possible
    (void)powerPercent;
    return false;
}
