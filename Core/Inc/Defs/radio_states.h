#pragma once

#include "semphr.h"
#include "main.h"

#define RESET_PORT GPIOA
#define RESET_PIN GPIO_PIN_4

#define SYNC_VALUE_1 0x2D
#define SYNC_VALUE_2 0x64

extern SemaphoreHandle_t gSpi1Mutex;

typedef enum
{
    SLEEP = 0x00,
    STANDBY = 0x01 << 2,
    FREQUENCY_SYNTH = 0x02 << 2,
    TX = 0x03 << 2,
    RX = 0x04 << 2
} RFM69_Mode;

typedef enum
{
    _250_KBPS_MSB = 0x00,
    _250_KBPS_LSB = 0x80,
} RFM69_Bitrate;

typedef enum
{
    RFM69_BAND_433MHZ,
    RFM69_BAND_868MHZ,
    RFM69_BAND_915MHZ
} RFM69_Band;

typedef struct
{
    uint8_t msb;
    uint8_t mid;
    uint8_t lsb;
} FreqRegisters;

typedef enum
{
    RFM69_BITRATE_250KBPS,
    RFM69_BITRATE_9600BPS,
    RFM69_BITRATE_4800BPS
} RFM69_Bitrate_Selector;

typedef struct
{
    uint8_t msb;
    uint8_t lsb;
} BitrateRegisters;

/**
 * @brief Packet Format (Bit 7)
 */
typedef enum
{
    PACKET_FORMAT_FIXED = 0x00,
    PACKET_FORMAT_VARIABLE = 0x01
} PacketFormat_t;

/**
 * @brief DC-free encoding/decoding (Bits 6-5)
 */
typedef enum
{
    DC_FREE_NONE = 0x00,
    DC_FREE_MANCHESTER = 0x01,
    DC_FREE_WHITENING = 0x02,
    // 0x03 is reserved
} DcFree_t;

/**
 * @brief CRC Calculation/Check (Bit 4)
 */
typedef enum
{
    CRC_OFF = 0x00,
    CRC_ON = 0x01
} CrcOn_t;

/**
 * @brief Behavior when CRC check fails (Bit 3)
 */
typedef enum
{
    CRC_AUTO_CLEAR_ON = 0x00, // Clear FIFO, restart reception
    CRC_AUTO_CLEAR_OFF = 0x01 // Do not clear FIFO, issue interrupt
} CrcAutoClearOff_t;

/**
 * @brief Address based filtering in Rx (Bits 2-1)
 */
typedef enum
{
    ADDRESS_FILTERING_NONE = 0x00,
    ADDRESS_FILTERING_NODE = 0x01,             // Match NodeAddress
    ADDRESS_FILTERING_NODE_OR_BROADCAST = 0x02 // Match NodeAddress or BroadcastAddress
} AddressFiltering_t;

// Bit offsets for RegPacketConfig1 (0x37)
#define RF_PACKETCONFIG1_FORMAT_SHIFT 7
#define RF_PACKETCONFIG1_DCFREE_SHIFT 5
#define RF_PACKETCONFIG1_CRCON_SHIFT 4
#define RF_PACKETCONFIG1_CRCAUTOCLEAROFF_SHIFT 3
#define RF_PACKETCONFIG1_ADDRESSFILTERING_SHIFT 1