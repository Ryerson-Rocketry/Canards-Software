#ifndef RFM9X_H
#define RFM9X_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Pin definitions (match .ioc) ─────────────────────────────────────── */
#define RFM9X_CS_PORT GPIOA
#define RFM9X_CS_PIN GPIO_PIN_4 /* SPI1_CS_3  PA4  */

#define RFM9X_RST_PORT GPIOC
#define RFM9X_RST_PIN GPIO_PIN_5 /* RADIO_RST  PC5  (active LOW on RFM9x) */

#define RFM9X_DIO0_PORT GPIOC
#define RFM9X_DIO0_PIN GPIO_PIN_15 /* RADIO_G0   PC15 (unused for now)     */

/* ── RF settings ──────────────────────────────────────────────────────── */
/* Everything in this block MUST match the ground station receiver.      */
#define RFM9X_FREQUENCY_HZ 433000000UL
#define RFM9X_BANDWIDTH RFM9X_BW_125KHZ
#define RFM9X_SPREADING_FACTOR 9        /* 7 … 12, SF9 is the max that fits 1 Hz */
#define RFM9X_CODING_RATE RFM9X_CR_4_5
#define RFM9X_PREAMBLE_LENGTH 8         /* symbols                            */
#define RFM9X_SYNC_WORD 0x12            /* RadioHead default, 0x34 = LoRaWAN  */
#define RFM9X_CRC_ON true

/* RadioHead-style header sent in front of every payload */
#define RFM9X_HEADER_TO 0xFF            /* 0xFF = broadcast, any node accepts */
#define RFM9X_HEADER_FROM 0x01
#define RFM9X_HEADER_FLAGS 0x00

/* ── Transmitter-only settings ────────────────────────────────────────── */
#define RFM9X_TX_POWER_DBM 17           /* 2 … 17 dBm, +20 is limited to 1% duty */
#define RFM9X_TX_TIMEOUT_MS 2000        /* > airtime of a full packet at SF9  */
#define RFM9X_CALLSIGN "VE3HIV"         /* amateur ID, first CSV field        */

/* ── Bandwidth (RegModemConfig1 bits 7-4) ─────────────────────────────── */
#define RFM9X_BW_62_5KHZ 0x06
#define RFM9X_BW_125KHZ 0x07
#define RFM9X_BW_250KHZ 0x08
#define RFM9X_BW_500KHZ 0x09

/* ── Coding rate (RegModemConfig1 bits 3-1) ───────────────────────────── */
#define RFM9X_CR_4_5 0x01
#define RFM9X_CR_4_6 0x02
#define RFM9X_CR_4_7 0x03
#define RFM9X_CR_4_8 0x04

/* ── RFM9x register map (LoRa mode, subset used here) ─────────────────── */
#define RFM9X_REG_FIFO 0x00
#define RFM9X_REG_OPMODE 0x01
#define RFM9X_REG_FRFMSB 0x06
#define RFM9X_REG_FRFMID 0x07
#define RFM9X_REG_FRFLSB 0x08
#define RFM9X_REG_PACONFIG 0x09
#define RFM9X_REG_FIFOADDRPTR 0x0D
#define RFM9X_REG_FIFOTXBASEADDR 0x0E
#define RFM9X_REG_IRQFLAGS 0x12
#define RFM9X_REG_MODEMCONFIG1 0x1D
#define RFM9X_REG_MODEMCONFIG2 0x1E
#define RFM9X_REG_PREAMBLEMSB 0x20
#define RFM9X_REG_PREAMBLELSB 0x21
#define RFM9X_REG_PAYLOADLENGTH 0x22
#define RFM9X_REG_MODEMCONFIG3 0x26
#define RFM9X_REG_SYNCWORD 0x39
#define RFM9X_REG_VERSION 0x42

/* ── OpMode values ────────────────────────────────────────────────────── */
#define RFM9X_OPMODE_LORA 0x80     /* LongRangeMode bit, only settable in sleep */
#define RFM9X_OPMODE_LOW_FREQ 0x08 /* LowFrequencyModeOn, for bands < 525 MHz  */
#define RFM9X_MODE_SLEEP 0x00
#define RFM9X_MODE_STANDBY 0x01
#define RFM9X_MODE_TX 0x03

/* ── IRQ flags ────────────────────────────────────────────────────────── */
#define RFM9X_IRQ_TXDONE 0x08

#define RFM9X_VERSION 0x12

/* ── Max payload ─────────────────────────────────────────────────────── */
#define RFM9X_HEADER_LEN 4
#define RFM9X_MAX_PAYLOAD (255 - RFM9X_HEADER_LEN) /* matches RH_RF95_MAX_MESSAGE_LEN */

/* ── Public API ───────────────────────────────────────────────────────── */
HAL_StatusTypeDef rfm9xInit(void);
HAL_StatusTypeDef rfm9xTransmit(const uint8_t *data, uint8_t len);

#endif /* RFM9X_H */
