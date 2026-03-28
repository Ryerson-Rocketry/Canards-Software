#ifndef RFM69_H
#define RFM69_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Pin definitions (match .ioc) ─────────────────────────────────────── */
#define RADIO_CS_PORT GPIOA
#define RADIO_CS_PIN GPIO_PIN_4 /* SPI1_CS_3  PA4  */

#define RADIO_RST_PORT GPIOC
#define RADIO_RST_PIN GPIO_PIN_5 /* RADIO_RST  PC5  */

#define RADIO_EN_PORT GPIOC
#define RADIO_EN_PIN GPIO_PIN_3 /* Radio_EN   PC3  */

#define RADIO_DIO0_PORT GPIOC
#define RADIO_DIO0_PIN GPIO_PIN_15 /* RADIO_G0   PC15 */

/* ── RF settings ──────────────────────────────────────────────────────── */
#define RFM69_FREQUENCY_MHZ 433.0f
#define RFM69_TX_POWER_DBM 20    /* –18 … +20 dBm (HCW high-power) */
#define RFM69_IS_HIGH_POWER true /* RFM69HCW always true            */

/* ── RFM69 register map (subset used here) ────────────────────────────── */
#define REG_FIFO 0x00
#define REG_OPMODE 0x01
#define REG_DATAMODUL 0x02
#define REG_BITRATEMSB 0x03
#define REG_BITRATELSB 0x04
#define REG_FDEVMSB 0x05
#define REG_FDEVLSB 0x06
#define REG_FRFMSB 0x07
#define REG_FRFMID 0x08
#define REG_FRFLSB 0x09
#define REG_PALEVEL 0x11
#define REG_OCP 0x13
#define REG_LNA 0x18
#define REG_RXBW 0x19
#define REG_IRQFLAGS1 0x27
#define REG_IRQFLAGS2 0x28
#define REG_SYNCCONFIG 0x2E
#define REG_SYNCVALUE1 0x2F
#define REG_PACKETCONFIG1 0x37
#define REG_PAYLOADLENGTH 0x38
#define REG_FIFOTHRESH 0x3C
#define REG_PACKETCONFIG2 0x3D
#define REG_TESTDAGC 0x6F
#define REG_TESTPA1 0x5A /* high-power PA boost */
#define REG_TESTPA2 0x5C
#define REG_SYNCVALUE2 0x30

/* ── OpMode values ────────────────────────────────────────────────────── */
#define RF_OPMODE_SLEEP 0x00
#define RF_OPMODE_STANDBY 0x04
#define RF_OPMODE_FS 0x08
#define RF_OPMODE_TRANSMITTER 0x0C
#define RF_OPMODE_RECEIVER 0x10

/* ── IRQ flags ────────────────────────────────────────────────────────── */
#define RF_IRQFLAGS2_PACKETSENT 0x08
#define RF_IRQFLAGS2_FIFONOTEMPTY 0x40
#define RF_IRQFLAGS1_MODEREADY 0x80

/* ── Max payload ─────────────────────────────────────────────────────── */
#define RFM69_MAX_PAYLOAD 61 /* matches RadioHead RH_RF69_MAX_MESSAGE_LEN */

/* ── Public API ───────────────────────────────────────────────────────── */
HAL_StatusTypeDef rfm69Init();
HAL_StatusTypeDef rfm69Transmit(uint8_t *data, uint8_t len);

#endif /* RFM69_H */
