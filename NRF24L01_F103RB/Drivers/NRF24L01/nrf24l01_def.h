/*
 * nrf24l01_def.h
 *
 *  Created on: Feb 16, 2020
 *      Author: lgengqiang
 */

#if defined(STM32F030x6)
#include "stm32f0xx_hal.h"
#endif

#if defined(STM32F103xB) || defined(STM32F103xE)
#include "stm32f1xx_hal.h"
#endif

#if defined(STM32F401xC) || defined(STM32F411xE)
#include "stm32f4xx_hal.h"
#endif

#if defined(STM32L051xx) || defined(STM32L071xx)
#include "stm32l0xx_hal.h"
#endif

/* -------------------------------------------------- */

#define NRF24L01_COMMAND_R_REGISTER			0x00 // R_REGISTER. Read command and status registers. AAAAA = 5 bit Register Map Address
#define NRF24L01_COMMAND_W_REGISTER			0x20 // W_REGISTER. Write command and status registers. AAAAA = 5 bit Register Map Address. Executable in power down or standby modes only.
#define NRF24L01_COMMAND_R_RX_PAYLOAD		0x61 // R_RX_PAYLOAD. Read RX-payload: 1 – 32 bytes. A read operation always starts at byte 0. Payload is deleted from FIFO after it is read. Used in RX mode.
#define NRF24L01_COMMAND_W_TX_PAYLOAD		0xA0 // W_TX_PAYLOAD. Write TX-payload: 1 – 32 bytes. A write operation always starts at byte 0 used in TX payload.
#define NRF24L01_COMMAND_FLUSH_TX			0xE1 // FLUSH_TX. Flush TX FIFO, used in TX mode
#define NRF24L01_COMMAND_FLUSH_RX			0xE2 // FLUSH_RX. Flush RX FIFO, used in RX mode
#define NRF24L01_COMMAND_REUSE_TX_PL		0xE3 // REUSE_TX_PL
#define NRF24L01_COMMAND_ACTIVATE			0x50 // R_REGISTER
#define NRF24L01_COMMAND_R_RX_PL_WID		0x60 // R_RX_PL_WID. Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO.
#define NRF24L01_COMMAND_W_ACK_PAYLOAD		0xA8 // W_ACK_PAYLOAD. Used in RX mode. Write Payload to be transmitted together with ACK packet on PIPE PPP.
#define NRF24L01_COMMAND_W_TX_PAYLOAD_NOACK	0xB0 // W_TX_PAYLOAD_NOACK. Used in TX mode. Disables AUTOACK on this specific packet.
#define NRF24L01_COMMAND_NOP				0xFF // NOP. No Operation. Might be used to read the STATUS register

#define NRF24L01_MASK_RW_REGISTER			0x1F
//#define NRF24L01_MASK_W_REGISTER			0x20

#define NRF24L01_REG_CONFIG			0x00 // Configuration Register
#define NRF24L01_REG_EN_AA			0x01 // Enable 'Auto Acknowledgment'
#define NRF24L01_REG_EN_RXADDR		0x02 // Enabled RX Addresses
#define NRF24L01_REG_SETUP_AW		0x03 // Setup of Address Widths
#define NRF24L01_REG_SETUP_RETR		0x04 // Setup of Automatic Retransmission
#define NRF24L01_REG_RF_CH			0x05 // RF Channel
#define NRF24L01_REG_RF_SETUP		0x06 // RF Setup Register
#define NRF24L01_REG_STATUS			0x07 // Status Register
#define NRF24L01_REG_OBSERVE_TX		0x08 // Transmit observe register
#define NRF24L01_REG_CD				0x09 // Carrier Detect
#define NRF24L01_REG_RX_ADDR_P0		0x0A // Receive address data pipe 0. 5 Bytes maximum length.
#define NRF24L01_REG_RX_ADDR_P1	 	0x0B // Receive address data pipe 1. 5 Bytes maximum length.
#define NRF24L01_REG_RX_ADDR_P2		0x0C // Receive address data pipe 2. 5 Bytes maximum length.
#define NRF24L01_REG_RX_ADDR_P3		0x0D // Receive address data pipe 3. 5 Bytes maximum length.
#define NRF24L01_REG_RX_ADDR_P4		0x0E // Receive address data pipe 4. 5 Bytes maximum length.
#define NRF24L01_REG_RX_ADDR_P5 	0x0F // Receive address data pipe 5. 5 Bytes maximum length.
#define NRF24L01_REG_TX_ADDR		0x10 // Transmit address. Used for a PTX device only.
#define NRF24L01_REG_RX_PW_P0		0x11
#define NRF24L01_REG_RX_PW_P1		0x12
#define NRF24L01_REG_RX_PW_P2		0x13
#define NRF24L01_REG_RX_PW_P3		0x14
#define NRF24L01_REG_RX_PW_P4		0x15
#define NRF24L01_REG_RX_PW_P5		0x16
#define NRF24L01_REG_FIFO_STATUS	0x17 // FIFO Status Register
#define NRF24L01_REG_DYNPD			0x1C // Enable dynamic payload length
#define NRF24L01_REG_FEATURE		0x1D // Feature Register

// Register: CONFIG(0x00)
#define MASK_RX_DR				0x40
#define MASK_TX_DS				0x20
#define MASK_MAX_RT				0x10
#define MASK_EN_CRC				0x08
#define MASK_CRCO				0x04
#define MASK_PWR_UP				0x02
#define MASK_PRIM_RX			0x01

// Register: EN_AA(0x01)
#define MASK_ENAA_P5			0x20
#define MASK_ENAA_P4			0x10
#define MASK_ENAA_P3			0x08
#define MASK_ENAA_P2			0x04
#define MASK_ENAA_P1			0x02
#define MASK_ENAA_P0			0x01
#define MASK_ENAA_ALL			0x3F

// Register: EN_RXADDR(0x02)
#define MASK_ERX_P5				0x20
#define MASK_ERX_P4				0x10
#define MASK_ERX_P3				0x08
#define MASK_ERX_P2				0x04
#define MASK_ERX_P1				0x02
#define MASK_ERX_P0				0x01

// Register: SETUP_AW(0x03)
#define MASK_AW_3_BYTES			0x01
#define MASK_AW_4_BYTES			0x02
#define MASK_AW_5_BYTES			0x03

// Register: SETUP_RETR(0x04)
#define MASK_ARD				0xF0
#define MASK_ARC				0x0F

// Register: RF_CH(0x05)
#define MASK_RF_CH				0x7F

// Register: RF_SETUP(0x06)
#define MASK_RF_DR_LOW			0x20
#define MASK_RF_DR_HIGH			0x08
#define MASK_RF_PWR				0x06
#define MASK_RF_LNA_HCURR		0x01

//#define MASK_RF_PWR_18dBm		0x00 // RF output power: -18dBm
//#define MASK_RF_PWR_12dBm		0x02 // RF output power: -12dBm
//#define MASK_RF_PWR_6dBm		0x04 // RF output power: -6dBm
//#define MASK_RF_PWR_0dBm		0x06 // RF output power: 0dBm

// Register: STATUS(0x07)
#define MASK_RX_DR				0x40
#define MASK_TX_DS				0x20
#define MASK_MAX_RT				0x10
#define MASK_RX_P_NO			0x0E
#define MASK_STATUS_TX_FULL 	0x01

// Register: OBSERVE_TX(0x08)
#define MASK_PLOS_CNT			0xF0 // Count lost packets
#define MASK_ARC_CNT			0x0F // Count retransmitted packets

// Register: CD(0x09)
#define MASK_CD					0x01 // Carrier Detect

// Register: FIFO_STATUS(0x17)
#define MASK_FIFO_TX_REUSE		0x40
#define MASK_FIFO_TX_FULL 		0x20
#define MASK_FIFO_TX_EMPTY		0x10
#define MASK_FIFO_RX_FULL		0x02
#define MASK_FIFO_RX_EMPTY		0x01

// Register: DYNPD(0x1C)
#define MASK_DPL_P5				0x20
#define MASK_DPL_P4				0x10
#define MASK_DPL_P3				0x08
#define MASK_DPL_P2				0x04
#define MASK_DPL_P1				0x02
#define MASK_DPL_P0				0x01

// Register: FEATURE(0x1D)
#define MASK_EN_DPL				0x04
#define MASK_EN_ACK_PAY			0x02
#define MASK_EN_DYN_ACK			0x01

/* -------------------------------------------------- */

#define NRF24L01_RX_PIPE_0		0x01
#define NRF24L01_RX_PIPE_1		0x02
#define NRF24L01_RX_PIPE_2		0x03
#define NRF24L01_RX_PIPE_3		0x04
#define NRF24L01_RX_PIPE_4		0x05
#define NRF24L01_RX_PIPE_5		0x06
#define NRF24L01_RX_PIPE_ALL	0xFF

/* -------------------------------------------------- */

#define _NRF24L01_IRQ_REG
//#define _NRF24L01_IRQ_GPIO

typedef enum
{
	TX_OK = (uint8_t)0x00,
	TX_MAX_RETRY = (uint8_t)0x01,
	TX_TIMEOUT = (uint8_t)0x02,
	TX_ERROR = (uint8_t)0x04,
} TXResult_TypeDef;

/* -------------------------------------------------- */

#define SI24R1

typedef enum
{
	Rate_250kbps = 0,
	Rate_1Mbps = 1,
	Rate_2Mbps = 2,
} DataRate_TypeDef;

typedef enum
{
#ifdef SI24R1
	Radio_7dBm = 0x07,
	Radio_4dBm = 0x06,
	Radio_3dBm = 0x05,
	Radio_1dBm = 0x04,
	Radio_0dBm = 0x03,
	Radio_n4dBm = 0x02,
	Radio_n6dBm = 0x01,
	Radio_n12dBm = 0x00,
#else
	Radio_0dBm = 0x06,
	Radio_n6dBm = 0x04,
	Radio_n12dBm = 0x02,
	Radio_n18dBm = 0x00,
#endif
} RadioPower_TypeDef;

typedef enum
{
	RX_PIPE_0 = 0,
	RX_PIPE_1 = 1,
	RX_PIPE_2 = 2,
	RX_PIPE_3 = 3,
	RX_PIPE_4 = 4,
	RX_PIPE_5 = 5,
} RxPipe_TypeDef;

typedef struct
{
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef * CSN_GPIO_Port;
	uint16_t CSN_GPIO_Pin;
	GPIO_TypeDef * CE_GPIO_Port;
	uint16_t CE_GPIO_Pin;
} NRF24L01_TypeDef;
