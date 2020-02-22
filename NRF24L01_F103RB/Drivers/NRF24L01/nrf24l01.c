/*
 * nrf24l01.c
 *
 *  Created on: Feb 16, 2020
 *      Author: lgengqiang
 */

#include "nrf24l01.h"

#define _SPI_TIMEOUT	0x1000
#define _TX_TIMEOUT		0xFFFF
#define _ADDRESS_WDTH	5
#define _PAYLOAD_SZE	32

/* -------------------------------------------------- */

static uint8_t ReadRegister(NRF24L01_TypeDef* nrf24l01, uint8_t reg);
//static void ReadMultiRegister(NRF24L01_TypeDef* nrf24l01, uint8_t reg, uint8_t* data, uint8_t length);
static uint8_t WriteRegister(NRF24L01_TypeDef* nrf24l01, uint8_t reg, uint8_t data);
static uint8_t WriteMultiRegister(NRF24L01_TypeDef* nrf24l01, uint8_t reg, uint8_t* data, uint8_t length);
static uint8_t ReadRxPayload(NRF24L01_TypeDef* nrf24l01, uint8_t* data, uint8_t length);
static uint8_t WriteTxPayload(NRF24L01_TypeDef* nrf24l01, uint8_t* data, uint8_t length);

/* -------------------------------------------------- */

void NRF24L01_CSN_Select(NRF24L01_TypeDef* nrf24l01)
{
	HAL_GPIO_WritePin(nrf24l01->CSN_GPIO_Port, nrf24l01->CSN_GPIO_Pin, GPIO_PIN_RESET);
}

void NRF24L01_CSN_Release(NRF24L01_TypeDef* nrf24l01)
{
	HAL_GPIO_WritePin(nrf24l01->CSN_GPIO_Port, nrf24l01->CSN_GPIO_Pin, GPIO_PIN_SET);
}

void NRF24L01_CE_High(NRF24L01_TypeDef* nrf24l01)
{
	HAL_GPIO_WritePin(nrf24l01->CE_GPIO_Port, nrf24l01->CE_GPIO_Pin, GPIO_PIN_SET);
}

void NRF24L01_CE_Low(NRF24L01_TypeDef* nrf24l01)
{
	HAL_GPIO_WritePin(nrf24l01->CE_GPIO_Port, nrf24l01->CE_GPIO_Pin, GPIO_PIN_RESET);
}

/* -------------------------------------------------- */

static uint8_t _SPI_RW(NRF24L01_TypeDef* nrf24l01, uint8_t data)
{
	uint8_t result;
	HAL_SPI_TransmitReceive(nrf24l01->hspi, &data, &result, 1, _SPI_TIMEOUT);
	return result;
}

/* -------------------------------------------------- */

uint8_t ReadRegister(NRF24L01_TypeDef* nrf24l01, uint8_t reg)
{
	uint8_t result = 0;

	NRF24L01_CSN_Select(nrf24l01);

	_SPI_RW(nrf24l01, reg & NRF24L01_MASK_RW_REGISTER);
	result = _SPI_RW(nrf24l01, result);

	NRF24L01_CSN_Release(nrf24l01);

	return result;
}

uint8_t WriteRegister(NRF24L01_TypeDef* nrf24l01, uint8_t reg, uint8_t data)
{
	uint8_t result = 0;

	NRF24L01_CSN_Select(nrf24l01);

	result = _SPI_RW(nrf24l01, NRF24L01_COMMAND_W_REGISTER | (reg & NRF24L01_MASK_RW_REGISTER));
	_SPI_RW(nrf24l01, data);

	NRF24L01_CSN_Release(nrf24l01);

	return result;
}

uint8_t WriteMultiRegister(NRF24L01_TypeDef* nrf24l01, uint8_t reg, uint8_t* data, uint8_t length)
{
	uint8_t result = 0;

	NRF24L01_CSN_Select(nrf24l01);

	result = _SPI_RW(nrf24l01, NRF24L01_COMMAND_W_REGISTER | (reg & NRF24L01_MASK_RW_REGISTER));
	while (length--)
	{
		_SPI_RW(nrf24l01, *data++);
	}

	NRF24L01_CSN_Release(nrf24l01);

	return result;
}

uint8_t ReadRxPayload(NRF24L01_TypeDef* nrf24l01, uint8_t* data, uint8_t length)
{
	uint8_t status;

	NRF24L01_CSN_Select(nrf24l01);

	status = _SPI_RW(nrf24l01, NRF24L01_COMMAND_R_RX_PAYLOAD);
	while (length--)
	{
		*data++ = _SPI_RW(nrf24l01, NRF24L01_COMMAND_NOP);
	}

	NRF24L01_CSN_Release(nrf24l01);

	return status;
}

uint8_t WriteTxPayload(NRF24L01_TypeDef* nrf24l01, uint8_t* data, uint8_t length)
{
	uint8_t status;
	uint8_t blank = 0;

	if (length > _PAYLOAD_SZE)
		length = _PAYLOAD_SZE;

	if (length < _PAYLOAD_SZE)
		blank = _PAYLOAD_SZE - length;

	NRF24L01_CSN_Select(nrf24l01);

	// TX Payload:
	//   W_TX_PAYLOAD: Write TX-payload
	//   W_TX_PAYLOAD_NO_ACK: AUTOACK Disable
	status = _SPI_RW(nrf24l01, NRF24L01_COMMAND_W_TX_PAYLOAD);
	while (length--)
	{
		_SPI_RW(nrf24l01, *data++);
	}

	// Write 0x00 if length less than 32 bytes
	while (blank--)
	{
		_SPI_RW(nrf24l01, 0x00);
	}

	NRF24L01_CSN_Release(nrf24l01);

	return status;
}

void NRF24L01_Init(NRF24L01_TypeDef* nrf24l01)
{
	WriteRegister(nrf24l01, NRF24L01_REG_CONFIG, MASK_EN_CRC | MASK_CRCO); // Enable CRC, 2 bytes
	WriteRegister(nrf24l01, NRF24L01_REG_EN_AA, MASK_ENAA_ALL); // Auto Acknowledgment Enable for All Pipe
	WriteRegister(nrf24l01, NRF24L01_REG_EN_RXADDR, 0x00); // All RX Pipe Disable
	WriteRegister(nrf24l01, NRF24L01_REG_SETUP_AW, MASK_AW_5_BYTES); // Address: 5 bytes
	WriteRegister(nrf24l01, NRF24L01_REG_SETUP_RETR, 0x0F); // ARD: 250us; ARC: 15
	WriteRegister(nrf24l01, NRF24L01_REG_RF_CH, 50); // 2450MHz
#ifdef SI24R1
	// Si24R1
	WriteRegister(nrf24l01, NRF24L01_REG_RF_SETUP, (uint8_t)Radio_7dBm); // 1Mbps, 7dBm
#else
	// NORDIC nRF24L01
	WriteRegister(nrf24l01, NRF24L01_REG_RF_SETUP, (uint8_t)Radio_0dBm); // 1Mbps, 0dBm
#endif
	WriteRegister(nrf24l01, NRF24L01_REG_STATUS, MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT);
	WriteRegister(nrf24l01, NRF24L01_REG_RX_PW_P0, 0);
	WriteRegister(nrf24l01, NRF24L01_REG_RX_PW_P1, 0);
	WriteRegister(nrf24l01, NRF24L01_REG_RX_PW_P2, 0);
	WriteRegister(nrf24l01, NRF24L01_REG_RX_PW_P3, 0);
	WriteRegister(nrf24l01, NRF24L01_REG_RX_PW_P4, 0);
	WriteRegister(nrf24l01, NRF24L01_REG_RX_PW_P5, 0);
	WriteRegister(nrf24l01, NRF24L01_REG_DYNPD, 0x00);
	WriteRegister(nrf24l01, NRF24L01_REG_FEATURE, 0x00);

	NRF24L01_FlushRx(nrf24l01);
	NRF24L01_FlushTx(nrf24l01);
}

void NRF24L01_PowerUp(NRF24L01_TypeDef* nrf24l01)
{
	uint8_t config = ReadRegister(nrf24l01, NRF24L01_REG_CONFIG);
	WriteRegister(nrf24l01, NRF24L01_REG_CONFIG, config | MASK_PWR_UP);
}

void NRF24L01_PowerDown(NRF24L01_TypeDef* nrf24l01)
{
	uint8_t config = ReadRegister(nrf24l01, NRF24L01_REG_CONFIG);
	WriteRegister(nrf24l01, NRF24L01_REG_CONFIG, config & ~MASK_PWR_UP);
}

void NRF24L01_RxMode(NRF24L01_TypeDef* nrf24l01, uint8_t* address)
{
	WriteMultiRegister(nrf24l01, NRF24L01_REG_RX_ADDR_P0, address, _ADDRESS_WDTH); // RX_ADDR_P0
	WriteRegister(nrf24l01, NRF24L01_REG_RX_PW_P0, _PAYLOAD_SZE); // RX_PW_P0
	WriteRegister(nrf24l01, NRF24L01_REG_EN_AA, MASK_ENAA_P0); // EN_AA
	WriteRegister(nrf24l01, NRF24L01_REG_EN_RXADDR, MASK_ERX_P0); // EN_RXADDR

	uint8_t config = ReadRegister(nrf24l01, NRF24L01_REG_CONFIG);
	WriteRegister(nrf24l01, NRF24L01_REG_CONFIG, config | MASK_PRIM_RX); // CONFIG

	NRF24L01_ClearIRQFlags(nrf24l01); // Clear IRQ Flags
}

void NRF24L01_MultiRxMode(NRF24L01_TypeDef* nrf24l01, RxPipe_TypeDef pipe, uint8_t* address)
{
	// Register: RX_ADDR_Px
	WriteMultiRegister(nrf24l01, NRF24L01_REG_RX_ADDR_P0 + (uint8_t)pipe, address, _ADDRESS_WDTH);

	// Register: RX_PW_Px
	WriteRegister(nrf24l01, NRF24L01_REG_RX_PW_P0 + (uint8_t)pipe, _PAYLOAD_SZE);

	// Register: EN_AA
	uint8_t enAA = ReadRegister(nrf24l01, NRF24L01_REG_EN_AA);
	WriteRegister(nrf24l01, NRF24L01_REG_EN_AA, enAA | (0x01 << (pipe)));

	// Register: EN_RXADDR
	uint8_t enRxAddr = ReadRegister(nrf24l01, NRF24L01_REG_EN_RXADDR);
	WriteRegister(nrf24l01, NRF24L01_REG_EN_RXADDR, enRxAddr | (0x01 << (pipe)));

	// Register: CONFIG
	uint8_t config = ReadRegister(nrf24l01, NRF24L01_REG_CONFIG);
	WriteRegister(nrf24l01, NRF24L01_REG_CONFIG, config | MASK_PRIM_RX);

	// Clear IRQ Flags
	NRF24L01_ClearIRQFlags(nrf24l01);
}

void NRF24L01_TxMode(NRF24L01_TypeDef* nrf24l01, uint8_t* address)
{
	WriteMultiRegister(nrf24l01, NRF24L01_REG_TX_ADDR, address, _ADDRESS_WDTH);
	WriteMultiRegister(nrf24l01, NRF24L01_REG_RX_ADDR_P0, address, _ADDRESS_WDTH);
	WriteRegister(nrf24l01, NRF24L01_REG_EN_AA, 0x01);
	WriteRegister(nrf24l01, NRF24L01_REG_EN_RXADDR, 0x01);

	NRF24L01_ClearIRQFlags(nrf24l01);
}

void NRF24L01_ClearIRQFlags(NRF24L01_TypeDef* nrf24l01)
{
	WriteRegister(nrf24l01, NRF24L01_REG_STATUS, MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT);
}

void NRF24L01_SetChannel(NRF24L01_TypeDef* nrf24l01, uint8_t channel)
{
	WriteRegister(nrf24l01, NRF24L01_REG_RF_CH, channel);
}

#ifdef SI24R1
void NRF24L01_SetRadio(NRF24L01_TypeDef* nrf24l01, DataRate_TypeDef dataRate, RadioPower_TypeDef radioPower)
#else
void NRF24L01_SetRadio(NRF24L01_TypeDef* nrf24l01, DataRate_TypeDef dataRate, RadioPower_TypeDef radioPower, bool setupLNA)
#endif
{
	uint8_t data = 0;
	switch (dataRate)
	{
	case Rate_2Mbps:
		data |= MASK_RF_DR_HIGH;
		break;
	case Rate_250kbps:
		data |= MASK_RF_DR_LOW;
		break;
	default:
		break;
	}

#ifdef SI24R1
	data |= ((uint8_t)radioPower) & 0x07;
#else
	data |= ((uint8_t)radioPower) & 0x06;
#endif

#ifndef SI24R1
	if (setupLNA)
		data |= MASK_RF_LNA_HCURR;
#endif

	WriteRegister(nrf24l01, NRF24L01_REG_RF_SETUP, data);
}

void NRF24L01_FlushRx(NRF24L01_TypeDef* nrf24l01)
{
	// Command: FLUSH_RX
	WriteRegister(nrf24l01, NRF24L01_COMMAND_FLUSH_RX, NRF24L01_COMMAND_NOP);
}

void NRF24L01_FlushTx(NRF24L01_TypeDef* nrf24l01)
{
	// Command: FLUSH_TX
	WriteRegister(nrf24l01, NRF24L01_COMMAND_FLUSH_TX, NRF24L01_COMMAND_NOP);
}

uint8_t NRF24L01_RxPackage(NRF24L01_TypeDef* nrf24l01, uint8_t* data, uint8_t* length)
{
	uint8_t status;

	//status = ReadRegister(nrf24l01, NRF24L01_REG_STATUS);
	//WriteRegister(nrf24l01, NRF24L01_REG_STATUS, status);
	//if (status & MASK_RX_DR)
	uint8_t fifo = ReadRegister(nrf24l01, NRF24L01_REG_FIFO_STATUS);
	if (!(fifo & MASK_FIFO_RX_EMPTY))
	{
		status = ReadRegister(nrf24l01, NRF24L01_REG_STATUS);
		uint8_t pipe = (status & 0x0E) >> 1;
		if (pipe > 5)
			return 0xFF;

		*length = ReadRegister(nrf24l01, NRF24L01_REG_RX_PW_P0 + pipe); // NRF24L01_REG_RX_PW_Px
		ReadRxPayload(nrf24l01, data, *length);

		// Clear IQR Flags
		NRF24L01_ClearIRQFlags(nrf24l01);
		return pipe;
	}

	return 0xFF;
}

TXResult_TypeDef NRF24L01_TxPackage(NRF24L01_TypeDef* nrf24l01, uint8_t* data, uint8_t length)
{
	__IO uint32_t wait = _TX_TIMEOUT;
	uint8_t status;

	NRF24L01_CE_Low(nrf24l01);

	WriteTxPayload(nrf24l01, data, length);

	NRF24L01_CE_High(nrf24l01);

#ifdef _NRF24L01_IRQ_REG
	//// Wait until: TX_DS = 1 or MAX_RT = 1
	//while (!(ReadRegister(nrf24l01, NRF24L01_REG_STATUS) & (MASK_TX_DS | MASK_MAX_RT)));
	while (wait--)
	{
		if (ReadRegister(nrf24l01, NRF24L01_REG_STATUS) & (MASK_TX_DS | MASK_MAX_RT))
			break;
	}
#else
	// Wait until IRQ is Low. Interrupt Active Low
	while (HAL_GPIO_ReadPin(NRF24L01_IRQ_GPIO_Port, NRF24L01_IRQ_Pin) == GPIO_PIN_SET);
#endif

	NRF24L01_CE_Low(nrf24l01);

	if (wait == 0)
	{
		NRF24L01_FlushTx(nrf24l01);
		return TX_TIMEOUT;
	}

	status = ReadRegister(nrf24l01, NRF24L01_REG_STATUS);

	// Clear IQR Flags
	NRF24L01_ClearIRQFlags(nrf24l01);

	if (status & MASK_TX_DS)
		return TX_OK;

	if (status & MASK_MAX_RT)
	{
		NRF24L01_FlushTx(nrf24l01);
		return TX_MAX_RETRY;
	}

	NRF24L01_FlushTx(nrf24l01);
	return TX_ERROR;
}
