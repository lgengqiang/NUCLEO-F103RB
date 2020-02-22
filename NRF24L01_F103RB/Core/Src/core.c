/*
 * core.c
 *
 *  Created on: Feb 17, 2020
 *      Author: lgengqiang
 */

#include "core.h"
#include "main.h"

#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "printf.h"

#include "nrf24l01.h"

/* -------------------------------------------------- */

extern SPI_HandleTypeDef hspi1;

NRF24L01_TypeDef nrf24l01 = {
		.hspi = &hspi1,
		.CSN_GPIO_Port = NRF24L01_CSN_GPIO_Port,
		.CSN_GPIO_Pin = NRF24L01_CSN_Pin,
		.CE_GPIO_Port = NRF24L01_CE_GPIO_Port,
		.CE_GPIO_Pin = NRF24L01_CE_Pin};

uint32_t _LD2Tick;

bool isTxMode = true;
uint8_t data[32] = {0};
bool isPowerUp = true;
uint32_t _TxTick;
uint8_t length;

/* -------------------------------------------------- */

void Startup(void)
{
	isTxMode = (HAL_GPIO_ReadPin(MODE_SELECT_GPIO_Port, MODE_SELECT_Pin) == GPIO_PIN_SET)? true: false;

	printf("nRF24L01 Demonstration for STM32F103RB (STM32CubeIDE_1.2.0)\r\n");
	printf("  SystemCoreClock = %d MHz\r\n", (uint16_t)(SystemCoreClock / 1000000));
	if (isTxMode)
		printf("Transmitter\r\n");
	else
		printf("Receiver\r\n");

	// Ticks
	_LD2Tick = HAL_GetTick();
	if (isTxMode)
		_TxTick = HAL_GetTick();

	// nRF24L01
	uint8_t address[5] = {"0Node"};
	uint8_t address1[5] = {"1Node"};
	uint8_t channel = 115;

	NRF24L01_CSN_Release(&nrf24l01);
	NRF24L01_CE_Low(&nrf24l01);

	NRF24L01_Init(&nrf24l01);
	NRF24L01_SetChannel(&nrf24l01, channel);
#ifdef SI24R1
	NRF24L01_SetRadio(&nrf24l01, Rate_1Mbps, Radio_7dBm);
#else
	NRF24L01_SetRadio(&nrf24l01, Rate_1Mbps, Radio_0dBm, true);
#endif

	if (isTxMode)
	{
		/* NRF24L01 TX Mode */
		NRF24L01_TxMode(&nrf24l01, address);
	}
	else
	{
		/* NRF24L01 RX Mode */

		// RX Pipe 0 Configuration
		NRF24L01_RxMode(&nrf24l01, address);

		// RX Pipe 1 Configuration
		NRF24L01_MultiRxMode(&nrf24l01, RX_PIPE_1, address1);
	}

	NRF24L01_PowerUp(&nrf24l01);
	NRF24L01_CE_High(&nrf24l01);
}

void Loop(void)
{
	if (HAL_GetTick() - _LD2Tick >= 500)
	{
		/* Toggle LD2 every 500ms */
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		_LD2Tick = HAL_GetTick();
	}

	if (isTxMode)
	{
		if (HAL_GetTick() - _TxTick >= 2000 && isPowerUp == true)
		{
			_TxTick = HAL_GetTick();

			// Prepare TX data
			uint32_t curTick = HAL_GetTick();
			data[0] = curTick >> 24;
			data[1] = curTick >> 16;
			data[2] = curTick >> 8;
			data[3] = curTick & 0xFF;
			printf("[Transmitter] Data: Tick=%ld  [0]=%02Xh  [1]=%02Xh  [2]=%02Xh  [3]=%02Xh.\r\n", curTick, data[0], data[1], data[2], data[3]);

			TXResult_TypeDef result = NRF24L01_TxPackage(&nrf24l01, data, 32);

			switch (result)
			{
			case TX_OK: // OK
				printf("[Transmitter] Sent Successfully.\r\n");
				break;
			case TX_MAX_RETRY: // Max retry
				printf("[Transmitter] Sent Max Retry.\r\n");
				break;
			case TX_TIMEOUT: //Time out
				printf("[Transmitter] Sent Time Out.\r\n");
				break;
			default: // Error
				printf("[Transmitter] Sent Error.\r\n");
				break;
			}
		}
	}
	else
	{
		uint8_t pipe = NRF24L01_RxPackage(&nrf24l01, data, &length);
		if (pipe != 0xFF)
		{
			printf("[Receiver] Received, Pipe %d, Length:%d. \r\n", pipe, length);
			for (uint8_t i = 0; i < 32; i++)
			{
				printf("%02Xh", data[i]);
				if((i + 1) % 8 == 0)
					printf("\r\n");
				else
					printf(" ");
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (isTxMode && GPIO_Pin == GPIO_PIN_13)
	{
		isPowerUp = !isPowerUp;

		NRF24L01_FlushRx(&nrf24l01);
		NRF24L01_FlushTx(&nrf24l01);

		if (isPowerUp)
			NRF24L01_PowerUp(&nrf24l01);
		else
			NRF24L01_PowerDown(&nrf24l01);

		printf("[Transmitter] Switch to %s.\r\n", isPowerUp? "Power Up Mode": "Power Down Mode");
	}
}
