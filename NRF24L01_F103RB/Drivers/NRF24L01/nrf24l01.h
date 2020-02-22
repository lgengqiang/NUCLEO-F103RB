/*
 * nrf24l01.h
 *
 *  Created on: Feb 16, 2020
 *      Author: lgengqiang
 */

#include "nrf24l01_def.h"

void NRF24L01_CSN_Select(NRF24L01_TypeDef* nrf24l01);
void NRF24L01_CSN_Release(NRF24L01_TypeDef* nrf24l01);
void NRF24L01_CE_High(NRF24L01_TypeDef* nrf24l01);
void NRF24L01_CE_Low(NRF24L01_TypeDef* nrf24l01);

void NRF24L01_Init(NRF24L01_TypeDef* nrf24l01);
void NRF24L01_PowerUp(NRF24L01_TypeDef* nrf24l01);
void NRF24L01_PowerDown(NRF24L01_TypeDef* nrf24l01);
void NRF24L01_SetChannel(NRF24L01_TypeDef* nrf24l01, uint8_t channel);
#ifdef SI24R1
void NRF24L01_SetRadio(NRF24L01_TypeDef* nrf24l01, DataRate_TypeDef dataRate, RadioPower_TypeDef radioPower);
#else
void NRF24L01_SetRadio(NRF24L01_TypeDef* nrf24l01, DataRate_TypeDef dataRate, RadioPower_TypeDef radioPower, bool setupLNA);
#endif

void NRF24L01_ClearIRQFlags(NRF24L01_TypeDef* nrf24l01);
void NRF24L01_FlushRx(NRF24L01_TypeDef* nrf24l01);
void NRF24L01_FlushTx(NRF24L01_TypeDef* nrf24l01);

void NRF24L01_RxMode(NRF24L01_TypeDef* nrf24l01, uint8_t* address);
void NRF24L01_MultiRxMode(NRF24L01_TypeDef* nrf24l01, RxPipe_TypeDef pipe, uint8_t* address);
void NRF24L01_TxMode(NRF24L01_TypeDef* nrf24l01, uint8_t* address);

uint8_t NRF24L01_RxPackage(NRF24L01_TypeDef* nrf24l01, uint8_t* data, uint8_t* length);
TXResult_TypeDef NRF24L01_TxPackage(NRF24L01_TypeDef* nrf24l01, uint8_t* data, uint8_t length);
