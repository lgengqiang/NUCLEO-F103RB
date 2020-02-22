nRF24L01 Library for STM32F103RB(NUCLEO-F103RB)

This library is based on HAL APIs, also support for others STM32 MCUs.

Default is for Si24R1. You can comments "#define SI24R1" in "nrf24l01_def.h" to support nRF24L01.

RX/TX mode is depends on PA1 when starting. PA1 high: TX mode; PA1 low: RX mode. Change the mode need to reset the device.

TX Mode:
	Address: 0Node
	Send data every 2 seconds. The first 4 bytes are from HAL_GetTick(), others are 0x00.

RX Mode:
	Pipe 0 Address: 0Node
	Pipe 1 Address: 1Node
	Support for multi receive pipes. It configures 2 receive addresses: 0Node and 1Node.
	Print the HEX code via USART1.

I tested this library in NUCLEO-F103RB, STM32F103C8 and STM32F030F4.
