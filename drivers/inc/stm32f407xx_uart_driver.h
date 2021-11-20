/*
 * stm32407xx_uart_driver.h
 *
 *  Created on: 26-Oct-2021
 *      Author: 007ma
 */

#ifndef INC_STM32F407XX_UART_DRIVER_H_
#define INC_STM32F407XX_UART_DRIVER_H_

#include "stm32f407xx.h"
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baudrate;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLen;
	uint8_t USART_ParityControl;
	uint8_t USART_HwFlowControl;



}USART_Config_t;
typedef struct
{
	USART_RegDef_t *pUSARTx;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t txstate;
	uint8_t rxstate;
	uint32_t txlen;
	uint32_t rxlen;

	USART_Config_t USART_Config;

}USART_Handle_t;

//busy flags

#define USART_BUSY_IN_TX 	0
#define USART_BUSY_IN_RX 	1
#define USART_FREE			2

//Usart mode types

#define USART_MODE_TX	0
#define USART_MODE_RX	1
#define USART_MODE_TXRX 2

//USART speed modes

#define USART_STD_BAUD_1200		1200
#define USART_STD_BAUD_2400		2400
#define USART_STD_BAUD_9600		9600
#define USART_STD_BAUD_19200	19200
#define USART_STD_BAUD_38400	38400
#define USART_STD_BAUD_57600	57600
#define USART_STD_BAUD_115200	115200
#define USART_STD_BAUD_230400	230400
#define USART_STD_BAUD_460800	460800
#define USART_STD_BAUD_921600	921600
#define USART_STD_BAUD_2M		2000000
#define USART_STD_BAUD_3M		3000000

//usart parity control

#define USART_PARITY_ODD				2
#define USART_PARITY_EVEN				1
#define USART_PARITY_DISABLE			0

//USART word len

#define USART_WORD_LEN8			0
#define USART_WORD_LEN9			1

//USART stopbit

#define USART_STOPBIT_1			0
#define USART_STOPBIT_0_5		1
#define USART_STOPBIT_2			2
#define USART_STOPBIT_1_5		3

//Hardware flow control

#define HARDWARE_FLOWCONTROL_NONE 		0
#define HARDWARE_FLOWCONTROL_CTS		1
#define HARDWARE_FLOWCONTROL_RTS		2
#define HARDWARE_FLOWCONTROL_CTS_RTS	3


//macros for status register for Errors

#define ParityErrorFlag		(1<<0)
#define FramingErFlag		(1<<1)
#define NoisedetErFlag		(1<<2)
#define OverrunErrFlag		(1<<3)

//macros for status register for status

#define idleFlag			(1<<4)
#define RxneFlag			(1<<5)
#define TCcompleteFlag		(1<<6)
#define TXNE_Flag			(1<<7)
#define LBD_Flag			(1<<8)
#define CTS_Flag			(1<<9)


void USART_PeriClockEnable(USART_RegDef_t *pUSARTx,uint8_t EnorDI);


void USART_PeripheralClockEnable(USART_RegDef_t *pUSARTx,uint8_t EnorDI);
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);


uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx,uint8_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx,uint8_t FlagName);
void USART_Init(USART_Handle_t *pUSARTHandle);

void USART_IRQ_Interuppt_Config(uint8_t IRQNumber, uint8_t EnorDI);
void USART_IRQ_Interuppt_priority_config(uint8_t IRQNumber,uint32_t IRQPriority);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

#endif /* INC_STM32F407XX_UART_DRIVER_H_ */
