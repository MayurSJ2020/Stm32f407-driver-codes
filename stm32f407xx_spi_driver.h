/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 17-Sep-2021
 *      Author: 007ma
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct  /*Configuration structure*/
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

typedef struct /*handle structure*/
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
	//below snippet is used to send and receive data trough interrupt
	uint8_t 	 *pTxBuffer;
	uint8_t		 *pRxBuffer;
	uint32_t	 TxLengt;
	uint32_t	 RxLengt;
	uint8_t 	TxState;
	uint8_t 	RxState;

}SPI_Handle_t;

//device modes macros

#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

//macros for bus config

#define SPI_BUSCNFG_FULLDUPLEX		1
#define SPI_BUSCNFG_HALFDUPLEX		2
#define SPI_BUSCNFG_SIM_RXONLY		3

//POSSIBLE EVENTS IN SPI

#define SPI_TX_EVENT_COMPLETE		1
#define SPI_RX_EVENT_COMPLETE		2
#define SPI_EVENT_OVR_ERR			3


//clock speed

#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

//FOR CONFIG LEN OF BITS

#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1

//FOR CPOL

#define SPI_CPOL_HIGH				1
#define SPI_CPOL_LOW				0

//FOR CPHA

#define SPI_CPHA_HIGH				1
#define SPI_CPHA_LOW				0

// FOR SSM

#define SPI_SSM_EN					1
#define SPI_SSM_DI					0

//For intruppt data sending and receiving macros

#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2


#define SPI_TXE_FLAG			(1 << 1)  // SR REGISTER OF SPI BIT FEILD WIC INDICATES WEATER TX BUFFER IS EMPTY OR NOT
#define SPI_RXE_FLAG			(1 << 0)
#define SPI_BSY_FLAG			(1 << 7)



// API for spi peripherals clock

void SPI_PCLK_Config(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

//init and Deint of SPI modes using spi config structure

void SPI_Init(SPI_Handle_t *pSPIhandlex); /* for initialization of port and its register modes */
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//DATA SEND AND RECEIVE FOR SPI

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len); // Tx buffer pointer ten assigned to base address of tx buffer register
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len); // Rx buffer pointer ten assigned to base address of tx buffer register


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len); // Tx buffer pointer ten assigned to base address of tx buffer register
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len);

//IRQ Configuration and ISR handling

void SPI_IRQ_Interuppt_Config(uint8_t IRQNumber, uint8_t EnorDI);
void SPI_IRQ_Interuppt_priority_config(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIhandlex);


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDI);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName);
void SPI_SSICONFIG(SPI_RegDef_t *pSPIx,uint8_t EnorDI);

void SPI_SSOE_Config(SPI_RegDef_t *pSPIx,uint8_t EnorDI);
void SPI_Clear_Ovr_Flag(SPI_RegDef_t *pSPIx);
void SPI_Close_Tr(SPI_Handle_t *pSPIHandle);
void SPI_Close_Rx(SPI_Handle_t *pSPIHandle);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
