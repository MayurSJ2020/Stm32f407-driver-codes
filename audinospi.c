/*
 * audinospi.c
 *
 *  Created on: 20-Sep-2021
 *      Author: 007ma
 *
 * TIS PINS ARE REFERRED FROM DATA SEET OF MCU
 * PB15 ---> SPI2_MOSI
 * PB14 ---> SPI2_MISO
 * PB13 ---> SPI2_SCLK
 * PB12 ---> NSS
 * ALT FUNCTION MODE 5
 */

#include "stm32f407xx.h"
#include <stdint.h>
#include <string.h>

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName);

void delay(void)
{
	for(uint32_t i=0;i<500000;i++);
}

void SPI2_GPIO_INIT(void)
{
	//This pin is used to behave gpio pin as spi pins
	// INITIALIZING PIN NO 13 OF PORT B FOR SCLK
	GPIO_Handle_t SPIInitiali;

	SPIInitiali.pGPIOx = GPIOB;
	SPIInitiali.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTER;
	SPIInitiali.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIInitiali.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIInitiali.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;
	SPIInitiali.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	SPIInitiali.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_13;
	GPIO_Init(&SPIInitiali);

	// INITIALIZING PIN NO 15 OF PORT B FOR MOSI

	SPIInitiali.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_15;
	GPIO_Init(&SPIInitiali);

	// INITIALIZING FOR MISO

	//SPIInitiali.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_14;
	//GPIO_Init(&SPIInitiali);

	// INITIALIZING FOR NSS

	SPIInitiali.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_12;
	GPIO_Init(&SPIInitiali);

}
void GPIO_BUTTON_INIT(void)
{
GPIO_Handle_t GpioSwitc;

	GpioSwitc.pGPIOx = GPIOA;
	GpioSwitc.GPIO_PinConfig.GPIO_Pinnumber= GPIO_PIN_N0_0;
	GpioSwitc.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IN;
	GpioSwitc.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	GpioSwitc.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_NOPUPD;

	GPIO_Init(&GpioSwitc);
}

void SPI2_INIT(void)
{
	SPI_Handle_t SPI2_CONFIG;

	SPI2_CONFIG.pSPIx = SPI2;
	SPI2_CONFIG.SPI_Config.SPI_BusConfig = SPI_BUSCNFG_FULLDUPLEX;
	SPI2_CONFIG.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_CONFIG.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //for 2megahertz
	SPI2_CONFIG.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2_CONFIG.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_CONFIG.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_CONFIG.SPI_Config.SPI_SSM = SPI_SSM_DI; //hardware slave managment

	SPI_Init(&SPI2_CONFIG);
}


int main()
{
	char userdata[]= "Hello world";

	SPI2_GPIO_INIT();

	GPIO_BUTTON_INIT();

	SPI2_INIT();

	//When ssoe is enabled it will make nss low when spi spe bit is set and make nss low when spe is not set
	// This is used only in master mode with ssm disabled
	SPI_SSOE_Config(SPI2, ENABLE);

	// THIS MAKES NSS SIGNAL INTERNALLY HIGH AND AVOIDS MODF ERROR
	//SPI_SSICONFIG(SPI2,ENABLE);
	// SSI managment only to be implemented during software slave management
	while(1)
	{
	while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_N0_0));

	delay();// to avoid debouncing

	//now spi is  not active to active you have to set the bit of spe in cr1 register bcz configuration of spi should be done when spi is inactive
	SPI_PeripheralControl(SPI2, ENABLE);

	//SENDING SLAVE LENGT OF DATA WE ARE GOING TO SEND

	uint8_t datalen = strlen(userdata);
	SPI_SendData(SPI2,&datalen,1);


	SPI_SendData(SPI2,(uint8_t*)userdata,strlen(userdata));

	// IF SPI BUSY FLAG IS NOT SET WE CAN DISABLE SPI PERI
	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

	SPI_PeripheralControl(SPI2, DISABLE);


}

	return 0;


}


