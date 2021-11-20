/*
 * spi.c
 *
 *  Created on: 18-Sep-2021
 *      Author: 007ma
 */
/*
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

	//SPIInitiali.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_12;
	//GPIO_Init(&SPIInitiali);

}

void SPI2_INIT(void)
{
	SPI_Handle_t SPI2_CONFIG;

	SPI2_CONFIG.pSPIx = SPI2;
	SPI2_CONFIG.SPI_Config.SPI_BusConfig = SPI_BUSCNFG_FULLDUPLEX;
	SPI2_CONFIG.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_CONFIG.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2_CONFIG.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2_CONFIG.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_CONFIG.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_CONFIG.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2_CONFIG);
}


int main()
{
	char userdata[]= "Hello world";
	SPI2_GPIO_INIT();

	SPI2_INIT();

	// THIS MAKES NSS SIGNAL INTERNALLY HIGH AND AVOIDS MODF ERROR
	SPI_SSICONFIG(SPI2,ENABLE);

	//now spi is  not active to active you have to set the bit of spe in cr1 register bcz configuration of spi should be done when spi is inactive
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2,(uint8_t*)userdata,strlen(userdata));

		while(SPI_GetFlagStatus(SPI2,SPI_BSY_FLAG));

		SPI_PeripheralControl(SPI2, DISABLE);

	return 0;

	while(1);
}
