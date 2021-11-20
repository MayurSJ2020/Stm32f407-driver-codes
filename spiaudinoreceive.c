/*
 * spiaudinoreceive.c
 *
 *  Created on: 20-Sep-2021
 *      Author: 007ma
 */
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
#include <stdio.h>



#define CMD_LED_CTRL	0x50  // TO TURN ON LED
#define CMD_SEN_READ	0x51  // TO READ SEN DATA IT WILL SEND 1 BYTE DATA
#define CMD_LED_READ	0x52  // TO READ LED BIT DATA SET OR NOT
#define CMD_PRINT		0x53  // TO SEND DATA FROM SLAVE TO MASTER
#define CMD_ID_READ		0x54  // SLAVE SENDS ITS SLAVE ID

#define LED_ON			1
#define LED_OFF
// AUDINO ANALOG PINS
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4
//ARDUINO LED PIN
#define LED_PIN 		9

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
	SPI2_CONFIG.SPI_Config.SPI_SSM = SPI_SSM_DI; //hardware slave management

	SPI_Init(&SPI2_CONFIG);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		return 1;
	}
	return 0;

}

int main()
{

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read = 0xff;


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

	// 1.CMD_LED_CTRL <PIN NO>  <VALUE>
	uint8_t command_code = CMD_LED_CTRL;
	SPI_SendData(SPI2,&command_code, 1);

	// after sending one byte of data there will be data in wic is sent by slave so rxen will be set to clear tis you must read it dummy read
	SPI_ReceiveData(SPI2, &dummy_read, 1);
	uint8_t ackbyte;
	uint8_t args[2];

	// ack data will be available in shift register and if we want to receive that data we have to send dummy bytes to receive data
	SPI_SendData(SPI2, &dummy_write, 1);

	// now receive data
	SPI_ReceiveData(SPI2, &ackbyte, 1);

	//Check if you receive ack or nack
	if(SPI_VerifyResponse(ackbyte))
	{
		args[0] = LED_PIN;
		args[1] = LED_ON;
		SPI_SendData(SPI2, args, 2);
	}

		// 2.send command to read the value from the sensor
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_N0_0));

		delay();
		command_code = CMD_SEN_READ;
		SPI_SendData(SPI2, &command_code, 1);
		//dummy read to clear rxne
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// dummy write to receive ack from sift register
		SPI_SendData(SPI2, &dummy_write, 1);
		// now read ack wic will be available in data register
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		// now if ack byte was sent by slave you can proceed wit transmission
		if(SPI_VerifyResponse(ackbyte))
			{	// wic pin to read
				args[0] = ANALOG_PIN0;
				SPI_SendData(SPI2, args, 1);

				SPI_ReceiveData(SPI2, &dummy_read, 1);
				delay(); //adc will require somw time to convert analog to digital value
				SPI_SendData(SPI2, &dummy_write, 1);

				uint8_t analog_read;
				SPI_ReceiveData(SPI2, &analog_read, 1);
				//printf("COMMAND_READ_LED %d\n",analog_read);
	}
		// to read led pin status form led pin
				while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_N0_0) );

				//to avoid button de-bouncing related issues 200ms of delay
				delay();

				command_code = CMD_LED_READ;

				//send command
				SPI_SendData(SPI2,&command_code,1);

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1);

				//Send some dummy byte to fetch the response from the slave
				SPI_SendData(SPI2,&dummy_write,1);

				//read the ack byte received
				SPI_ReceiveData(SPI2,&ackbyte,1);

				if( SPI_VerifyResponse(ackbyte))
				{
					args[0] = LED_PIN;

					//send arguments
					SPI_SendData(SPI2,args,1); //sending one byte of

					//do dummy read to clear off the RXNE
					SPI_ReceiveData(SPI2,&dummy_read,1);

					//insert some delay so that slave can ready with the data
					delay();

					//Send some dummy bits (1 byte) fetch the response from the slave
					SPI_SendData(SPI2,&dummy_write,1);

					uint8_t led_status;
					SPI_ReceiveData(SPI2,&led_status,1);
					//printf("COMMAND_READ_LED %d\n",led_status);

				}

				//4. CMD_PRINT 		<len(2)>  <message(len) >

				//wait till button is pressed
				while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_N0_0) );

				//to avoid button de-bouncing related issues 200ms of delay
				delay();

				command_code = CMD_PRINT;

				//send command
				SPI_SendData(SPI2,&command_code,1);

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1);

				//Send some dummy byte to fetch the response from the slave
				SPI_SendData(SPI2,&dummy_write,1);

				//read the ack byte received
				SPI_ReceiveData(SPI2,&ackbyte,1);

				uint8_t message[] = "Hello ! How are you ??";
				if( SPI_VerifyResponse(ackbyte))
				{
					args[0] = strlen((char*)message);

					//send arguments
					SPI_SendData(SPI2,args,1); //sending length

					//do dummy read to clear off the RXNE
					SPI_ReceiveData(SPI2,&dummy_read,1);

					delay();

					//send message
					for(int i = 0 ; i < args[0] ; i++){
						SPI_SendData(SPI2,&message[i],1);
						SPI_ReceiveData(SPI2,&dummy_read,1);
					}

					//printf("COMMAND_PRINT Executed \n");

				}

				//5. CMD_ID_READ
				//wait till button is pressed
				while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_N0_0) );

				//to avoid button de-bouncing related issues 200ms of delay
				delay();

				command_code = CMD_ID_READ;

				//send command
				SPI_SendData(SPI2,&command_code,1);

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1);

				//Send some dummy byte to fetch the response from the slave
				SPI_SendData(SPI2,&dummy_write,1);

				//read the ack byte received
				SPI_ReceiveData(SPI2,&ackbyte,1);

				uint8_t id[11];
				uint32_t i=0;
				if( SPI_VerifyResponse(ackbyte))
				{
					//read 10 bytes id from the slave
					for(  i = 0 ; i < 10 ; i++)
					{
						//send dummy byte to fetch data from slave
						SPI_SendData(SPI2,&dummy_write,1);
						SPI_ReceiveData(SPI2,&id[i],1);
					}

					id[10] = '\0';

					//printf("COMMAND_ID : %s \n",id);

				}



		// IF SPI BUSY FLAG IS NOT SET WE CAN DISABLE SPI PERI
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		SPI_PeripheralControl(SPI2, DISABLE);


}

	return 0;


}




