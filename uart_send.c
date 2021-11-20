/*
 * uart_send.c
 *
 *  Created on: 16-Nov-2021
 *      Author: 007ma
 */

#include "stm32f407xx.h"
#include <string.h>

USART_Handle_t usart2_config;
char buffer[] = "Hello from my side";

void GPIO_Uart();
void USART_In();

int main()
{
	while(1)
	{


	GPIO_Uart();
	USART_In();

	USART_PeripheralClockEnable(USART2, ENABLE);
	USART_SendData(&usart2_config,(uint8_t*) buffer, strlen(buffer));



}
	return 0;
}
void GPIO_Uart()
{
	GPIO_Handle_t Gpio_usart;
	Gpio_usart.pGPIOx = GPIOA;
	Gpio_usart.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_2 | GPIO_PIN_N0_3;
	Gpio_usart.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTER;
	Gpio_usart.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	Gpio_usart.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpio_usart.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	Gpio_usart.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&Gpio_usart);
}

void USART_In()
{

	usart2_config.pUSARTx = USART2;
	usart2_config.USART_Config.USART_Baudrate = USART_STD_BAUD_115200;
	usart2_config.USART_Config.USART_WordLen = USART_WORD_LEN8;
	usart2_config.USART_Config.USART_Mode = USART_MODE_TX;
	usart2_config.USART_Config.USART_HwFlowControl = HARDWARE_FLOWCONTROL_NONE;
	usart2_config.USART_Config.USART_NoOfStopBits = USART_STOPBIT_1;
	usart2_config.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART_Init(&usart2_config);
}
