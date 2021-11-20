/*
 * 001ledtoggle.c
 *
 *  Created on: 11-Sep-2021
 *      Author: 007ma
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include <stdint.h>

void delay(void)
{
	for(uint32_t i=0;i<500000;i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_Pinnumber= GPIO_PIN_N0_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_NOPUPD;

	GPIO_Handle_t GpioSwitc;

		GpioSwitc.pGPIOx = GPIOA;
		GpioSwitc.GPIO_PinConfig.GPIO_Pinnumber= GPIO_PIN_N0_0;
		GpioSwitc.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IN;
		GpioSwitc.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
		GpioSwitc.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_NOPUPD;


	GPIO_PCLKControl(GPIOD, ENABLE);
	GPIO_PCLKControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioSwitc);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_N0_0) == ENABLE)
		{
			delay();
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_N0_12);

		}else{}



}return 0;}
