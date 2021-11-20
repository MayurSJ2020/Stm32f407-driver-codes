/*
 * 004btnintr.c
 *
 *  Created on: 15-Sep-2021
 *      Author: 007ma
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include <stdint.h>
#include <string.h>

void delay(void)
{
	for(uint32_t i=0;i<500000;i++);
}

int main(void)
{

	while(1)
	{

	GPIO_Handle_t GpioLed,Gpiobutton;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&Gpiobutton,0,sizeof(Gpiobutton)); //clearing entire structure before initialization to avoid garbage value.

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_Pinnumber= GPIO_PIN_N0_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_NOPUPD;



	Gpiobutton.pGPIOx = GPIOD;
	Gpiobutton.GPIO_PinConfig.GPIO_Pinnumber= GPIO_PIN_N0_5;
	Gpiobutton.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IT_RT; // INTERUPPT USING EXTERNAL BUTTON
	Gpiobutton.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	Gpiobutton.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PU;// NORMALLY HIGH BCZ IT IS PULLED UP


	GPIO_PCLKControl(GPIOD, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&Gpiobutton);

//IRQ CONFIGARATION USING API DEVELOPED
	GPIO_IRQ_Interuppt_priority_config(IRQ_NO_EXTI9_5, IRQ_PRIO_15);
	GPIO_IRQ_Interuppt_Config(IRQ_NO_EXTI9_5, ENABLE); //IRQ NO REFERED FROM MANUAL UNDER INTERUPPT TOPIC

// CALLING ISR FROM STARTUP AS IT IS WEAK AND OVERWRITE IT BY CALLING USER DEFINED API FOR IRQ HANDLING
	}

}

void EXTI9_5_IRQHandler(void)
{	 delay(); // delay to avoid button de-bouncing
	 GPIO_IRQHandling(GPIO_PIN_N0_5);
	 GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_N0_12);
}



