/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 08-Sep-2021
 *      Author: 007ma
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct  /*Configuration structure*/
{
	uint8_t GPIO_Pinnumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

typedef struct /*handle structure*/
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

/* GPIO possible pin numbers */

#define GPIO_PIN_N0_0			0
#define GPIO_PIN_N0_1			1
#define GPIO_PIN_N0_2			2
#define GPIO_PIN_N0_3			3
#define GPIO_PIN_N0_4			4
#define GPIO_PIN_N0_5			5
#define GPIO_PIN_N0_6			6
#define GPIO_PIN_N0_7			7
#define GPIO_PIN_N0_8			8
#define GPIO_PIN_N0_9			9
#define GPIO_PIN_N0_10			10
#define GPIO_PIN_N0_11			11
#define GPIO_PIN_N0_12			12
#define GPIO_PIN_N0_13			13
#define GPIO_PIN_N0_14			14
#define GPIO_PIN_N0_15			15


/* GPIO possible modes */
#define GPIO_MODE_IN 			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTER			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

/* GPIO possible output modes */

#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

/* GPIO possible output speed */

#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/* GPIO possible pull up and pull down config */

#define GPIO_PIN_NOPUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2
/*Api supported by this driver*/

/* for clock control either to enable or disable */
void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDI);

/* Init and Deinit
 *
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle); /* for initialization of port and its register modes */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
void GPIO_IRQ_Interuppt_Config(uint8_t IRQNumber, uint8_t EnorDI);
void GPIO_IRQ_Interuppt_priority_config(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
