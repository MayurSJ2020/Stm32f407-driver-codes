/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: 08-Sep-2021
 *      Author: 007ma
 */

#include "stm32f407xx_gpio_driver.h"

void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDI)
{
	if(EnorDI == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();

		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}

		}else{
			if(pGPIOx == GPIOA)
					{
						GPIOA_PCLK_DI();

					}else if (pGPIOx == GPIOB)
					{
						GPIOB_PCLK_DI();
					}else if (pGPIOx == GPIOC)
					{
						GPIOC_PCLK_DI();
					}else if (pGPIOx == GPIOD)
					{
						GPIOD_PCLK_DI();
					}else if (pGPIOx == GPIOE)
					{
						GPIOE_PCLK_DI();
					}else if (pGPIOx == GPIOF)
					{
						GPIOF_PCLK_DI();
					}else if (pGPIOx == GPIOG)
					{
						GPIOG_PCLK_DI();
					}else if (pGPIOx == GPIOH)
					{
						GPIOH_PCLK_DI();
					}else if (pGPIOx == GPIOI)
					{
						GPIOI_PCLK_DI();
					}
}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) /* for initialization of port and its register modes */
{
	uint32_t temp=0;

	// ENABLING GPIO CLOCK
	 GPIO_PCLKControl(pGPIOHandle->pGPIOx,ENABLE);


	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// configuration for mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber); // Clearing before setting
		pGPIOHandle->pGPIOx->MODER |= temp; // setting the bit
		temp =0;

		// configuration for speed
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;
		temp=0;

		// configuration for pull up and pull down register
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);
		pGPIOHandle->pGPIOx->PUPDR |= temp;
		temp=0;

		//config of pin output type

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<(pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber));
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);
		pGPIOHandle->pGPIOx->OTYPER |= temp;
		temp=0;

		//config alternate functionality

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTER){
		uint32_t temp1=0;
		uint32_t temp2=0;
		temp1=(pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber)/8;
		temp2=(pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber)%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (temp2 * 4)); // clearing before entering
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4 * temp2));

		}


	}else
	{	//SELECTING WIC MODE OF INTRUPPT NEEDED FALLING RAISING OR BOT
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{	//ENABLING FTSR USING EXTI FTSR REGISTER
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber );
			//DISABLING RTSR BCZ WE ONLY NEED FALLING EDGE TRIGGER
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber );


		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber );
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber );

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{

			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber );
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber );

		}

		//CONFIGURING WIC GPIO PORT PIN NEEDED USING SYSCONFIG EXTICR REGISTER

		uint8_t temp3 = ((pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber)/4);
		uint8_t temp4 = ((pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber)%4);
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp3] |= portcode <<(4*temp4);

		//ENABLING EXTI INTERUPT DELIVERY USING IMR TO NVIC
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber );

	}


}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();

			}else if (pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}else if (pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}else if (pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}else if (pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}else if (pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}

}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value1;
	value1 = (uint16_t)pGPIOx->IDR ;
	return value1;

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
	pGPIOx->ODR |= (1 <<PinNumber);
	}else{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR = Value;

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);

}

void GPIO_IRQ_Interuppt_priority_config(uint8_t IRQNumber,uint32_t IRQPriority)
{
uint8_t iprx = IRQNumber/4; //calculated to know wic IRQ register total tere are 60 registers
uint8_t section = IRQNumber %4; //tere are four IRQ register section in one register
uint8_t shift_value = (8*section) + (8-NO_OF_PR_BIT_IMPLIMENTED);
*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority <<(shift_value)); //nvic priority base address defined in mcu file and if we add one it will move up by one to next register


}

void GPIO_IRQ_Interuppt_Config(uint8_t IRQNumber, uint8_t EnorDI)
{
	if(EnorDI == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISERX register refer user manul of arm cortex M4 processor side registers
			*NVIC_ISER0 |= (1<<IRQNumber);
		}else if(IRQNumber >31 && IRQNumber <64)
		{
			//program ISER1 register refer user manul of arm cortex M4
			*NVIC_ISER1 |= ((1<<IRQNumber)%32);
		}else if(IRQNumber >=64 && IRQNumber <96)
		{
			//program ISER2 register refer user manul of arm cortex M4
			*NVIC_ISER2 |= ((1<<IRQNumber)%64);
		}

	}else{
		//it is used to program ICERX register it is used to clear the interrupt
		if(IRQNumber <= 31)
				{
				*NVIC_ICER0 |= (1<<IRQNumber);

				}else if(IRQNumber >31 && IRQNumber <64)
				{
					*NVIC_ICER1 |= ((1<<IRQNumber)%32);

				}else if(IRQNumber >=64 && IRQNumber <96)
				{
					*NVIC_ICER2 |= ((1<<IRQNumber)%64);

				}

	}
}

void GPIO_IRQHandling(uint8_t PinNumber) //PIN NUMBER OF EXTIX NOT OF TE NVIC
{
	if(EXTI->PR &(1<<PinNumber))
	{
		EXTI -> PR |= (1<<PinNumber);//clearing after interrupt bcz if tis is not cleared interrupt will execute continuosly
	}
}






