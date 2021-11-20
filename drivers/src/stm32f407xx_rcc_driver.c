/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: 14-Nov-2021
 *      Author: 007ma
 */

#include"stm32f407xx_rcc_driver.h"

uint16_t Ahb_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t Apb_PreScaler[8] = {2,4,8,16};

uint32_t GetPLLclock()
{
	return 0;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t PCLK1,Sysclk;
	uint8_t Clksrc,temp,Ahbp,Apbp;
	Clksrc = ((RCC->CFGR >>2) & 0X3);

	if(Clksrc == 0)
	{
		Sysclk = 16000000;
	}
	else if(Clksrc == 1)
	{
		Sysclk = 8000000;
	}
	else if(Clksrc == 2)
	{
		Sysclk = GetPLLclock();
	}
	// to find ahbprescalar value
	temp = ((RCC->CFGR >>4) & 0xF);

	if(temp < 8)
	{
		Ahbp=1;
	}
	else
	{
		Ahbp = Ahb_PreScaler[temp-8];
	}
	// to find apbprescalar value
	temp = ((RCC->CFGR >>10) & 0x7);

	if(temp < 4)
	{
		Apbp=1;
	}
	else
	{
		Apbp = Apb_PreScaler[temp-4];
	}

	PCLK1 = (Sysclk/Ahbp)/Apbp;

	return PCLK1;
}

// this calculates pclk for apb2 bus
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t PCLK2,Sysclk;
		uint8_t Clksrc,temp,Ahbp,Apbp;
		Clksrc = ((RCC->CFGR >>2) & 0X3);

		if(Clksrc == 0)
		{
			Sysclk = 16000000;
		}
		else if(Clksrc == 1)
		{
			Sysclk = 8000000;
		}
		else if(Clksrc == 2)
		{
			Sysclk = GetPLLclock();
		}
		// to find ahbprescalar value
		temp = ((RCC->CFGR >>4) & 0xF);

		if(temp < 8)
		{
			Ahbp=1;
		}
		else
		{
			Ahbp = Ahb_PreScaler[temp-8];
		}
		// to find apbprescalar value
		temp = ((RCC->CFGR >>13) & 0x7);

		if(temp < 4)
		{
			Apbp=1;
		}
		else
		{
			Apbp = Apb_PreScaler[temp-4];
		}

		PCLK2 = (Sysclk/Ahbp)/Apbp;

		return PCLK2;
}
