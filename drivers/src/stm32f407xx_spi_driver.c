/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 17-Sep-2021
 *      Author: 007ma
 */

#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIhandlex);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIhandlex);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIhandlex);

#define SPI_READY 		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

void SPI_PCLK_Config(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
	if(pSPIx == SPI1)
	{
		SPI1_PCLK_EN();
	}else if(pSPIx == SPI2)
	{
		SPI2_PCLK_EN();
	}else if(pSPIx == SPI3)
	{
		SPI3_PCLK_EN();
	}

	}else{
		if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}

	}
}

//init and Deint of SPI modes using spi config structure

void SPI_Init(SPI_Handle_t *pSPIhandlex) /* for initialization of port and its register modes */
{ uint32_t temp = 0;

// CLOCK ENABLING

SPI_PCLK_Config(pSPIhandlex->pSPIx,ENABLE);

//configuration of device mode

temp |= pSPIhandlex->SPI_Config.SPI_DeviceMode << 2;

// bus configaration

if(pSPIhandlex->SPI_Config.SPI_BusConfig == SPI_BUSCNFG_FULLDUPLEX)
{
	//bidi must be cleared
	temp &= ~(1<<15);

}else if(pSPIhandlex->SPI_Config.SPI_BusConfig == SPI_BUSCNFG_HALFDUPLEX)
{
	//bidi must be set
	temp |= (1<<15);
}else if(pSPIhandlex->SPI_Config.SPI_BusConfig == SPI_BUSCNFG_SIM_RXONLY)
{
	// bidi must be cleared and rx only must be set
	temp &= ~(1<<15);
	temp |= (1<<10);
}

//config for clock speed

temp |= pSPIhandlex->SPI_Config.SPI_SclkSpeed << 3;

// config for dff

temp |= pSPIhandlex->SPI_Config.SPI_DFF << 11;

// config for cpol

temp |= pSPIhandlex->SPI_Config.SPI_CPOL << 1;

//config for cp

temp |= pSPIhandlex->SPI_Config.SPI_CPHA << 0;

// config for ssm

temp |= pSPIhandlex->SPI_Config.SPI_SSM << 9;

pSPIhandlex->pSPIx->CR1 = temp;
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDI)
{
	if(EnorDI == ENABLE)
	{
	pSPIx->CR1 |= 1<<6;

}else
{
	pSPIx->CR1 &= ~(1<<6);
}
}

void SPI_SSICONFIG(SPI_RegDef_t *pSPIx,uint8_t EnorDI)
{
	if(EnorDI == ENABLE)
	{
	pSPIx->CR1 |= 1<<8;

}else
{
	pSPIx->CR1 &= ~(1<<8);
}

}

void SPI_SSOE_Config(SPI_RegDef_t *pSPIx,uint8_t EnorDI)
{
	if(EnorDI == ENABLE)
		{
		pSPIx->CR2 |= 1<<2;

	}else
	{
		pSPIx->CR2 &= ~(1<<2);
	}



}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		{
		SPI1_REG_RESET();
		}else if(pSPIx == SPI2)
		{
			SPI2_REG_RESET();
		}else if(pSPIx == SPI3)
		{
			SPI3_REG_RESET();

		}

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
	if(pSPIx->SR &FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

//DATA SEND AND RECEIVE FOR SPI

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len) // Tx buffer pointer ten assigned to base address of tx buffer register
{
	while(Len > 0)
	{
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		if((pSPIx->CR1 & (1<<11)))
		{
			//if this is true ten it is 16 bit data
			// now load data into data register of SPI
			pSPIx->DR = *((uint16_t*)pTxBuffer); // basically ptxbuffer pointer is of 8 bit so to convert it to 16 we should type cast it to 16 bit pointer
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//Eight bit data format
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len) // Rx buffer pointer ten assigned to base address of tx buffer register
{
	while(Len > 0)
		{
			while(SPI_GetFlagStatus(pSPIx,SPI_RXE_FLAG) == FLAG_RESET);

			if((pSPIx->CR1 & (1<<11)))
			{
				//if this is true ten it is 16 bit data
				// now load data from dr to rx buffer
				*((uint16_t*)pRxBuffer) = pSPIx->DR; // basically ptxbuffer pointer is of 8 bit so to convert it to 16 we should type cast it to 16 bit pointer
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//Eight bit data format
				*pRxBuffer = pSPIx->DR ;
				Len--;
				pRxBuffer++;
			}
		}

}
//IRQ Configuration and ISR handling

void SPI_IRQ_Interuppt_Config(uint8_t IRQNumber, uint8_t EnorDI)
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
void SPI_IRQ_Interuppt_priority_config(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4; //calculated to know wic IRQ register total tere are 60 registers
	uint8_t section = IRQNumber %4; //tere are four IRQ register section in one register
	uint8_t shift_value = (8*section) + (8-NO_OF_PR_BIT_IMPLIMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority <<(shift_value)); //nvic priority base address defined in mcu file and if we add one it will move up by one to next register

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len) // Tx buffer pointer ten assigned to base address of tx buffer register
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
	// saving information in some global variable to use it later wen interrupt occurs for txne
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLengt = Len;

	//mark spi status as busy in transmission so that no other code take over same spi peripheral
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//enabling txeie interrupt to get interrupt wen trasnmission register is empty
	pSPIHandle->pSPIx->CR2 |= (1<<7);

	//if transmission data register is empty ten txeie interrupt will be triggered and code in ISR will be executed

}
	return state;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len)
{
	uint8_t status = pSPIHandle->RxState;
	if(status != SPI_BUSY_IN_RX)
	{
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLengt = Len;
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		pSPIHandle->pSPIx->CR2 |= (1<<6);
	}
	return status;
}

void I2C_SlaveSend_Data(I2C_RegDef_t *pI2Cx,uint8_t data)
{
	pI2Cx->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}
void SPI_IRQHandling(SPI_Handle_t *pSPIhandlex)
{
	// first cek for wic reason interrupt is occured

	uint8_t temp1,temp2;
	temp1 = pSPIhandlex->pSPIx->SR & (1<<1); // for txe flag
	temp2 = pSPIhandlex->pSPIx->CR2 & (1<<7); // for Txeie flag
	if(temp1 && temp2)
	{
		// Handle txe
		spi_txe_interrupt_handle(pSPIhandlex);
	}

	temp1 = pSPIhandlex->pSPIx->SR & (1<<0); // for rxe flag
	temp2 = pSPIhandlex->pSPIx->CR2 & (1<<6); // for Rxeie flag

	if(temp1 && temp2)
	{
		spi_rxe_interrupt_handle(pSPIhandlex);
	}
	// cecking for overrun flag
	temp1 = pSPIhandlex->pSPIx->SR & (1<<6); // for ovr flag is set or not
	temp2 = pSPIhandlex->pSPIx->CR2 & (1<<5); // to know if overrun is enbaled

	if(temp1 && temp2)
	{
		spi_ovr_err_interrupt_handle(pSPIhandlex);
	}


}

//static function implementation

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIhandlex)
{

	if((pSPIhandlex->pSPIx->CR1 & (1<<11)))
	{
		//if this is true ten it is 16 bit data
		// now load data into data register of SPI
		pSPIhandlex->pSPIx->DR = *((uint16_t*)pSPIhandlex->pTxBuffer); // basically ptxbuffer pointer is of 8 bit so to convert it to 16 we should type cast it to 16 bit pointer
		pSPIhandlex->TxLengt--;
		pSPIhandlex->TxLengt--;
		(uint16_t*)pSPIhandlex->pTxBuffer++;
	}else
	{
		//Eight bit data format
		pSPIhandlex->pSPIx->DR = *pSPIhandlex->pTxBuffer;
		pSPIhandlex->TxLengt--;
		pSPIhandlex->pTxBuffer++;
	}
	if(!pSPIhandlex->TxLengt)
	{
		// so len is 0 you can close te communication clear txeie bit
		void SPI_Close_Tr(SPI_Handle_t *pSPIHandle);
		SPI_ApplicationEventCallback(pSPIhandlex,SPI_TX_EVENT_COMPLETE);
	}
}
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIhandlex)
{
	if((pSPIhandlex->pSPIx->CR1 & (1<<11)))
				{
					//if this is true ten it is 16 bit data
					// now load data from dr to rx buffer
					*((uint16_t*)pSPIhandlex->pRxBuffer) = pSPIhandlex->pSPIx->DR; // basically ptxbuffer pointer is of 8 bit so to convert it to 16 we should type cast it to 16 bit pointer
					pSPIhandlex->RxLengt--;
					pSPIhandlex->RxLengt--;
					pSPIhandlex->pRxBuffer++;
					pSPIhandlex->pRxBuffer++;
				}else
				{
					//Eight bit data format
					*pSPIhandlex->pRxBuffer = pSPIhandlex->pSPIx->DR ;
					pSPIhandlex->RxLengt--;
					pSPIhandlex->pRxBuffer++;
				}
	if(!pSPIhandlex->RxLengt)
	{
		// CLOSING OF RECEPTION AFTER FULL DATA IS TRANSMITTED AS SUGGESTED BY TE USER
		void SPI_Close_Rx(SPI_Handle_t *pSPIhandlex);
		SPI_ApplicationEventCallback(pSPIhandlex,SPI_RX_EVENT_COMPLETE);

	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIhandlex)
{	uint8_t temp;
	// clear overun flag
	if(pSPIhandlex->TxState != SPI_BUSY_IN_TX)
	// inform application about occurance of error
	{
		temp = pSPIhandlex->pSPIx->DR;
		temp = pSPIhandlex->pSPIx->SR;
	}
	(void)temp; // to clear unused variable warning and to tell compiler
	SPI_ApplicationEventCallback(pSPIhandlex,SPI_EVENT_OVR_ERR);
}

void SPI_Close_Tr(SPI_Handle_t *pSPIHandle)
{
pSPIHandle->pSPIx->CR2 &= ~(1<<7);
pSPIHandle->pTxBuffer = NULL;
pSPIHandle->TxLengt = 0;
pSPIHandle->TxState = SPI_READY;
}
void SPI_Close_Rx(SPI_Handle_t *pSPIHandle)
{
pSPIHandle->pSPIx->CR2 &= ~(1<<6);
pSPIHandle->pRxBuffer = NULL;
pSPIHandle->RxLengt = 0;
pSPIHandle->RxState = SPI_READY;
}

void SPI_Clear_Ovr_Flag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//tis is weak implememtation and tis can be ovewritten by user.
}
