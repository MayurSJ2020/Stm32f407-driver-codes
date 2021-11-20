/*
 * stm32407xx_uart_driver.c
 *
 *  Created on: 26-Oct-2021
 *      Author: 007ma
 */
#include "stm32f407xx_uart_driver.h"

void USART_PeriClockEnable(USART_RegDef_t *pUSARTx,uint8_t EnorDI)
{
	if(EnorDI == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}

	}
}


void USART_PeripheralClockEnable(USART_RegDef_t *pUSARTx,uint8_t EnorDI)
{
	if(EnorDI == ENABLE)
	{
		pUSARTx->CR1 |= (1<<13);
	}
	else
	{
		pUSARTx->CR1 &= ~(1<<13);
	}
}


uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx,uint8_t FlagName)
{
	if(pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
void USART_ClearFlag(USART_RegDef_t *pUSARTx,uint8_t FlagName)
{
	pUSARTx->SR &= ~(FlagName);
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	 USART_PeriClockEnable(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << 2);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << 3 );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << 2) | ( 1 << 3) );
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLen << 12 ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EVEN)
	{
		//Implement the code to enale the parity control
		tempreg |= ( 1 << 10 );


		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << 10);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << 9);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << 12;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HwFlowControl == HARDWARE_FLOWCONTROL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << 9);


	}else if (pUSARTHandle->USART_Config.USART_HwFlowControl == HARDWARE_FLOWCONTROL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= ( 1 << 8);

	}else if (pUSARTHandle->USART_Config.USART_HwFlowControl == HARDWARE_FLOWCONTROL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= ( 1 << 8)|( 1 << 9);
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baudrate);
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,TXNE_Flag));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLen == USART_WORD_LEN9)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,TCcompleteFlag));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,RxneFlag));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLen == USART_WORD_LEN9)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) pUSARTHandle->pUSARTx->DR & 0X7F;

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->txstate;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->txlen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->txstate = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1<<7);


		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1<<6);


	}

	return txstate;

}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->rxstate;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->rxlen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->rxstate = USART_BUSY_IN_RX;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1<<5);

	}

	return rxstate;

}


void USART_IRQ_Interuppt_Config(uint8_t IRQNumber, uint8_t EnorDI)
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
void USART_IRQ_Interuppt_priority_config(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4; //calculated to know wic IRQ register total tere are 60 registers
	uint8_t section = IRQNumber %4; //tere are four IRQ register section in one register
	uint8_t shift_value = (8*section) + (8-NO_OF_PR_BIT_IMPLIMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority <<(shift_value)); //nvic priority base address defined in mcu file and if we add one it will move up by one to next register

}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << 15))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((PCLKx) / (8 *BaudRate))*100;
  }else
  {
	   //over sampling by 16
	  usartdiv = ((PCLKx) / (16 * BaudRate))*100;
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << 15))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}
