/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 02-Oct-2021
 *      Author: 007ma
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include<stm32f407xx.h>

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t	 I2C_ACKControl;
	uint16_t I2C_FMDutyCycle; //RISE TIME

}I2C_PinConfig_t;

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	uint8_t 	 *pTxBuffer;
	uint8_t		 *pRxBuffer;
	uint32_t	 TxLengt;
	uint32_t	 RxLengt;
	uint8_t 	 TxRxState;
	uint8_t 	 Deviceaddr;
	uint8_t 	 RxSize;
	uint8_t 	 Sr;
	I2C_PinConfig_t I2C_PinConfig;

}I2C_Handle_t;

// macros for sclspeed

#define I2C_SCL_SPEED_SM  		100000
#define I2C_SCL_SPEED_FM4K  	400000
#define I2C_SCL_SPEED_FM2K  	200000

//ACK CONTROL

#define I2C_ACK_EN				1
#define I2C_ACK_DI				0

//FOR DUTY CYCLE OF CLK DURING FAST MODE

#define I2C_FM_DUTY_2 		0
#define I2C_FM_DUTY_16_9	1

//I2C STATUS FLAGS

#define I2C_SB_FLAG         (1<<0)
#define I2C_ADDR_FLAG       (1<<1)
#define I2C_BTF_FLAG        (1<<2)
#define I2C_STOPF_FLAG      (1<<4)
#define I2C_RXE_FLAG		(1<<6)
#define I2C_TXE_FLAG		(1<<7)
#define I2C_BERR_FLAG		(1<<8)
#define I2C_ARLO_FLAG		(1<<9)
#define I2C_AF_FLAG			(1<<10)
#define I2C_OVR_FLAG		(1<<11)
#define I2C_PECERR_FLAG		(1<<12)
#define I2C_TIMEOUT_FLAG	(1<<14)

//macros for repeated start
#define I2C_NO_SR			0
#define I2C_SR				1

// for interupt based data sending

#define I2C_READY				0
#define I2C_BUSY_IN_TX			1
#define I2C_BUSY_IN_RX			2

//application callback macros

#define I2C_EV_TX_COMPLETE		0
#define I2C_EV_STOP				1
#define I2C_EV_RX_COMPLETE		2
#define I2C_ERR_BERR			3
#define I2C_ERR_ARLO			4
#define I2C_ERR_AF				5
#define I2C_ERR_OVRRN			6
#define I2C_ERR_TIMEOUT			7
#define I2C_EV_DATA_REQ			8
#define I2C_EV_DATA_RECEIVE		9


//API FOR USER

void I2C_PericlockEnable(I2C_RegDef_t *pI2Cx,uint8_t ENorDI);

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_MasterSend_Data(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint8_t Len,uint8_t Slaveaddress,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint8_t Len,uint8_t Slaveaddress,uint8_t Sr);

uint8_t I2C_MasterSend_DataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint8_t Len,uint8_t Slaveaddress,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer,uint8_t Len,uint8_t Slaveaddress,uint8_t Sr);

void I2C_IRQ_Interuppt_Config(uint8_t IRQNumber, uint8_t EnorDI);
void I2C_IRQ_Interuppt_priority_config(uint8_t IRQNumber,uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ERR_IRQHandling(I2C_Handle_t *pI2CHandle);


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t ENorDI);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx,uint8_t ENorDI);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_Close_SendData(I2C_Handle_t *pI2CHandle);

//slave
void I2C_SlaveSend_Data(I2C_RegDef_t *pI2Cx,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);
void I2C_SlaveEnableDisableCallbackEve(I2C_RegDef_t *pI2Cx,uint8_t ENorDI);


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
