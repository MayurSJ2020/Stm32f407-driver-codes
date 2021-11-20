/*
 * i2c_sendit.c
 *
 *  Created on: 17-Oct-2021
 *      Author: 007ma
 */


#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

//extern void initialise_monitor_handles();



#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buf[32];

/*
 * PB6-> SCL
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTER;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_6;
	GPIO_Init(&I2CPins);


	//sda
	I2CPins.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_7;
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_PinConfig.I2C_ACKControl = I2C_ACK_EN;
	I2C1Handle.I2C_PinConfig.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_PinConfig.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_PinConfig.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_Pinnumber = GPIO_PIN_N0_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;

	GPIO_Init(&GPIOBtn);

}


int main(void)
{

	uint8_t commandcode;

	uint8_t len;

	//initialise_monitor_handles();

	//printf("Application is running\n");

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//IRQ config
	I2C_IRQ_Interuppt_Config(IRQ_NO_I2C1EV, ENABLE);
	I2C_IRQ_Interuppt_Config(IRQ_NO_I2C1ER, ENABLE);

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2C_ACK_EN);

	printf("Start\n");

	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_N0_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = 0x51;

		while(I2C_MasterSend_DataIT(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_SR) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1Handle,&len,1,SLAVE_ADDR,I2C_SR)!= I2C_READY);

		commandcode = 0x52;
		while(I2C_MasterSend_DataIT(&I2C1Handle,&commandcode,1,SLAVE_ADDR,I2C_SR) != I2C_READY);


		while(I2C_MasterReceiveDataIT(&I2C1Handle,rcv_buf,len,SLAVE_ADDR,I2C_NO_SR)!= I2C_READY);

		rcv_buf[len+1] = '\0';

		printf("Data : %s\n",rcv_buf);

	}

}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ERR_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	if(AppEv == I2C_EV_TX_COMPLETE)
	{
		printf("Transmission complete\n");
	}
	else if(AppEv == I2C_EV_RX_COMPLETE)
	{
		printf("Reception is complete\n");
	}
	else if(AppEv == I2C_ERR_AF)
	{
		printf("Ack failure\n");
		I2C_Close_SendData(&I2C1Handle);

		//Generate stop condition
		I2C_GenerateStopCondition(I2C1);

		while(1);


	}

}

