/*
 * stm32f407xx.h
 *
 *  Created on: Sep 6, 2021
 *      Author: 007ma
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include <stdint.h>
#include<stddef.h>

#define __weak  __attribute__((weak))

/*ARM cortex specific register base addresses of NVIC ISER Register address details
 *
 */
#define NVIC_ISER0        ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1        ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2        ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3        ((volatile uint32_t*)0xE000E10C)

/*ARM cortex specific register base addresses of NVIC ICER Register address details
 *
 */
#define NVIC_ICER0		  ((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1		  ((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2		  ((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3		  ((volatile uint32_t*)0XE000E18C)

//address of arm cortex priority registers

#define NVIC_PR_BASE_ADDR ((volatile uint32_t*)0xE000E400)
#define NO_OF_PR_BIT_IMPLIMENTED     4


/*base addresses of flash and sram memory's*/
#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20000000U //sram2 is not used its not needed in this project so it is not defined
#define SRAM2_BASEADDR 0x20001C00U /*sram1 plus (112*1024)+base addr of sram 1*/
#define ROM_BASEADDR   0x1FFF0000U
#define SRAM           SRAM1_BASEADDR// sram1 is main ram here so its address is given


/*base addresses AHBX and APBx bus of peripherals */

#define PERI_PBASEADDR  0x40000000U
#define APB1_PBASEADDR  PERI_PBASEADDR /*offset of 0x00*/
#define APB2_PBASEADDR  0x40010000U   /*offset of 0x00010000 */
#define AHB1_PBASEADDR  0x40020000U   /*offset of 0x00020000 */
#define AHB2_PBASEADDR  0x50000000U   /*offset of 0x1000 0000 */

/*defining base addresses of peripherals which are under AHB1 bus
 * Note: Only listed peripherals which I am going to use in this project
 */

#define GPIOA_BASEADDR (AHB1_PBASEADDR + 0x0000)  /*Each Gpio pin is of 4 bytes*/
#define GPIOB_BASEADDR (AHB1_PBASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB1_PBASEADDR + 0X0800)
#define GPIOD_BASEADDR (AHB1_PBASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB1_PBASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB1_PBASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB1_PBASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB1_PBASEADDR + 0x1C00)
#define GPIOI_BASEADDR (AHB1_PBASEADDR + 0x2000)
#define RCC_BADEADDR   (AHB1_PBASEADDR + 0x3800)

/*Clock enabling Macros for GPIOx peripherals*/
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1<<8))

/*Clock enabling Macros for I2Cx peripherals*/

#define I2C1_PCLK_EN()   (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()   (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()   (RCC->APB1ENR |= (1<<23))

/*Clock enabling Macros for SPIx peripherals*/

#define SPI1_PCLK_EN()   (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()   (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()   (RCC->APB1ENR |= (1<<15))

/*Clock enabling Macros for UARTx peripherals*/

#define UART4_PCLK_EN()  (RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()  (RCC->APB1ENR |= (1<<20))

/*Clock enabling Macros for USARTx peripherals*/

#define USART1_PCLK_EN()  (RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()  (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()  (RCC->APB1ENR |= (1<<18))
#define USART6_PCLK_EN()  (RCC->APB2ENR |= (1<<5))


#define SYSCFG_PCLK_EN()  (RCC->APB2ENR |= (1<<14))

/*Clock disabling Macros for GPIOx peripherals*/

#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(1<<8))

/*Clock disabling Macros for I2Cx peripherals*/

#define I2C1_PCLK_DI()   (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()   (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()   (RCC->APB1ENR &= ~(1<<23))

/*Clock disabling Macros for SPIx peripherals*/

#define SPI1_PCLK_DI()   (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()   (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()   (RCC->APB1ENR &= ~(1<<15))

/*Clock disabling Macros for UARTx peripherals*/

#define UART4_PCLK_DI()  (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()  (RCC->APB1ENR &= ~(1<<20))

/*Clock disabling Macros for USARTx peripherals*/

#define USART1_PCLK_DI()  (RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()  (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()  (RCC->APB1ENR &= ~(1<<18))
#define USART6_PCLK_DI()  (RCC->APB2ENR &= ~(1<<5))

#define SYSCFG_PCLK_DI()  (RCC->APB2ENR &= ~(1<<14))

/* RESETTING GPIO PERI
 *
 */

#define GPIOA_REG_RESET() do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET() do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET() do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET() do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET() do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_REG_RESET() do{ (RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_REG_RESET() do{ (RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6)); }while(0)
#define GPIOH_REG_RESET() do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); }while(0)
#define GPIOI_REG_RESET() do{ (RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8)); }while(0)

//resetting spix

#define SPI1_REG_RESET() do{ (RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12)); }while(0)
#define SPI2_REG_RESET() do{ (RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14)); }while(0)
#define SPI3_REG_RESET() do{ (RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15)); }while(0)


#define I2C1_REG_RESET() do{ (RCC->APB1RSTR |= (1<<21)); (RCC->APB1RSTR &= ~(1<<21)); }while(0)
#define I2C2_REG_RESET() do{ (RCC->APB1RSTR |= (1<<22)); (RCC->APB1RSTR &= ~(1<<22)); }while(0)
#define I2C3_REG_RESET() do{ (RCC->APB1RSTR |= (1<<23)); (RCC->APB1RSTR &= ~(1<<23)); }while(0)

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 : \
									 (x == GPIOB) ? 1 : \
									 (x == GPIOC) ? 2 : \
									 (x == GPIOD) ? 3 : \
									 (x == GPIOE) ? 4 : \
									 (x == GPIOF) ? 5 : \
									 (x == GPIOG) ? 6 : \
									 (x == GPIOH) ? 7 : \
									 (x == GPIOI) ? 8 : 0 )

/*defining base addresses of peripherals which are under APB1 bus
 * Note: Only listed peripherals which I am going to use in this project
 */

#define I2C1_BASEADDR   (APB1_PBASEADDR + 0x5400)
#define I2C2_BASEADDR   (APB1_PBASEADDR + 0x5800)
#define I2C3_BASEADDR   (APB1_PBASEADDR + 0x5C00)

#define SPI2_BASEADDR   (APB1_PBASEADDR + 0x3800)
#define SPI3_BASEADDR   (APB1_PBASEADDR + 0X3C00)

#define USART2_BASEADDR (APB1_PBASEADDR + 0x4400)
#define USART3_BASEADDR (APB1_PBASEADDR + 0x4800)
#define UART4_BASEADDR  (APB1_PBASEADDR + 0x4C00)
#define UART5_BASEADDR  (APB1_PBASEADDR + 0x5000)

/*defining base addresses of peripherals which are under APB2 bus
 * Note: Only listed peripherals which I am going to use in this project
  */

#define EXTI_BASEADDR   (APB2_PBASEADDR + 0x3C00)
#define SPI1_BASEADDR   (APB2_PBASEADDR + 0x3000)
#define SYSCFG_BASEADDR (APB2_PBASEADDR + 0x3800)
#define USART1_BASEADDR (APB2_PBASEADDR + 0x1000)
#define USART6_BASEADDR (APB2_PBASEADDR + 0x1400)

/*defining general register mapping of GPIOx register by using an struct*/

typedef struct
{

	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2]; /*because we have two afr registers we can also define each individually */

}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t 		   RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t 		   RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t 		   RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t 		   RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t 		   RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t 		   RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t 		   RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
}RCC_RegDef_t;

/* EZTI peri based registers for enebling mode of intruppt
 *
 */

typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;

}EXTI_RegDef_t;

/* Creating sysconfig register structure for enabling exti pin to connect to nvic
 */

typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t 		  RESERVED[2];
	volatile uint32_t CMPCR;

}SYSCFG_RegDef_t;

//generic struct for all spi peripherals

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;

}SPI_RegDef_t;


typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;



}I2C_RegDef_t;

typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
}USART_RegDef_t;
/*type casting all GPIOs base addresses in formate of RefDef so we can directly point it to base address of
 * GPIO example GPIO_RegDef_t *pGPIO = GPIOA if we type cast we can use like this
 */

#define GPIOA  ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB  ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC  ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD  ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE  ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF  ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG  ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH  ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI  ((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define RCC    ((RCC_RegDef_t*)  RCC_BADEADDR)
#define EXTI   ((EXTI_RegDef_t*)  EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t*)  SYSCFG_BASEADDR)
#define SPI1	((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3	((SPI_RegDef_t*) SPI3_BASEADDR)
#define I2C1	((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2	((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3	((I2C_RegDef_t*) I2C3_BASEADDR)
#define USART1  ((USART_RegDef_t*) USART1_BASEADDR)
#define USART2  ((USART_RegDef_t*) USART2_BASEADDR)
#define USART3  ((USART_RegDef_t*) USART3_BASEADDR)
#define USART6  ((USART_RegDef_t*) USART6_BASEADDR)
#define UART4  ((USART_RegDef_t*) UART4_BASEADDR)
#define UART5  ((USART_RegDef_t*) UART5_BASEADDR)

/* Some generic macros
 *
 */

#define ENABLE 		1
#define DISABLE 	0
#define SET			ENABLE
#define RESET		DISABLE
#define FLAG_RESET RESET
#define FLAG_SET   SET

#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

// refer inturrup section in rm407 manual
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5  	23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_I2C1EV		31
#define IRQ_NO_I2C1ER		32
#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
//DEFINING IRQ PRIORITY

#define IRQ_PRIO_0          0
#define IRQ_PRIO_1          1
#define IRQ_PRIO_2          2
#define IRQ_PRIO_3          3
#define IRQ_PRIO_4          4
#define IRQ_PRIO_5          5
#define IRQ_PRIO_6          6
#define IRQ_PRIO_7          7
#define IRQ_PRIO_8          8
#define IRQ_PRIO_9          9
#define IRQ_PRIO_10         10
#define IRQ_PRIO_11         11
#define IRQ_PRIO_12         12
#define IRQ_PRIO_13         13
#define IRQ_PRIO_14         14
#define IRQ_PRIO_15         15




#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_uart_driver.h"
#include"stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
