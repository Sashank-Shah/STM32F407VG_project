/*
 * 		STM32F407xx.h
 *
 *  	Author: SASHANK SHAH
 *		It is a device specific header file, check for your microcontroller.
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>
/*--------------------------- BASE ADDRESS OF MEMORY--------------------------------*/
//Reference manual -> Section 3
#define _vo					volatile

#define FMEM_BADDR     		0x08000000U
#define SRAM1_BADDR    		0x20000000U
#define SRAM2_BADDR	 		0x20001C00U //BADDRr_SRAM2 = BADDR_SRAM1 + hex(SRAM1_SIZE_BYTES)
#define ROM_BADDR		 	0x1FFF0000U //ROM = System memory

/*--------------------------BASE ADDRESS OF APBx and AHBx PERIPHERAL BUS ----------*/
//	Reference Manual -> Section 2.3

#define PRIPHL_BADDR 			0x40000000U
#define APB1_PRIPHL_BADDR		PRIPHL_BADDR
#define APB2_PRIPHL_BADDR		0x40010000U
#define AHB1_PRIPHL_BADDR		0x40020000U
#define AHB2_PRIPHL_BADDR		0x50000000U

/*--------------------BASE ADDRESS OF peripherals hanging on AHB1 BUS----------------*/
//	Reference Manual -> Section 2.3

#define GPIOA_BADDR 		(AHB1_PRIPHL_BADDR + 0x00) // use this way of addressing
#define GPIOB_BADDR			 0x40020400U 					// or get base address directly from reference manual
#define GPIOC_BADDR			 0x40020800U
#define GPIOD_BADDR			 0x40020C00U				//GPIOx BASE ADDRESS
#define GPIOE_BADDR			 0x40021000U
#define GPIOF_BADDR			 0x40021400U
#define GPIOG_BADDR			 0x40021800U
#define GPIOH_BADDR			 0x40021C00U
#define GPIOI_BADDR			 0x40022000U
#define RCC_BADDR			 0x40023800U

/*--------------------BASE ADDRESS OF peripherals hanging on APB1 BUS----------------*/
//	Reference Manual -> Section 2.3

#define USART2_BADDR		0x40004400U
#define USART3_BADDR		0x40004800U
#define UART4_BADDR			0x40004C00U
#define UART5_BADDR			0x40005000U

#define I2C1_BADDR			0x40005400U
#define I2C2_BADDR			0x40005800U
#define I2C3_BADDR			0x40005C00U

#define SPI2_BADDR			0x40003800U
#define SPI3_BADDR			0x40003C00U


/*--------------------BASE ADDRESS OF peripherals hanging on APB2 BUS----------------*/
//	Reference Manual -> Section 2.3

#define USART1_BADDR		0x40011000U
#define USART6_BADDR		0x40011400U

#define SPI1_BADDR			0x40013000U

#define EXTI_BADDR			0x40013C00U
#define SYSCFG_BADDR		0x40013800U


/*--------------------structure for GPIO PORT registers----------------*/
//	Reference Manual -> Section 8.4





typedef struct
{
	_vo uint32_t MODER;		//GPIO port mode register				 Address offset: 0x00
	_vo uint32_t OTYPER;	//GPIO port output type register 		 Address offset: 0x04
	_vo uint32_t OSPEEDR; 	//GPIO port output speed register  		 Address offset: 0x08
	_vo uint32_t PUPDR; 	//GPIO port pull-up/pull-down register   Address offset: 0x0C
	_vo uint32_t IDR;		//GPIO port input data register          Address offset: 0x10
	_vo uint32_t ODR;		//GPIO port output data register 		 Address offset: 0x14
	_vo uint32_t BSRR;		//GPIO port bit set/reset register       Address offset: 0x18
	_vo uint32_t LCKR;		//GPIO port configuration lock register  Address offset: 0x1C
	_vo uint32_t AFR[2];	//GPIO alternate function low register   Address offset: 0x20	AFR[0]
	 	 	 	 	 	 	//	 	 	 	 	 	 	 	 		 Address offset: 0x24	AFR[1]

}gpio_RegDef_t;


/**********GPIOx*********************/
#define GPIOA 					((gpio_RegDef_t*)GPIOA_BADDR)  //makes it easy to use
#define GPIOB					((gpio_RegDef_t*)GPIOB_BADDR)  // pointer to base address of GPIOx
#define GPIOC 					((gpio_RegDef_t*)GPIOC_BADDR)
#define GPIOD 					((gpio_RegDef_t*)GPIOD_BADDR)
#define GPIOE 					((gpio_RegDef_t*)GPIOE_BADDR)
#define GPIOF 					((gpio_RegDef_t*)GPIOF_BADDR)
#define GPIOG 					((gpio_RegDef_t*)GPIOG_BADDR)
#define GPIOH 					((gpio_RegDef_t*)GPIOH_BADDR)
#define GPIOI 					((gpio_RegDef_t*)GPIOI_BADDR)

/* ------example-------

 typedef struct			|	int main()
 {						|	{
 	 int age;			|		new_age *ptr, p1;
 }new_age;				|		ptr = &p1;
 	 	 	 	 	 	|	 	scanf("%d", &ptr->age);
 	 	 	 	 	 	| 	 	printf("%d", ptr->age);
						|	}
---------example ends --------
*/


/*--------------------structure for RCC registers----------------*/
//	Reference Manual -> Section 6.3.26

typedef struct
{
	_vo uint32_t CR;  			//RCC clock control register	    	Address offset: 0x00
	_vo uint32_t PLLCFGR;		//RCC PLL configuration register 		Address offset: 0x04
	_vo uint32_t CFGR;			//RCC clock configuration register		Address offset: 0x08
	_vo uint32_t CIR;			//RCC clock interrupt register			Address offset: 0x0C

	_vo uint32_t AHB1RSTR;      //RCC AHB1 peripheral reset register	Address offset: 0x10
	_vo uint32_t AHB2RSTR;		//RCC AHB2 peripheral reset register	Address offset: 0x14
	_vo uint32_t AHB3RSTR;		//RCC AHB3 peripheral reset register	Address offset: 0x18
	uint32_t RESD0;					//RESERVED
	_vo uint32_t APB1RSTR;		//RCC APB1 peripheral reset register	Address offset: 0x20
	_vo uint32_t APB2RSTR;		//RCC APB2 peripheral reset register	Address offset: 0x24
	uint32_t RESD1[2];

	_vo uint32_t AHB1ENR;		//RCC AHB1 peripheral clock register	Address offset: 0x30
	_vo uint32_t AHB2ENR;		//RCC AHB2 peripheral clock enable 		Address offset: 0x34
	_vo uint32_t AHB3ENR;		//RCC AHB3 peripheral clock enable 		Address offset: 0x38
	uint32_t RESD2;
	_vo uint32_t APB1ENR;		//RCC APB1 peripheral clock enable		Address offset: 0x40
	_vo uint32_t APB2ENR;		//RCC APB2 peripheral clock enable 		Address offset: 0x44
	uint32_t RESD3[2];
	_vo uint32_t AHB1LPENR;    //RCC AHB1 peripheral clock enable in low power mode register	Address offset: 0x50
	_vo uint32_t AHB2LPENR;	   //RCC AHB2 peripheral clock enable in low power mode register	Address offset: 0x54
	_vo uint32_t AHB3LPENR;	   //RCC AHB3 peripheral clock enable in low power mode register	Address offset: 0x58
	uint32_t RESD4;
	_vo uint32_t APB1LPENR;	   //RCC APB1 peripheral clock enable in low power mode register	Address offset: 0x60
	_vo uint32_t APB2LPENR;	   //RCC APB2 peripheral clock enabled in low power mode register	Address offset: 0x64
	uint32_t RESD5[2];

	_vo uint32_t BDCR;			//RCC Backup domain control register 	Address offset: 0x70
	_vo uint32_t CSR;			//RCC clock control & status register 	Address offset: 0x74
	uint32_t RESD6[2];
	_vo uint32_t SSCGR;		    //RCC spread spectrum clock generation register 	Address offset: 0x80
	_vo uint32_t PLLI2SCFGR;	//RCC PLLI2S configuration register					Address offset: 0x84
	_vo uint32_t PLLSAICFGR;	//RCC PLL configuration register					Address offset: 0x88
	_vo uint32_t DCKCFGR;		//RCC Dedicated Clock Configuration Register		Address offset: 0x8C

}rcc_RegDef_t;

#define RCC 			((rcc_RegDef_t*)RCC_BADDR)



/*--------------------structure for EXTI registers----------------*/
typedef struct
{
	_vo uint32_t IMR; 		//EXTI interrupt mask register       			Address offset: 0x00
	_vo uint32_t EMR;		//EXTI event mask register           			Address offset: 0x04
	_vo uint32_t RTSR;		//EXTI rising trigger selection register		Address offset: 0x08
	_vo uint32_t FTSR;		//EXTI falling trigger selection register		Address offset: 0x0C
	_vo uint32_t SWIER;		//EXTI software interrupt event register		Address offset: 0x10
	_vo uint32_t PR;		//EXTI pending register							Address offset: 0x14

}exti_RegDef_t;


#define	EXTI			((exti_RegDef_t*)EXTI_BADDR)



/*--------------------GPIO code for Port select through SYSCFG----------------*/
#define GPIO_CODE(X)	   ((X == GPIOA)?0:\
							(X == GPIOB)?1:\
							(X == GPIOC)?2:\
							(X == GPIOD)?3:\
							(X == GPIOE)?4:\
							(X == GPIOF)?5:\
							(X == GPIOG)?6:\
							(X == GPIOH)?7:\
							(X == GPIOI)?8:0)

/*--------------------structure for SYSCFG registers----------------*/
typedef struct
{
	_vo uint32_t MEMRMP;	//SYSCFG memory remap register						Address offset: 0x00
	_vo uint32_t PMC;		//SYSCFG peripheral mode configuration  			Address offset: 0x04
	_vo uint32_t EXTICR[4];	//SYSCFG external interrupt configuration (1 to 4)	Address offset: 0x08 - 0x14
	uint32_t RESD7[2];      //Reserved
	_vo uint32_t CMPCR;	    //SYSCFG compensation cell control register			Address offset: 0x20

}syscfg_RegDef_t;

#define SYSCFG			((syscfg_RegDef_t *)SYSCFG_BADDR)


/*-------------------GPIOx clock enable---------------------*/
#define GPIOA_CLK_EN() 				(RCC->AHB1ENR |= (1<<0))
#define GPIOB_CLK_EN() 				(RCC->AHB1ENR |= (1<<1))
#define GPIOC_CLK_EN() 				(RCC->AHB1ENR |= (1<<2))
#define GPIOD_CLK_EN() 				(RCC->AHB1ENR |= (1<<3))
#define GPIOE_CLK_EN() 				(RCC->AHB1ENR |= (1<<4))
#define GPIOF_CLK_EN() 				(RCC->AHB1ENR |= (1<<5))
#define GPIOG_CLK_EN() 				(RCC->AHB1ENR |= (1<<6))
#define GPIOH_CLK_EN() 				(RCC->AHB1ENR |= (1<<7))
#define GPIOI_CLK_EN() 				(RCC->AHB1ENR |= (1<<8))

/*-------------------GPIOx clock disable---------------------*/
#define GPIOA_CLK_DIS() 				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_CLK_DIS() 				(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_CLK_DIS() 				(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_CLK_DIS() 				(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_CLK_DIS() 				(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_CLK_DIS() 				(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_CLK_DIS() 				(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_CLK_DIS() 				(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_CLK_DIS() 				(RCC->AHB1ENR &= ~(1<<8))

/*-----------------GPIOx Register RESET---------------------*/
#define GPIOA_RESET() 				do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_RESET() 				do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_RESET() 				do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_RESET() 				do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_RESET() 				do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_RESET() 				do{ (RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_RESET() 				do{ (RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6)); }while(0)
#define GPIOH_RESET() 				do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); }while(0)
#define GPIOI_RESET() 				do{ (RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8)); }while(0)


#define SYSCFG_EN()					(RCC->APB2ENR |= (1<<14))

/*----------------------IRQ numbers--------------------------*/
#define IRQ0						6
#define IRQ1						7
#define IRQ2						8
#define IRQ3						9
#define IRQ4						10
#define IRQ9_5						23
#define IRQ15_10					40
#define SPI1_IRQ					35
#define SPI2_IRQ					36
#define SPI3_IRQ					51

/*------------------NVIC registers--------------------------*/
#define NVIC_ISER0			((_vo uint32_t*)0xE000E100)
#define NVIC_ISER1			((_vo uint32_t*)0xE000E104)
#define NVIC_ISER2			((_vo uint32_t*)0xE000E108)
#define NVIC_ISER3			((_vo uint32_t*)0xE000E10C)
#define NVIC_ISER4			((_vo uint32_t*)0xE000E110)
#define NVIC_ISER5			((_vo uint32_t*)0xE000E114)
#define NVIC_ISER6			((_vo uint32_t*)0xE000E118)
#define NVIC_ISER7			((_vo uint32_t*)0xE000E11C)

#define NVIC_ICER0			((_vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((_vo uint32_t*)0XE000E184)
#define NVIC_ICER2			((_vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((_vo uint32_t*)0XE000E18C)
#define NVIC_ICER4			((_vo uint32_t*)0XE000E190)
#define NVIC_ICER5			((_vo uint32_t*)0XE000E194)
#define NVIC_ICER6			((_vo uint32_t*)0XE000E198)
#define NVIC_ICER7			((_vo uint32_t*)0XE000E19C)

#define NVIC_IPR0			((_vo uint32_t*)0xE000E400)
#define NVIC_IPR1			((_vo uint32_t*)0xE000E404)
#define NVIC_IPR2			((_vo uint32_t*)0xE000E408)
#define NVIC_IPR3			((_vo uint32_t*)0xE000E40C)
#define NVIC_IPR4			((_vo uint32_t*)0xE000E410)
#define NVIC_IPR5			((_vo uint32_t*)0xE000E414)
#define NVIC_IPR6			((_vo uint32_t*)0xE000E418)
#define NVIC_IPR7			((_vo uint32_t*)0xE000E41C)

#define NVIC_PRTY		((_vo uint8_t *)0xE000E400U)



/*--------------------structure for NVIC Parity registers----------------*/
/*typedef struct
{
	_vo uint32_t NVIC_PTY[60];   //NVIC Parity registers numbered from 0 to 59
}nvic_RegDef_t;

#define NVIC_PTR			((nvic_RegDef_t*)NVIC_PRTY_BASEADDR)

*/


/*------------------------------ SPI ---------------------------------------------*/

typedef struct
{
	_vo uint32_t SPI_CR1;			//SPI control Register 				Address Offset 0x00
	_vo uint32_t SPI_CR2;			//SPI control Register 				Address Offset 0x04
	_vo uint32_t SPI_SR;			//SPI status register				Address Offset 0x08
	_vo uint32_t SPI_DR;			//SPI data register					Address Offset 0x0C
	_vo uint32_t SPI_CRCPR;			//SPI CRC polynomial register		Address Offset 0x10
	_vo uint32_t SPI_RXCRCR;		//SPI RX CRC register				Address Offset 0x14
	_vo uint32_t SPI_TXCRCR;		//SPI TX CRC register				Address Offset 0x18
	_vo uint32_t SPI_I2SCFGR;		//SPI_I2S configuration register 	Address Offset 0x1C
	_vo uint32_t SPI_I2SPR;			//SPI_I2S prescaler register		Address Offset 0x20

}spi_RegDef_t;


#define SPI1				((spi_RegDef_t*)SPI1_BADDR)
#define SPI2				((spi_RegDef_t*)SPI2_BADDR)
#define SPI3				((spi_RegDef_t*)SPI3_BADDR)



//Peripheral clock enable macro for SPI
#define SPI1_EN()			(RCC->APB2ENR |= (1<<12))
#define SPI2_EN()			(RCC->APB1ENR |= (1<<14))
#define SPI3_EN()			(RCC->APB1ENR |= (1<<15))

//Peripheral clock disable macro for SPI
#define SPI1_DS()			(RCC->APB2ENR &= ~(1<<12))
#define SPI2_DS()			(RCC->APB1ENR &= ~(1<<14))
#define SPI3_DS()			(RCC->APB1ENR &= ~(1<<15))

//SPI resets
#define SPI1_RESET()		do{ (RCC->APB2ENR |= (1<<12)); (RCC->APB2ENR &= ~(1<<12)); }while(0)
#define SPI2_RESET()		do{ (RCC->APB1ENR |= (1<<14)); (RCC->APB1ENR &= ~(1<<14)); }while(0)
#define SPI3_RESET()		do{ (RCC->APB1ENR |= (1<<15)); (RCC->APB1ENR &= ~(1<<15)); }while(0)






/*-----------------------------i2c structure---------------------*/

typedef struct
{
	_vo uint32_t I2C_CR1;		//I2C Control register 1    	Address Offset 0x00
	_vo uint32_t I2C_CR2;		//I2C Control register 2 		Address offset: 0x04
	_vo uint32_t I2C_OAR1;		//I2C Own address register 1	Address offset: 0x08
	_vo uint32_t I2C_OAR2;		//I2C Own address register 2 	Address offset: 0x0C
	_vo uint32_t I2C_DR;		//2C Data register				Address offset: 0x10
	_vo uint32_t I2C_SR1;		//I2C Status register 1			Address offset: 0x14
	_vo uint32_t I2C_SR2;		//I2C Status register 2			Address offset: 0x18
	_vo uint32_t I2C_CCR;		//I2C Clock control register 	Address offset: 0x1C
	_vo uint32_t I2C_TRISE;		//I2C TRISE register 			Address offset: 0x20
	_vo uint32_t I2C_FLTR;		//I2C FLTR register				Address offset: 0x24

}i2c_RegDef_t;



#define I2C1				((i2c_RegDef_t *)I2C1_BADDR)
#define I2C2				((i2c_RegDef_t *)I2C2_BADDR)
#define I2C3				((i2c_RegDef_t *)I2C3_BADDR)


//I2C peripheral clk enable
#define I2C1_EN()			(RCC->APB1ENR |= (1 << 21))
#define I2C2_EN()			(RCC->APB1ENR |= (1 << 22))
#define I2C3_EN()			(RCC->APB1ENR |= (1 << 23))


//I2C peripheral clk disable
#define I2C1_DS()			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_DS()			(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_DS()			(RCC->APB1ENR &= ~(1 << 23))


//I2C clk reset
#define I2C1_RESET()		do{ (RCC->APB1ENR |= (1<<21)); (RCC->APB1ENR &= ~(1<<21)); }while(0)
#define I2C2_RESET()		do{ (RCC->APB1ENR |= (1<<22)); (RCC->APB1ENR &= ~(1<<22)); }while(0)
#define I2C3_RESET()		do{ (RCC->APB1ENR |= (1<<23)); (RCC->APB1ENR &= ~(1<<23)); }while(0)

/*------------------------------------------------------------------------------------------------*/




/*-----------------------------usart structure---------------------*/

typedef struct
{
	_vo uint32_t USART_SR;		//Status register 						Address offset: 0x00
	_vo uint32_t USART_DR;		//Data register 						Address offset: 0x04
	_vo uint32_t USART_BRR;		//Baud rate register 					Address offset: 0x08
	_vo uint32_t USART_CR1;		//Control register 						Address offset: 0x0C
	_vo uint32_t USART_CR2;		//Control register 2 					Address offset: 0x10
	_vo uint32_t USART_CR3;		//Control register 3 					Address offset: 0x14
	_vo uint32_t USART_GTPR;	//Guard time and prescaler register		Address offset: 0x18

}usart_RegDef_t;


#define USART1  			((USART_RegDef_t*)USART1_BADDR)
#define USART2  			((USART_RegDef_t*)USART2_BADDR)
#define USART3  			((USART_RegDef_t*)USART3_BADDR)
#define UART4  				((USART_RegDef_t*)UART4_BADDR)
#define UART5  				((USART_RegDef_t*)UART5_BADDR)
#define USART6  			((USART_RegDef_t*)USART6_BADDR)


//Peripheral clk enable
#define USART1_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_EN() (RCC->APB2ENR |= (1 << 5))


//Peripheral clk disable

#define USART1_DS() (RCC->APB2ENR &= ~(1 <<4))
#define USART2_DS() (RCC->APB1ENR &= ~(1 <<17))
#define USART3_DS() (RCC->APB1ENR &= ~(1 <<18))
#define UART4_DS()  (RCC->APB1ENR &= ~(1 <<19))
#define UART5_DS()  (RCC->APB1ENR &= ~(1 <<20))
#define USART6_DS() (RCC->APB2ENR &= ~(1 <<5))




/*-----------------------------------------------------------------------------------------*/


/* misc. macros */
#define ENABLE 						1
#define DISABLE 					0
#define SET 						1
#define RESET 						0
#define GPIO_PIN_SET 				SET
#define GPIO_PIN_RESET 				RESET
#define HIGH						SET
//#define LOW							RESET {defined in GPIO_header file}

#include "gpios_stm32f4xx.h"
#include "spi_stm32f4xx.h"
#include "i2c_stm32f4xx.h"
#include "usart_stm32f407xx.h"


#endif /* INC_STM32F407XX_H_ */
