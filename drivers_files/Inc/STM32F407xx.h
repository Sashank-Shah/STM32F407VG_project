/*
 * 		STM32F407xx.h
 *
 *  	Created on: Oct 14, 2022
 *      Author: SASHANK SHAH
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
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

/**********GPIOx is indicated by Px (shorthand for PORTx)*********************/
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
	_vo uint32_t AHB2LPENR;	//RCC AHB2 peripheral clock enable in low power mode register	Address offset: 0x54
	_vo uint32_t AHB3LPENR;	//RCC AHB3 peripheral clock enable in low power mode register	Address offset: 0x58
	uint32_t RESD4;
	_vo uint32_t APB1LPENR;	//RCC APB1 peripheral clock enable in low power mode register	Address offset: 0x60
	_vo uint32_t APB2LPENR;	//RCC APB2 peripheral clock enabled in low power mode register	Address offset: 0x64
	uint32_t RESD5[2];

	_vo uint32_t BDCR;			//RCC Backup domain control register 	Address offset: 0x70
	_vo uint32_t CSR;			//RCC clock control & status register 	Address offset: 0x74
	uint32_t RESD6[2];
	_vo uint32_t SSCGR;		//RCC spread spectrum clock generation register 	Address offset: 0x80
	_vo uint32_t PLLI2SCFGR;	//RCC PLLI2S configuration register					Address offset: 0x84
	_vo uint32_t PLLSAICFGR;	//RCC PLL configuration register					Address offset: 0x88
	_vo uint32_t DCKCFGR;		//RCC Dedicated Clock Configuration Register		Address offset: 0x8C

}rcc_RegDef_t;

#define RCC 			((rcc_RegDef_t*)RCC_BADDR)

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
#endif /* INC_STM32F407XX_H_ */
