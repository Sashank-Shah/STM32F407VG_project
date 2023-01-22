/*
 * gpio.h
 *
 *  Author: Sashank Shah
 *
 */

#ifndef INC_GPIOS_STM32F4XX_H_
#define INC_GPIOS_STM32F4XX_H_
#include "STM32F407xx.h"
#include <stdint.h>

/*
 * @GPIO_PIN_Number
 */
#define P0			0
#define P1			1
#define P2			2
#define P3			3
#define P4			4
#define P5			5
#define P6			6
#define P7			7
#define P8			8
#define P9			9
#define P10			10
#define P11			11
#define P12			12
#define P13			13
#define P14			14
#define P15 		15

/*
 * @GPIO_PIN_MODE
 */

#define IN				 0
#define OUT				 1
#define ALTFUN			 2
#define ANALOG_M		 3

#define INT_FT			4		//interrupt at falling edge
#define INT_RT			5		//rising edge triggered
#define INT_RFT			6		//rising and falling triggered

/*
 * @GPIO_PIN_SPEED
 */

#define LOW						0
#define	MEDIUM					1
#define FAST      				2
#define VFAST					3

/*
 * @PULL_UP AND PULL_DOWN CONTROL
 */

#define NO_PU_PD				0
#define PULL_UP					1
#define PULL_DOWN        		2

/*
 * @GPIO_PIN_OUTPUT_TYPE
 */
#define PUSH_PULL				0
#define OPEN_DRAIN				1


/********************************** GPIO functionality configuration ********************/
typedef struct
{
	uint8_t pinNumber;				// @GPIO_PIN_Number //
	uint8_t pinMode;				// @GPIO_PIN_MODE   //
	uint8_t pinSpeed;				// @GPIO_PIN_SPEED  //
	uint8_t puPdControl;			// @PULL_UP AND PULL_DOWN CONTROL //
	uint8_t pinOutType;				// @GPIO_PIN_OUTPUT_TYPE //
	uint8_t pinAltMode;


}gpio_Config_t;

/****************************** Handle structure for GPIO pin *********************/
typedef struct
{
	gpio_RegDef_t *pGPIOx; 				//Pointer to hold the base address of the GPIO port to which the pin belongs
	gpio_Config_t gpioConf;				//GPIO pin configuration settings structure


}gpio_Handler_t;

/* 				GPIO Driver API Requirements are as follows:
 * 			    	1> GPIO initialization
 * 			    	2> Enable/Disable GPIO port clock
 * 			    	3> Read from a GPIO pin or Port
 * 			    	4> Write to a GPIO pin or Port
 * 			    	5> Configure Alt Functionality
 * 			    	6> Handle Interrupts
 *
 */



/******************* Peripheral clock setup API ******************/

void GPIO_PeriClkCtrl(gpio_RegDef_t *pGPIOx, uint8_t ENorDS);

/************* Initialization and De-initialization API **************/

void GPIO_init(gpio_Handler_t *pHandler);
void GPIO_DInit(gpio_RegDef_t *pGPIOx);


/*********** Data handling API *********************/

uint8_t GPIO_PinRead(gpio_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_portRead(gpio_RegDef_t *pGPIOx);
void GPIO_pinWrite(gpio_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_portWrite(gpio_RegDef_t *pGPIOx, uint16_t value);
void GPIO_pinToggle(gpio_RegDef_t *pGPIOx, uint8_t pinNumber);
void GPIO_AltFun();


/********* Api for IRQ configuration and ISR handling API *******************/

void GPIO_IRQ(uint8_t, uint8_t);
void GPIO_IRQ_PRTY(uint8_t, uint8_t);



#endif /* INC_GPIOS_STM32F4XX_H_ */
