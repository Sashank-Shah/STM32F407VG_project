/*
 * gpio_x.c
 *
 *  Created on: Dec 1, 2022
 *      Author: owner
 */

#include "gpios_stm32f4xx.h"

/******************* Peripheral clock setup API ******************
 * @fn				: GPIO Perpherical Clock control
 *
 * @brief			: Enables the bus line i.e. enables the peripheral clock for gpio port
 *
 * @param[in]		: Base address of gpio
 * @param[in]		: Enable or Disable Macro
 *
 * @return 			: None
 *
 * @note			: None
*/

void GPIO_PeriClkCtrl(gpio_RegDef_t *pGPIOx, uint8_t ENorDS)
{
	if(ENorDS == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_CLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_CLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_CLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_CLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_CLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_CLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_CLK_EN();
		}

	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_DIS();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_DIS();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_CLK_DIS();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_CLK_DIS();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_CLK_DIS();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_CLK_DIS();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_CLK_DIS();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_CLK_DIS();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_CLK_DIS();
		}


	}
}

/************* Initialization API **************
* @fn				: GPIO initialize control
*
* @brief			: Use to configure GPIO pin such as - MODES, Speed, etc.
*
* @param[in]		: Address of Handler variable of type gpio_Handler_t
* @param[in]		:
*
* @return 			: None
*
* @note			: None
*/
void GPIO_init(gpio_Handler_t *pHandler)
{
	uint32_t temp = 0;  //captures the value that needs to be updated to the specific register

	/************ 1. Setting Modes of gpio **************************/
	//We have two mode sections : non-interrupt modes and interrupt modes

	if(pHandler->gpioConf.pinMode <= ANALOG_M)
	{
		temp = ((pHandler->gpioConf.pinMode) << 2*(pHandler->gpioConf.pinNumber)); //multiply 2 because each gpio port pin takes 2 bits.
		pHandler->pGPIOx->MODER &= ~(0x3 << pHandler->gpioConf.pinNumber);		   //0x3 i.e 11 to clear two bits at a time
		pHandler->pGPIOx->MODER |= temp;
	}

	//1.1 Setting for Interrupt Mode
	else
	{

	}

	/************* 2. Setting for Speed register. ********************/
	temp = 0; //reset the temp variable to use again

	temp = ((pHandler->gpioConf.pinSpeed) << 2*(pHandler->gpioConf.pinNumber));
	pHandler->pGPIOx->OSPEEDR &= ~(0x3 << pHandler->gpioConf.pinNumber);
	pHandler->pGPIOx->OSPEEDR |= temp;

	/************ 3.Setting for Pull-up and Pull-down register *****************/
	temp = 0;
	temp = ((pHandler->gpioConf.puPdControl) << 2*(pHandler->gpioConf.pinNumber));
	pHandler->pGPIOx->PUPDR &= ~(0x3 << pHandler->gpioConf.pinNumber); //0x3 i.e 11 to clear two bits at a time
	pHandler->pGPIOx->PUPDR |= temp;

	/*********** 4. Setting for Output Type *****************************/
	temp = 0;
	temp = ((pHandler->gpioConf.pinOutType) << (pHandler->gpioConf.pinNumber));
	pHandler->pGPIOx->OTYPER &= ~(0x1 << pHandler->gpioConf.pinNumber); //0x1 to clear single bit at a time
	pHandler->pGPIOx->OTYPER |= temp;

	/*********** 5. Alternate Functionality Mode **********************/

	if(pHandler->gpioConf.pinMode == ALTFUN)
	{
		uint8_t temp1, temp2;

		temp1 = pHandler->gpioConf.pinNumber / 8; //gives 1 for pinNumber > 8 and 0 for pinNumber < 8 (useful for AFR register)
		temp2 = pHandler->gpioConf.pinNumber % 8; //useful for deciding register bit number to change

		pHandler->pGPIOx->AFR[temp1] &= ~(0x4 << (4 * temp2)); //0x4 to clear 4 bits at a time
		pHandler->pGPIOx->AFR[temp1] |= pHandler->gpioConf.pinAltMode << (4 * temp2); //4 multiplied because 1 pin takes 4 bits of AFR[x]
	}
}



/************* De-initialization API **************
* @fn				: GPIO De-initialize control
*
* @brief			: Use to Reset the GPIO register values
*
* @param[in]		: Address of GPIOx variable which is of type gpio_RegDef_t
* @param[in]		:
*
* @return 			: None
*
* @note			: None
*/

void GPIO_DInit(gpio_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_RESET();
	}

}


/******************* Pin Read API ******************
 *
 * @fn				: GPIOx input pin read
 *
 * @brief			: Read the pin state, when pin is set as input
 *
 * @param[in]		: GPIOx i.e GPIO base addr
 * @param[in]		: pin number to read
 *
 * @return 			: 0 or 1 i.e. pin read LOW or HIGH
 *
 * @note			: None
*/

uint8_t GPIO_PinRead(gpio_RegDef_t *pGPIOx, uint8_t pinNumber)
{

	uint8_t val = (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001); //first shift the required pin IDR bit to LSB and check LSB
	return val;

}

/******************* GPIO PORT Read API ******************
 *
 * @fn				: GPIOx input port read
 *
 * @brief			: Read the whole port state, when set as input
 *
 * @param[in]		: GPIO base addr -> GPIOx
 * @param[in]		:
 *
 * @return 			: 0 or 1
 *
 * @note			: None
*/

uint16_t GPIO_portRead(gpio_RegDef_t *pGPIOx)
{
	uint16_t val = (uint16_t)pGPIOx->IDR;
	return val;
}

/******************* GPIO pin write API ******************
 *
 * @fn				: GPIOx output pin write
 *
 * @brief			: write HIGH or LOW logic on output gpio pin
 *
 * @param[in]		: GPIOx -> GPIO base addr
 * @param[in]		: pin number to write
 * @param[in]		: Logic HIGH or LOW
 *
 * @return 			: None
 *
 * @note			: None
*/

void GPIO_pinWrite(gpio_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	if(value == HIGH)
	{
		pGPIOx->ODR |= (1 << pinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << pinNumber);
	}
}

/******************* GPIO port write API ******************
 *
 * @fn				: GPIOx output port write
 *
 * @brief			: write HIGH or LOW logic on output gpio port
 *
 * @param[in]		: GPIOx -> GPIO base addr
 * @param[in]		: value to write to the port
 *
 * @return 			: None
 *
 * @note			: None
*/

void GPIO_portWrite(gpio_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/******************* GPIO pin toggle API ******************
 *
 * @fn				: GPIOx output pin write
 *
 * @brief			: Toggles output gpio pin
 *
 * @param[in]		: GPIOx -> GPIO base addr
 * @param[in]		: pin number to toggle
 *
 * @return 			: None
 *
 * @note			: None
*/


void GPIO_pinToggle(gpio_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= (1 << pinNumber);
}
void GPIO_AltFun();

/********* Api for IRQ configuration and ISR handling API *******************/

void GPIO_IRQConfig();
void GPIO_IRQHandle();



