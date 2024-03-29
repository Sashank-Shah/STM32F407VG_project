/*
 * gpio_x.c
 *
 *
 *      Author: Sashank Shah
 *
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
		pHandler->pGPIOx->MODER &= ~(0x3 << 2*(pHandler->gpioConf.pinNumber));		   //0x3 i.e 11 to clear two bits at a time
		pHandler->pGPIOx->MODER |= temp;
	}

	//1.1 Setting for Interrupt Mode
	else
	{
		if(pHandler->gpioConf.pinMode == INT_FT)
		{
			EXTI->FTSR |= (1 << pHandler->gpioConf.pinNumber);
			EXTI->RTSR &= ~(1 << pHandler->gpioConf.pinNumber);

		}
		else if(pHandler->gpioConf.pinMode == INT_RT)
		{
			EXTI->RTSR |= (1 << pHandler->gpioConf.pinNumber);
			EXTI->FTSR &= ~(1 << pHandler->gpioConf.pinNumber);
		}
		else if(pHandler->gpioConf.pinMode == INT_RFT)
		{
			EXTI->FTSR |= (1 << pHandler->gpioConf.pinNumber);
			EXTI->RTSR |= (1 << pHandler->gpioConf.pinNumber);
		}

		//specify which gpio to use for the corresponding interrupt pin
		uint8_t arrNumber = 0;
		uint8_t bitNumber = 0;
		arrNumber = pHandler->gpioConf.pinNumber / 4;
		bitNumber = pHandler->gpioConf.pinNumber % 4;
		uint8_t gpioCode = GPIO_CODE(pHandler->pGPIOx);
		SYSCFG_EN();
		SYSCFG->EXTICR[arrNumber] |= (gpioCode << (4*bitNumber));

		//enable interrupt mask register
		EXTI->IMR |= (1 << pHandler->gpioConf.pinNumber);

	}

	/************* 2. Setting for Speed register. ********************/
	temp = 0; //reset the temp variable to use again

	temp = ((pHandler->gpioConf.pinSpeed) << 2*(pHandler->gpioConf.pinNumber));
	pHandler->pGPIOx->OSPEEDR &= ~(0x3 << 2*(pHandler->gpioConf.pinNumber));
	pHandler->pGPIOx->OSPEEDR |= temp;

	/************ 3.Setting for Pull-up and Pull-down register *****************/
	temp = 0;
	temp = ((pHandler->gpioConf.puPdControl) << 2*(pHandler->gpioConf.pinNumber));
	pHandler->pGPIOx->PUPDR &= ~(0x3 << 2*(pHandler->gpioConf.pinNumber)); //0x3 i.e 11 to clear two bits at a time
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


/******************* Enable the IRQ******************
 *
 * @fn				: Enables the Specific IRQ
 *
 * @brief			: None
 *
 * @param[in]		: IRQ Number
 * @param[in]		: Enable or Disable
 *
 * @return 			: None
 *
 * @note			: None
*/
void GPIO_IRQ(uint8_t irq, uint8_t ENorDS)
{
	if(ENorDS == ENABLE)
	{
		if(irq < 32)
		{
			*NVIC_ISER0 |= (1 << irq);
		}
		else if(irq > 31 && irq < 64)
		{
			*NVIC_ISER1 |= (1 << (irq%32));
		}
		else if(irq > 63 && irq < 96)
		{
			*NVIC_ISER2 |= (1 << (irq%64));
		}
	}
	else
	{
		if(irq < 32)
		{
			*NVIC_ICER0 |= (1<<irq);
		}
		else if(irq > 31 && irq < 64)
		{
			*NVIC_ICER1 |= (1<<(irq%32));
		}
		else if(irq > 63 && irq < 96)
		{
			*NVIC_ICER2 |= (1<<(irq%64));
		}
	}
}


/******************* Setting the priority ******************
 *
 * @fn				: Sets the priority for the IRQ
 *
 * @brief			: IRQ priority can be configured using the Function
 *
 * @param[in]		: IRQ Number
 * @param[in]		: IRQ Priority
 *
 * @return 			: None
 *
 * @note			: None
*/
void GPIO_IRQ_PRTY(uint8_t irq, uint8_t prty)
{
	uint8_t ipr_num = irq / 4;
	uint8_t ipr_sec = irq % 4;
	uint8_t ipr_position = ((8 * ipr_sec) + 4); //each section takes 8 bits but only upper nibbles are used
	//Hence, we have to shift the position by 8*section + 4 i.e. about 12 bits shift is required
	//NVIC_PTR->NVIC_PTY[ipr_num] |= (prty << ipr_position);
	*( NVIC_PRTY + ipr_num) |= (prty << ipr_position);

}

void GPIO_ISRHandling(uint8_t pinNumber)
{
	//Check the pending register is set or cleared by the interrupt
	if(EXTI->PR & (1 << pinNumber))
	{
		//To clear the pr we have to set the bit
		EXTI->PR |= (1<<pinNumber); //clear the interrupt corresponding to the pr register
	}
}


