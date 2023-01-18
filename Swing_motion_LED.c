/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Sashank Shah
 * @brief          : Swing motion using LED present on the STM32F407VG DISC1 - Board
 ******************************************************************************

* */

#include <stdint.h>
#include "gpios_stm32f4xx.h"


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


uint8_t pins[] = {P12, P13, P14, P15};

void delay()
{
	int i = 0;
	while(i<50000)
	{
		i++;
	}
}

void blink(uint8_t pin, gpio_Handler_t GPIO)
{
	GPIO.gpioConf.pinNumber = pin;
	GPIO_init(&GPIO);

	GPIO_pinWrite(GPIOD, pin, HIGH);
	delay();
	GPIO_pinWrite(GPIOD, pin, LOW);
	delay();
}

int main(void)
{
    /* Loop forever */
	GPIO_PeriClkCtrl(GPIOD, ENABLE); //Enable AHB1, peripheral clock of GPIO.

	gpio_Handler_t GPIO_D; //Creating a handler variable.

	GPIO_D.pGPIOx = GPIOD;

	//Assigning the values to the configuration structure
	//GPIO_D.gpioConf.pinNumber = P13;
	GPIO_D.gpioConf.pinMode = OUT;
	GPIO_D.gpioConf.pinOutType = PUSH_PULL;
	GPIO_D.gpioConf.puPdControl = NO_PU_PD;
	GPIO_D.gpioConf.pinSpeed = FAST;

	//passing the configuration structure members to the init function
	//Init function takes in the structure as input and utilizes the structure members to do neccessary configuration
	//GPIO_init(&GPIO_D);

	//Writing to the output pin

	uint8_t count  = 0;

	while(1)
	{
		if(count < 2)
		{
			for(int i = 0; i<4; i++)
			{
				blink(pins[i], GPIO_D);
			}
			count++;
			delay();
		}
		else
		{
			count--;
			for(int i = 4; i >= 0; i--)
			{
				blink(pins[i], GPIO_D);
			}
			delay();
		}

	}


}
