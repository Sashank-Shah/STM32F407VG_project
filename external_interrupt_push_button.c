/*
 * main.c
 *
 *  Created on: Jan 24, 2023
 *      Author: sashank.shah
 */


#include <stdlib.h>
#include <string.h>
#include "gpios_stm32f4xx.h"

int main()
{





	//Output pin configuration
	gpio_Handler_t GPIO_D; //OUTPUT PIN

	//initialize the value of structure members to 0
	memset(&GPIO_D, 0, sizeof(GPIO_D));
	GPIO_D.pGPIOx = GPIOD;

	GPIO_D.gpioConf.pinNumber = P12;
	GPIO_D.gpioConf.pinMode = OUT;
	GPIO_D.gpioConf.pinSpeed = FAST;
	GPIO_D.gpioConf.pinOutType = PUSH_PULL;
	GPIO_D.gpioConf.puPdControl = NO_PU_PD;
	GPIO_PeriClkCtrl(GPIOD, ENABLE);
	GPIO_init(&GPIO_D);

	//GPIO_pinWrite(GPIOD, P12, HIGH);

	//input pin configuration
	gpio_Handler_t GPIO_A; //INPUT PIN
	memset(&GPIO_A, 0, sizeof(GPIO_A));

	GPIO_A.pGPIOx = GPIOA;
	GPIO_A.gpioConf.pinNumber = P0;
	GPIO_A.gpioConf.pinMode = INT_FT;
	GPIO_A.gpioConf.pinOutType = PUSH_PULL;
	GPIO_A.gpioConf.pinSpeed  = FAST;
	GPIO_A.gpioConf.puPdControl = NO_PU_PD;


	GPIO_PeriClkCtrl(GPIOA, ENABLE);

	GPIO_init(&GPIO_A);


	GPIO_IRQ_PRTY(IRQ0, 4);
	GPIO_IRQ(IRQ0, ENABLE);


	while(1);

}

void EXTI0_IRQHandler(void)
{
	GPIO_ISRHandling(P0);
	for(int i=0; i<50000; i++);
	GPIO_pinToggle(GPIOD, P12);
}
