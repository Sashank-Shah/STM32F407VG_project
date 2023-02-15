/*
 * msp.c
 *
 *  Created on: Feb 9, 2023
 *      Author: sashank.shah
 */

#include "stm32f4xx_hal.h"
void HAL_MspDeInit(void)
{
	//Setup priority grouping to 4 bits.
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	//Enable the BUS, MEM and UsageFault
	SCB->SHCSR |= (0x7 << 16);

	//system exception priority settings
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);



}


void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	//peripheral clk enable
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	//pin configuration using some
	GPIO_InitTypeDef gpioA;
	gpioA.Pin = GPIO_PIN_2;
	gpioA.Mode = GPIO_MODE_AF_PP;
	gpioA.Pull = GPIO_PULLUP;
	gpioA.Speed = GPIO_SPEED_FREQ_LOW;
	gpioA.Alternate = GPIO_AF7_USART2;

	HAL_GPIO_Init(GPIOA, &gpioA);

	gpioA.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &gpioA);

	//IRQ settings

	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_SetPriority(USART2_IRQn, 15, 0);


}
