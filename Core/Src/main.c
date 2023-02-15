/*
 * main.c
 *
 *  Created on: Feb 9, 2023
 *      Author: sashank.shah
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include <string.h>

void SytemClockConfig(void);
void exception_led(void);
int main()
{

	char *str = "Shri Krishna Govind hare Murari\r\n";
	int len = strlen(str);
	SytemClockConfig();
	HAL_Init();

	//uart configuration starts
	UART_HandleTypeDef uart2_handle; //hanlder type variable

	uart2_handle.Instance = USART2; //usart to base address
	uart2_handle.Init.BaudRate = 115200;
	uart2_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart2_handle.Init.Mode = UART_MODE_TX_RX;
	uart2_handle.Init.OverSampling = UART_OVERSAMPLING_16;
	uart2_handle.Init.Parity = UART_PARITY_NONE;
	uart2_handle.Init.StopBits = UART_STOPBITS_1;
	uart2_handle.Init.WordLength = UART_WORDLENGTH_8B;

	//initialze usart with all the user inputs
	if(HAL_UART_Init(&uart2_handle) != HAL_OK)
	{
		//if uart is not properly initailzed then call any exception exmaple blink an red led
		exception_led();
	}

	HAL_UART_Transmit(&uart2_handle, (uint8_t*)str, (uint16_t)len, HAL_MAX_DELAY);
	while(1);

	return 0;
}

void SytemClockConfig(void)
{
	//use HSI
}

void exception_led(void)
{
	while(1);
}
