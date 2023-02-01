/*
 * main.c
 *
 *  Created on: Jan 24, 2023
 *      Author: sashank.shah
 */


#include <stdlib.h>
#include <string.h>
#include "STM32F407xx.h"

int main()
{





	/*---------------------------gpio configuration for SPI2 on GPIOB-------------------------------*/

	gpio_Handler_t SPI2_B; //OUTPUT PIN

	SPI2_B.pGPIOx = GPIOB;

	SPI2_B.gpioConf.pinMode = ALTFUN;
	SPI2_B.gpioConf.pinAltMode = 5;
	SPI2_B.gpioConf.pinSpeed = FAST;
	SPI2_B.gpioConf.pinOutType = PUSH_PULL;
	SPI2_B.gpioConf.puPdControl = NO_PU_PD;

	GPIO_PeriClkCtrl(GPIOB, ENABLE);


	//NSS
	SPI2_B.gpioConf.pinNumber = P12;
	GPIO_init(&SPI2_B);


	//SCK
	SPI2_B.gpioConf.pinNumber = P13;
	GPIO_init(&SPI2_B);

	//MISO
	SPI2_B.gpioConf.pinNumber = P14;
	GPIO_init(&SPI2_B);

	//MOSI
	SPI2_B.gpioConf.pinNumber = P15;
	GPIO_init(&SPI2_B);



	/*----------------------------SPI2 config--------------------------------------*/

	spi_Handler_t SPI2_Handle;

	SPI2_Handle.pSPI = SPI2;

	SPI2_Handle.spiConf.SPI_MODE = MASTER;
	SPI2_Handle.spiConf.SPI_DFF = BIT_FRAME_8;
	SPI2_Handle.spiConf.SPI_COM = FULL_DUPLEX;
	SPI2_Handle.spiConf.SPI_SM = SLAVE_SW_EN;
	SPI2_Handle.spiConf.SPI_CPHA = LOW;
	SPI2_Handle.spiConf.SPI_CPOL = LOW;
	SPI2_Handle.spiConf.SPI_Speed = CLK_DIV2;

	SPI_init(&SPI2_Handle);

	/*----------------------------------------------------------------------------*/

	char dataSend[] = "Jai Shri Ram";

	SSI_EN(SPI2, ENABLE); //Keeps the NSS pin to high

	SPI_START(SPI2); //Starts the SPI communication

	SPI_Send(SPI2, (uint8_t*)dataSend, strlen(dataSend));

	SPI_STOP(SPI2); //stops the SPI communication











	while(1);

}

