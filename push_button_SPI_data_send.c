/*
 * main.c
 *
 *  Created on: Jan 24, 2023
 *      Author: sashank.shah
 */


#include <stdlib.h>
#include <string.h>
#include "STM32F407xx.h"


void delay()
{
	for(uint32_t i = 0; i<50000/2; i++);
}

void button()
{
	//input pin configuration
		gpio_Handler_t GPIO_A; //INPUT PIN


		GPIO_A.pGPIOx = GPIOA;
		GPIO_A.gpioConf.pinNumber = P0;
		GPIO_A.gpioConf.pinMode = IN;
		GPIO_A.gpioConf.pinOutType = PUSH_PULL;
		GPIO_A.gpioConf.pinSpeed  = FAST;
		GPIO_A.gpioConf.puPdControl = NO_PU_PD;


		GPIO_PeriClkCtrl(GPIOA, ENABLE);

		GPIO_init(&GPIO_A);
}


int main()
{


	button();
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
	//SPI2_B.gpioConf.pinNumber = P14;
	//GPIO_init(&SPI2_B);

	//(only sending data)
	//MOSI
	SPI2_B.gpioConf.pinNumber = P15;
	GPIO_init(&SPI2_B);



	/*----------------------------SPI2 config--------------------------------------*/

	spi_Handler_t SPI2_Handle;

	SPI2_Handle.pSPI = SPI2;

	SPI2_Handle.spiConf.SPI_MODE = MASTER;
	SPI2_Handle.spiConf.SPI_DFF = BIT_FRAME_8;
	SPI2_Handle.spiConf.SPI_COM = FULL_DUPLEX;
	SPI2_Handle.spiConf.SPI_SM = SLAVE_SW_DS; //Hardware ss
	SPI2_Handle.spiConf.SPI_CPHA = LOW;
	SPI2_Handle.spiConf.SPI_CPOL = LOW;
	SPI2_Handle.spiConf.SPI_Speed = CLK_DIV8; //clk frequency 2MHz

	SPI_init(&SPI2_Handle);

	/*----------------------------------------------------------------------------*/

	char dataSend[] = "Jai Shri Ram";


	SPI_SSOE(SPI2, ENABLE); //NSS toggles in accordance with the SPE bit and is managed by the hardware


	while(1)
	{
		while(!(GPIO_PinRead(GPIOA, P0))); //block the code if button is not pressed

		delay();
		//if button is presses the control comes out of the while loop

		SPI_START(SPI2); //Starts the SPI communication

		uint8_t dataLenght = strlen(dataSend);
		SPI_Send(SPI2, &dataLenght, 1); //1 byte of data containing the length

		SPI_Send(SPI2, (uint8_t*)dataSend, strlen(dataSend)); //sending the data

		while(SPI2_Handle.pSPI->SPI_SR & (1 << BSY)); //wait till the TX buffer is empty

		SPI_STOP(SPI2); //stops the SPI communication

	}

}

