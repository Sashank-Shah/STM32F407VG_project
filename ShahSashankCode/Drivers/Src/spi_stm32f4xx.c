/*
 * spi_stm32f4xx.c
 *
 *  Created on: Jan 29, 2023
 *      Author: sashank.shah
 */
#include "spi_stm32f4xx.h"

void SPI_ClkCtrl(spi_RegDef_t *pSPI, uint8_t ENorDS)
{
	if(ENorDS == ENABLE)
	{
		if(pSPI == SPI1)
		{
			SPI1_EN();
		}
		else if(pSPI == SPI2)
		{
			SPI2_EN();
		}
		else if(pSPI == SPI3)
		{
			SPI3_EN();
		}

	}
	else
	{
		if(pSPI == SPI1)
		{
			SPI1_DS();
		}
		else if(pSPI == SPI2)
		{
			SPI2_DS();
		}
		else if(pSPI == SPI3)
		{
			SPI3_DS();
		}
	}


}

void SSI_EN(spi_RegDef_t *pSPI, uint8_t ENorDS)
{
	if(ENorDS == ENABLE)
	{
		pSPI->SPI_CR1 |= (1 << SSI);
	}
	else
	{
		pSPI->SPI_CR1 &= ~(1 << SSI);
	}
}


void SPI_START(spi_RegDef_t *pSPI)
{
		pSPI->SPI_CR1 |= (1 << SPE);

}



void SPI_STOP(spi_RegDef_t *pSPI)
{
	pSPI->SPI_CR1 &= ~(1 << SPE);

}



void SPI_SSOE(spi_RegDef_t *pSPI, uint8_t ENorDS)
{
	if(ENorDS == ENABLE)
	{
		pSPI->SPI_CR2 |= (1 << SSOE);
	}
	else
	{
		pSPI->SPI_CR2 &= ~(1 << SSOE);
	}
}



void SPI_init(spi_Handler_t *spiHandler_p)
{
	uint32_t bitmask = 0;

	bitmask |= spiHandler_p->spiConf.SPI_MODE << MSTR;

	if(spiHandler_p->spiConf.SPI_COM == FULL_DUPLEX)
	{
		bitmask &= ~(1 << BIDI_MODE);		// Full duplex
	}
	else if(spiHandler_p->spiConf.SPI_COM == HALF_DUPLEX)
	{
		bitmask |= (1 << BIDI_MODE);		// 1 line bidirectional (half duplex)
	}
	else if(spiHandler_p->spiConf.SPI_COM == SIMPLEX_RX)
	{
		bitmask &= ~(1 << BIDI_MODE); // Full duplex with TX disabled and RX enable (simplex receive)
		bitmask |= (1 << RX);		  // Enable only RX in Master and forces the SCL from the master to output clock even if the MOSI is disabled
	}

	bitmask |= (spiHandler_p->spiConf.SPI_Speed << BR); //Baud rate settings
	bitmask |= (spiHandler_p->spiConf.SPI_DFF << DFF);  //Data frame format
	bitmask |= (spiHandler_p->spiConf.SPI_CPOL << CPOL);
	bitmask |= (spiHandler_p->spiConf.SPI_CPHA << CPHA);
	spiHandler_p->pSPI->SPI_CR1 |= bitmask;
}


void SPI_DInit(spi_RegDef_t *pSPI)
{
	if(pSPI == SPI1)
	{
		SPI1_RESET();
	}
	else if(pSPI == SPI2)
	{
		SPI2_RESET();
	}
	else if(pSPI == SPI3)
	{
		SPI3_RESET();
	}
}


void SPI_Send(spi_RegDef_t *pSPI, uint8_t *data, uint32_t len)
{
	//loop till the data is empty
	while(len>0)
	{
		//wait until the TXE buffer is empty

		while(!(pSPI->SPI_SR & (1<<TXE)));

		//if DFF is set it means that the 16BIT data frame is selected
		if(pSPI->SPI_CR1 & (1 << DFF))
		{
			pSPI->SPI_DR = *((uint16_t *)data);
			len--;
			len--;
			(uint16_t *)data++;

		}
		//if DFF is 0 it means that the 8BIT data frame is selected
		else if ((pSPI->SPI_CR1 & (1 << DFF)) == 0)
		{
			pSPI->SPI_DR = *data;
			len--;
			data++;
		}

	}
}


void SPI_Recieve(spi_RegDef_t *pSPI, uint8_t *RXBuffer, uint32_t len)
{
	//loop till the data is empty
	while(len>0)
	{
		//wait until the RX buffer is full

		while(!(pSPI->SPI_SR & (1<<RXNE)));

		//if DFF is set it means that the 16BIT data frame is selected
		if(pSPI->SPI_CR1 & (1 << DFF))
		{
			*((uint16_t *)RXBuffer) = pSPI->SPI_DR;
			len--;
			len--;
			(uint16_t *)RXBuffer++;

		}
		//if DFF is 0 it means that the 8BIT data frame is selected
		else if ((pSPI->SPI_CR1 & (1 << DFF)) == 0)
		{
			*RXBuffer = pSPI->SPI_DR;
			len--;
			data++;
		}

	}
}



