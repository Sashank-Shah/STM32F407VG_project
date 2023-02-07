/*
 * spi_stm32f4xx.c
 *
 *  Created on: Jan 29, 2023
 *      Author: sashank.shah
 */
#include "spi_stm32f4xx.h"


static void spi_txe_interrupt(spi_Handler_t *pSPIHandle);
static void spi_rxe_interrupt(spi_Handler_t *pSPIHandle);
static void spi_ovr_interrupt(spi_Handler_t *pSPIHandle);
void SPI_TX_CLOSE(spi_Handler_t *pSPIHandle);



/******************* Peripheral clock setup API ******************
 * @fn				: SPI Perpherical Clock control
 *
 * @brief			: Enables the bus line i.e. enables the peripheral clock for SPI
 *
 * @param[in]		: Base address of SPI
 * @param[in]		: Enable or Disable Macro
 *
 * @return 			: None
 *
 * @note			: None
*/
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



/******************* SSI ENABLE  ******************
 * @fn				: The present bit has effect on the NSS pin in software slave select
 *
 * @brief			: when the SSM=1, SSI = 1 -> 	sets the NSS pin logic 1.
 *
 * @param[in]		: Base address of SPI
 * @param[in]		: Enable or Disable Macro
 *
 * @return 			: None
 *
 * @note			: None
*/

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




/******************* SPI ENABLE  ******************
 * @fn				: The SPI is enabled or switched ON
 *
 * @brief			: SPI peripheral enabled
 *
 * @param[in]		: Base address of SPI
 *
 * @return 			: None
 *
 * @note			: None
*/
void SPI_START(spi_RegDef_t *pSPI)
{
		pSPI->SPI_CR1 |= (1 << SPE);

}


/******************* SPI Disable ******************
 * @fn				: The SPI is disabled or switched OFF
 *
 * @brief			: SPI peripheral disabled
 *
 * @param[in]		: Base address of SPI
 *
 * @return 			: None
 *
 * @note			: None
*/

void SPI_STOP(spi_RegDef_t *pSPI)
{
	pSPI->SPI_CR1 &= ~(1 << SPE);

}


/******************* SPI SSOE ******************
 * @fn				: The present bit has effect on NSS pin Hardware slave management mode
 *
 * @brief			: When hardware slave management is used i.e. SSM = 0, NSS bit toggles wrt to SPE when SSOE is set.
 *
 * @param[in]		: Base address of SPI
 * @param[in]		: Enable or Disable
 *
 * @return 			: None
 *
 * @note			: None
*/

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



/******************* SPI initialization ******************
 * @fn				: SPI initialization API
 *
 * @brief			: Takes the SPI base address and sets the CR1 i.e. control register  bits according to the user input.
 *
 * @param[in]		: Base address of SPI
 *
 * @return 			: None
 *
 * @note			: None
*/

void SPI_init(spi_Handler_t *spiHandler_p)
{
	SPI_ClkCtrl(spiHandler_p->pSPI, ENABLE);  //Enable Peripheral clk for spi
	uint32_t bitmask = 0;

	//macros are define for the each register bits with corresponding bit name
	//cannot use ( bitmask_value << pinNUmber ) as used in gpios case.

	bitmask |= spiHandler_p->spiConf.SPI_MODE << MSTR;

	if(spiHandler_p->spiConf.SPI_COM == FULL_DUPLEX)
	{
		bitmask &= ~(1 << BIDI_MODE);		// Full duplex (2 line unidirectional)
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
	bitmask |= (spiHandler_p->spiConf.SPI_SM << SSM); 	//software slave
	spiHandler_p->pSPI->SPI_CR1 |= bitmask;  //assigning the bitmask value to the SPI Control Register 1.
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





/******************* SPI send data******************
 * @fn				: SPI send data API
 *
 * @brief			: It is a blocking API since it waits for the TX buffer to get empty.
 *
 * @param[in]		: Base address of SPI
 * @param[in]		: Base address of data to be sent i.e. user created TX buffer
 * @param[in]		: Tx buffer length in bytes.
 * @return 			: None
 *
 * @note			: It is a blocking API
*/
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



/******************* SPI receive data******************
 * @fn				: SPI receive data API
 *
 * @brief			: It is a blocking API since it waits for the RX buffer to get full.
 *
 * @param[in]		: Base address of SPI
 * @param[in]		: Base address of buffer to save the received bytes.
 * @param[in]		: RX buffer length in bytes.
 * @return 			: None
 *
 * @note			: It is a blocking API
*/
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
			RXBuffer++;
		}

	}
}



void SPI_IRQ(uint8_t irq, uint8_t ENorDS)
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


void SPI_IRQ_PRTY(uint8_t irq, uint8_t prty)
{
	uint8_t ipr_num = irq / 4;
	uint8_t ipr_sec = irq % 4;
	uint8_t ipr_position = ((8 * ipr_sec) + 4); //each section takes 8 bits but only upper nibbles are used
	//Hence, we have to shift the position by 8*section + 4 i.e. about 12 bits shift is required
	//NVIC_PTR->NVIC_PTY[ipr_num] |= (prty << ipr_position);
	*( NVIC_PRTY + ipr_num) |= (prty << ipr_position);

}

/******************* SPI send using interrupt ******************
 * @fn				: This API stores the values in the structure members
 *
 * @brief			: Contains structure members for the SPI ISR to use
 *
 * @param[in]		: Base address of SPI Handle structure
 * @param[in]		: Base address of buffer where data to be sent is stored
 * @param[in]		: TX buffer length in bytes.
 * @return 			: None
 *
 * @note			: Only used to assign values to the structure members
*/

uint8_t SPI_SendINT(spi_Handler_t *pSPIHandle, uint8_t *pData, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;		//store the state of the buffer
	if(state != SPI_BSY_TX)
	{
		pSPIHandle->pTXBuffer = pData; 			//passing the address of the user data
		pSPIHandle->TXlen = len;	   			//length of data
		pSPIHandle->TxState = SPI_BSY_TX; 		//tx busy state
		pSPIHandle->pSPI->SPI_CR2	|= (1 << TXEIE); //Enables the interrupt when TXE flag s set
	}

	return state;

}



/******************* SPI receive using interrupt ******************
 * @fn				: This API stores the values in the structure members
 *
 * @brief			: Contains structure members for the SPI ISR to use
 *
 * @param[in]		: Base address of SPI Handle structure
 * @param[in]		: Base address of buffer where data is to be stored
 * @param[in]		: RX buffer length in bytes.
 * @return 			: None
 *
 * @note			: Only used to assign values to the structure members
*/
uint8_t SPI_RecieveINT(spi_Handler_t *pSPIHandle, uint8_t *pRXdata, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BSY_RX)
	{
		pSPIHandle->pRXBuffer = pRXdata; 		//passing the address of the buffer (char array)  where we will store the received bytes
		pSPIHandle->RXlen = len;	   			//length of data
		pSPIHandle->RxState = SPI_BSY_RX; 		//RX busy state so that no other code can utilize spi
		pSPIHandle->pSPI->SPI_CR2	|= (1 << RXNEIE); //Enables the interrupt when RXE flag is set.
	}

	return state;

}


/******************* SPI ISR handle API ******************
 * @fn				: This API is used to store the
 *
 * @brief			: Contains structure members for the SPI ISR to use
 *
 * @param[in]		: Base address of SPI Handle structure
 * @param[in]		: Base address of buffer where data is to be stored
 * @param[in]		: RX buffer length in bytes.
 * @return 			: None
 *
 * @note			: Only used to assign values to the structure members
*/
void SPI_ISRHandling(spi_Handler_t *pSPIHandle)
{
	uint8_t temp1, temp2;

	temp1 = pSPIHandle->pSPI->SPI_SR & (1 << TXE);
	temp2 = pSPIHandle->pSPI->SPI_CR1 & (1 << TXEIE);

	//if Tx Buffer is empty which set the TXE flag and if Tx interrupt is enabled through TXEIE
	if(temp1 && temp2)
	{
		//code for TXE interrupt handle
		spi_txe_interrupt(pSPIHandle);
	}
	temp1 = pSPIHandle->pSPI->SPI_SR & (1 << RXNE);
	temp2 = pSPIHandle->pSPI->SPI_CR1 & (1 << RXNEIE);

	//if Rx Buffer is full which set the RXNE flag and if Rx interrupt is enabled through RXNEIE
	if(temp1 && temp2)
	{
		//code for RXNE interrupt handle
		spi_rxe_interrupt(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPI->SPI_SR & (1 << OVR);
	temp2 = pSPIHandle->pSPI->SPI_CR1 & (1 << ERRIE);

	//if overrun error occurred and error interrupt is enabled
	if(temp1 && temp2)
	{
		//code for overrun interrupt handle
		spi_ovr_interrupt(pSPIHandle);

	}




}



static void spi_txe_interrupt(spi_Handler_t *pSPIHandle)
{
	//checking for DFF size
	if(pSPIHandle->pSPI->SPI_CR1 & (1 << DFF))
	{
		//16 bit DFF size
		pSPIHandle->pSPI->SPI_DR = *((uint16_t *)pSPIHandle->pTXBuffer);
		pSPIHandle->TXlen--;
		pSPIHandle->TXlen--;
		(uint16_t*)pSPIHandle->pTXBuffer++;
	}
	else
	{
		//8 bit DFF size
		pSPIHandle->pSPI->SPI_DR = *(pSPIHandle->pTXBuffer);
		pSPIHandle->TXlen--;
		pSPIHandle->pTXBuffer++;

	}

	if(pSPIHandle->TXlen == 0)
	{
		//Tx Len is 0 so close the transmission
		SPI_TX_CLOSE(pSPIHandle);
		//SPI_APllication_callback();
	}
}

void SPI_TX_CLOSE(spi_Handler_t *pSPIHandle)
{
	pSPIHandle->pSPI->SPI_CR2 &= ~(1 << TXEIE); //clearing the TXEIE bit to disable the Interrupt
	pSPIHandle->pTXBuffer = NULL;				//Assignment NULL to pointer to TX buffer
	pSPIHandle->TXlen = 0;						//TX length to 0
	pSPIHandle->TxState = SPI_READY;			//Tx state to SPI_READY

}


static void spi_rxe_interrupt(spi_Handler_t *pSPIHandle)
{
	//checking for the DFF length
	if(pSPIHandle->pSPI->SPI_CR1 & (1 << DFF))
	{
		*((uint16_t*)pSPIHandle->pRXBuffer) = (uint16_t)pSPIHandle->pSPI->SPI_DR; //storing the data from the data register to the user RxBuffer

	}
}

static void spi_ovr_interrupt(spi_Handler_t *pSPIHandle)
{

}




