/*
 * spi_stm32f4xx.h
 *
 *  Created on: Jan 29, 2023
 *      Author: sashank.shah
 */

#ifndef INC_SPI_STM32F4XX_H_
#define INC_SPI_STM32F4XX_H_
#include "STM32F407xx.h"
#include <stdint.h>

/*
 * @SPI Mode Select
 */

#define SLAVE 					0
#define MASTER 					1


/*
 * @SPI communication
 */

#define FULL_DUPLEX				1
#define HALF_DUPLEX				2
#define SIMPLEX_RX				3


/*
 * @Data frame format
 */

#define BIT_FRAME_8				0
#define BIT_FRAME_16			1


/*
 * @SPI CLK Speed
 */

#define CLK_DIV2				0
#define CLK_DIV4				1
#define CLK_DIV8				2
#define CLK_DIV16				3
#define CLK_DIV32				4
#define CLK_DIV64				5
#define CLK_DIV128				6
#define CLK_DIV256				7

/*
 * @SPI CPOL
 */

#define CPOL_LOW				0
#define CPOL_HIGH				1



/*
 * @SPI CPHA
 */

#define CPHA_LOW				0
#define CPHA_HIGH				1


/*
 * @SLAVE MANAGE
 */

#define SLAVE_SW_DS				0
#define SLAVE_SW_EN				1

/*
 * SPI_CR1 Bit fields
 */

#define CPHA				0
#define CPOL				1
#define MSTR				2
#define BR					3
#define SPE					6
#define LSB_FIRST			7
#define SSI					8
#define SSM					9
#define RX					10
#define DFF					11
#define CRC_NEXT			12
#define CRC_EN				13
#define BIDI_OE				14
#define BIDI_MODE			15


/*
 * SPI_CR2 Bit fields
 */

#define RXDMAEN				0
#define TXDMAEN				1
#define SSOE				2
#define FRF					4
#define ERRIE				5
#define RXNEIE				6
#define TXEIE				7


/*
 * SPI_SR Bit fields
 */
#define RXNE				0
#define TXE					1
#define CHSIDE				2
#define UDR					3
#define CRC_ERR				4
#define MODF				5
#define OVR					6
#define BSY					7
#define FRE					8

/*----------------- User entered data for SPI --------------------------*/
typedef struct
{
	uint8_t SPI_MODE;			//Master or Slave mode select
	uint8_t	SPI_COM;			//Half duplex, simplex, or full duplex (communication type)
	uint8_t	SPI_DFF;			//Data frame format (8 bits shift register or 16 bits shift register)
	uint8_t	SPI_CPHA;			//Phase select for SPI
	uint8_t SPI_CPOL;			//Polarity select for SPI
	uint8_t	SPI_SM;				//Either software or hardware slave management
	uint8_t	SPI_Speed;			//SPI speed setting

}spi_Config_t;




/*--------------- Handling structure--------------------------*/
typedef struct
{
	spi_RegDef_t *pSPI;
	spi_Config_t spiConf;

}spi_Handler_t;



void SPI_ClkCtrl(spi_RegDef_t *pSPI, uint8_t ENorDS);

/************* Initialization and De-initialization API **************/

void SPI_init(spi_Handler_t *pSPI);


void SPI_DInit(spi_RegDef_t *pSPI);

void SSI_EN(spi_RegDef_t *pSPI, uint8_t ENorDS);


void SPI_START(spi_RegDef_t *pSPI);
void SPI_STOP(spi_RegDef_t *pSPI);

/*----------------------SPI Read data coming to the device--------------------------------------*/
//void SPI_Read(spi_RegDef_t *pSPI);

/*----------------------SPI Send data from the device-------------------------------------*/
void SPI_Send(spi_RegDef_t *pSPI, uint8_t *data, uint32_t len);




/*----------------------SPI interrupt handling APIs-----------------*/


#endif /* INC_SPI_STM32F4XX_H_ */
