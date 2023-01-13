/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Sashank Shah
 * @brief          : The Code is targeted for generating HSI clock output on the gpio pin PA8
 ******************************************************************************
 */

#include <stdint.h>
#define RCC_BASE_ADDR 			 0x40023800UL
#define RCC_CFGR_ADDR   		 (RCC_BASE_ADDR + 0X08)

#define RCC_AHB1_CLK_EN			 (RCC_BASE_ADDR + 0X30)

#define GPIOA_BASE_ADDR			 0x40020000UL

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif



int main(void)
{
    /* Loop forever */
	//set HSI as clock select on MCO1
	uint32_t *CFGRptr = (uint32_t *)(RCC_CFGR_ADDR);
	*CFGRptr &= ~(0x3 << 21);
 
	//Set prescaler for  MCO1 as 2
	*CFGRptr |= (1 << 26);


	//enable peripheral clk for GPIOA
	uint32_t *AHB1 = (uint32_t *)RCC_AHB1_CLK_EN;
	*AHB1 |= (1 << 0);

	//set moder register for alternate function mode
	uint32_t *MODER = (uint32_t *)(GPIOA_BASE_ADDR + 0x00);
	*MODER &= ~(0X3 << 16);
	*MODER |= (0X2 << 16);

	//set alternate mode to AF0
	uint32_t *AFR = (uint32_t *)(GPIOA_BASE_ADDR + 0x24);
	*AFR &= ~(0XF << 0); // 0X0000




	for(;;);
}
