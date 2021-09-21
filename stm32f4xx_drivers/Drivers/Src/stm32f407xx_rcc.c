/*
 * stm32f407xx_rcc.c
 *
 *  Created on: Sep 11, 2021
 *      Author: OSdZ
 */

#include "stm32f407xx_rcc.h"

// Possible configurations of AHB prescaler RCC_CFGR->HPRE
uint16_t ahbPrescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};

// Possible configurations of APB1 prescaler RCC_CFGR->PPRE1
uint8_t apbxPrescaler[4] = {2, 4, 8, 16};

/** @brief 		Obtain APB1 clock frequency
 *
 * @param[in] 	pClkX		PCLK1 for APB1 clock or PCLK2 for APB2 clock
 *
 * @return		uint32_t	APB1 clock frequency
 */
uint32_t RCC_GetPclkFreq(uint8_t pClkX)
{
	uint32_t pClk1, sysClk, ahbP, apb1P;

	uint8_t bits;

	// Obtain system clock source

	bits = 0x3 & (RCC->CFGR >> RCC_CFGR_SWS);

	if (bits == 0)		// HSI
	{
		sysClk = HSI_CLK_FREQ;
	}
	else if (bits == 1)	// HSE
	{
		sysClk = HSE_CLK_FREQ;
	}
	else if (bits == 2)	// PLL
	{
		sysClk = RCC_GetPllClkFreq();
	}

	// Obtain AHB prescaler value
	bits = 0xF & (RCC->CFGR >> RCC_CFGR_HPRE);

	if (bits < 8)		// No prescaler
	{
		ahbP = 1;
	}
	else
	{
		ahbP = ahbPrescaler[bits - 8];
	}

	// Obtain APB1 prescaler value

	if (pClkX == PCLK1)
	{
		bits = 0x7 & (RCC->CFGR >> RCC_CFGR_PPRE1);
	}
	else if (pClkX == PCLK2)
	{
		bits = 0x7 & (RCC->CFGR >> RCC_CFGR_PPRE2);
	}

	if (bits < 4)		// No prescaler
	{
		apb1P = 1;
	}
	else
	{
		apb1P = apbxPrescaler[bits - 4];
	}

	pClk1 = (sysClk / ahbP) / apb1P;

	return pClk1;

}

/** @brief 		Obtain PLL clock frequency
 *
 * @param[in] 	none
 *
 * @return		uint32_t	PLL clock frequency
 */
uint32_t RCC_GetPllClkFreq(void)
{
	// TODO: implement PLL Clock getter
	return 0;
}
