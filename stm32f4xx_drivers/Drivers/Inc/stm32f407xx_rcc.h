/*
 * stm32f407xx_rcc.h
 *
 *  Created on: Sep 11, 2021
 *      Author: OSdZ
 */

#ifndef INC_STM32F407XX_RCC_H_
#define INC_STM32F407XX_RCC_H_

#include "stm32f407xx.h"

#define PCLK1	1	// Internal APB1 clock
#define PCLK2	2	// Internal APB2 clock

uint32_t RCC_GetPclkFreq(uint8_t pClkX);		// Obtain APB1 clock frequency
uint32_t RCC_GetPllClkFreq(void);		// Obtain PLL clock frequency

#endif /* INC_STM32F407XX_RCC_H_ */
