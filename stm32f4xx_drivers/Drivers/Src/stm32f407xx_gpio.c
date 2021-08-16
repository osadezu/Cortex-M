/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Aug 12, 2021
 *      Author: oscar
 */

#include "stm32f407xx_gpio.h"

/*****************************
 *  API Function Definitions *
 *****************************/

/** @brief 		Enable or disable clock for GPIO port
 *
 * @param[in] 	*pGPIOx Base address of the GPIO port
 * @param[in] 	state	ENABLE or DISABLE
 *
 * @return		none
 */
void GPIO_ClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t state)
{
	if (state == ENABLE)
	{
		if (pGPIOx == GPIOA) {
			GPIOA_CLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_CLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_CLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_CLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_CLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_CLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_CLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_CLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_CLK_EN();
		}
	}
	else if (state == DISABLE)
	{
		if (pGPIOx == GPIOA) {
			GPIOA_CLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_CLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_CLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_CLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_CLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_CLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_CLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_CLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_CLK_DI();
		}
	}
}

/** @brief 		Initialize GPIO port
 *
 * @param[in] 	*pGPIOHandle	GPIO handle structure with port base address and pin configuration
 *
 * @return		none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t bits = 0;

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Configure pin mode: GPIOx_MODER
		bits = 0x3 & pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))); // Clear
		pGPIOHandle->pGPIOx->MODER |= (bits << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))); // Set
	}
	else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode > GPIO_MODE_ANALOG)
	{
		// TODO: Configure interrupt modes
	}

	// Configure output type: GPIOx_OTYPER
	bits = 0x1 & pGPIOHandle->GPIO_PinConfig.GPIO_PinOType;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear
	pGPIOHandle->pGPIOx->OTYPER |= (bits << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Set

	// Configure output speed: GPIOx_OSPEEDR
	bits = 0x3 & pGPIOHandle->GPIO_PinConfig.GPIO_PinOSpeed;
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))); // Clear
	pGPIOHandle->pGPIOx->OSPEEDR |= (bits << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))); // Set

	// Configure pull-up / pull-down: GPIOx_PUPDR
	bits = 0x3 & pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPd;
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))); // Clear
	pGPIOHandle->pGPIOx->PUPDR |= (bits << (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))); // Set

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN)
	{
		// Configure alternate function: GPIOx_AFRx
		bits = 0xF & pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode;
		uint8_t afrx = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8; // Which AFRx
		uint8_t afrxY = 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8); // Which position
		pGPIOHandle->pGPIOx->AFR[afrx] &= ~(0xF << afrxY); // Clear
		pGPIOHandle->pGPIOx->AFR[afrx] |= (bits << afrxY); // Set
	}
}

/** @brief 		De-initialize GPIO port
 *
 * @param[in] 	*pGPIOx	GPIO base address
 *
 * @return		none
 */
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx) // Return port configuration to reset state
{
	if (pGPIOx == GPIOA) {
		GPIOA_RST();
	} else if (pGPIOx == GPIOB) {
		GPIOB_RST();
	} else if (pGPIOx == GPIOC) {
		GPIOC_RST();
	} else if (pGPIOx == GPIOD) {
		GPIOD_RST();
	} else if (pGPIOx == GPIOE) {
		GPIOE_RST();
	} else if (pGPIOx == GPIOF) {
		GPIOF_RST();
	} else if (pGPIOx == GPIOG) {
		GPIOG_RST();
	} else if (pGPIOx == GPIOH) {
		GPIOH_RST();
	} else if (pGPIOx == GPIOI) {
		GPIOI_RST();
	}
}

/** @brief 		Read from input pin
 *
 * @param[in] 	*pGPIOx	GPIO base address
 * @param[in] 	pin		Pin to read
 *
 * @return		uint8_t Pin state
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t pin)
{
	return (uint8_t)(0x1 & ((pGPIOx->IDR) >> pin));
}

/** @brief 		Read from input port
 *
 * @param[in] 	*pGPIOx	GPIO base address
 *
 * @return		uint16_t Port state
 */
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
}

/** @brief 		Write to output pin
 *
 * @param[in] 	*pGPIOx	GPIO base address
 * @param[in] 	pin		Pin to write to
 * @param[in] 	value	Value to write
 *
 * @return		none
 */
void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t pin, uint8_t value)
{
	pGPIOx->ODR &= ~(0x1 << pin);
	pGPIOx->ODR |= ((0x1 & value) << pin);
}

/** @brief 		Write to output port
 *
 * @param[in] 	*pGPIOx	GPIO base address
 * @param[in] 	value	Value to write
 *
 * @return		none
 */
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/** @brief 		Toggle Output pin
 *
 * @param[in] 	*pGPIOx	GPIO base address
 * @param[in] 	pin		Pin to toggle
 *
 * @return		none
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pin)
{
	pGPIOx->ODR ^= (0x1 << pin);
}


// IRQ Configuration and ISR Handling

void GPIO_IRQConfig(uint8_t irqNumber, uint8_t irqPriority, uint8_t state) // Configure port IRQ
{

}

void GPIO_IRQHandle(uint8_t pin) // Handle port IRQ
{

}
