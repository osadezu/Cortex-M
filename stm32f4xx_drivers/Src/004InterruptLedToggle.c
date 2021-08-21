/*
 * 004InterruptLedToggle.c
 *
 *  Created on: Aug 21, 2021
 *      Author: OSdZ
 */


#include <stdint.h>
#include "stm32f407xx_gpio.h"

#define LOW				0
#define BUTTON_PRESSED	LOW

void delay(uint32_t x)
{
	for (uint32_t i = 0; i < x; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLED;

	gpioLED.pGPIOx = GPIOA;
	gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
	gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLED.GPIO_PinConfig.GPIO_PinOSpeed = GPIO_OSPEED_HI; // Doesn't matter, just a test
	gpioLED.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;

	GPIO_ClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&gpioLED);

	GPIO_Handle_t gpioButton;

	gpioButton.pGPIOx = GPIOD;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_EX_FT;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPd = GPIO_PUPD_PU;

	GPIO_ClkCtrl(GPIOD, ENABLE);
	GPIO_Init(&gpioButton);

	GPIO_IRQConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, IRQ_PRI_3);

	while(1);

	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	delay(500000); // Button de-bouncing
	GPIO_TogglePin(GPIOA, GPIO_PIN_8);
	GPIO_IRQReset(GPIO_PIN_5);
}

