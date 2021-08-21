/*
 * 003ExtButtonLedToggle.c
 *
 *  Created on: Aug 16, 2021
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

	gpioButton.pGPIOx = GPIOB;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPd = GPIO_PUPD_PU;

	GPIO_ClkCtrl(GPIOB, ENABLE);
	GPIO_Init(&gpioButton);

	while(1)
	{
		if (GPIO_ReadPin(GPIOB, GPIO_PIN_12) == BUTTON_PRESSED)
		{
			delay(500000); // Button de-bouncing
			GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		}
	}

	return 0;
}

