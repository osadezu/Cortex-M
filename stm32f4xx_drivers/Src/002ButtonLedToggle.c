/*
 * 002ButtonLedToggle.c
 *
 *  Created on: Aug 15, 2021
 *      Author: OSdZ
 */

#include <stdint.h>
#include "stm32f407xx_gpio.h"

void delay(uint32_t x)
{
	for (uint32_t i = 0; i < x; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLED;

	gpioLED.pGPIOx = GPIOD;
	gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12; // PD12 is green user LED on Discovery board
	gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLED.GPIO_PinConfig.GPIO_PinOSpeed = GPIO_OSPEED_HI; // Doesn't matter, just a test
	gpioLED.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;

	GPIO_ClkCtrl(GPIOD, ENABLE);
	GPIO_Init(&gpioLED);

	GPIO_Handle_t gpioButton;

	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0; // PA0 is user button on Discovery board
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;

	GPIO_ClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&gpioButton);

	while(1)
	{
		if (GPIO_ReadPin(GPIOA, GPIO_PIN_0))
		{
			delay(500000); // Button de-bouncing
			GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		}
	}

	return 0;
}
