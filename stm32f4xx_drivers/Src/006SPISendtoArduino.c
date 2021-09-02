/*
 * 006SPISendtoArduino.c
 *
 *  Created on: Aug 27, 2021
 *      Author: OSdZ
 */

#include <stdint.h>
#include <string.h>
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_spi.h"

/*
 * SPI Alt Fun for STM32F407
 *
 * SPI2_NSS 	PB12 (AF5)
 * SPI2_SCK		PB13 (AF5)
 * SPI2_MISO	PB14 (AF5)
 * SPI2_MOSI	PB15 (AF5)
 *
 */

void delay(uint32_t x)
{
	for (uint32_t i = 0; i < x; i++);
}

void gpio_config(void)
{
	GPIO_Handle_t spi2_gpio;

	spi2_gpio.pGPIOx = GPIOB;

	spi2_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	spi2_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_PIN_AF5;
	spi2_gpio.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
	spi2_gpio.GPIO_PinConfig.GPIO_PinPuPd = GPIO_PUPD_NO;
	spi2_gpio.GPIO_PinConfig.GPIO_PinOSpeed = GPIO_OSPEED_HI;

	// SCLK
	spi2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&spi2_gpio);

	// MOSI
	spi2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&spi2_gpio);

//	// MISO
//	spi2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
//	GPIO_Init(&spi2_gpio);

	// NSS
	spi2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&spi2_gpio);
}

void spi_config(void)
{
	SPI_Handle_t spi2;

	spi2.pSPIx = SPI2;

	spi2.SPI_Config.SPI_DeviceMode = SPI_MODE_MASTER;
	spi2.SPI_Config.SPI_BusConfig = SPI_BUS_FD;
	spi2.SPI_Config.SPI_ClkMode = SPI_CLKMODE_0;
	spi2.SPI_Config.SPI_SclkSpeed = SPI_SCLK_DIV8;	// 2 MHZ
	spi2.SPI_Config.SPI_DFF = SPI_DFF_8BIT;
	spi2.SPI_Config.SPI_SSM = SPI_SSM_DISABLE;		// Hardware Management

	SPI_Init(&spi2);
}

void button_config(void)
{
	GPIO_Handle_t gpioButton;

	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0; // PA0 is user button on Discovery board
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;

	GPIO_Init(&gpioButton);
}

int main (void)
{
	char user_data[] = "Hello World!";

	gpio_config();
	spi_config();
	button_config();

	// Enable SS output
	SPI_SSOEControl(SPI2, SET);

	while(1)
	{
		while(!GPIO_ReadPin(GPIOA, GPIO_PIN_0));	// Wait for button
		delay(500000); 								// Button debouncing

		// Enable SPI Peripheral
		SPI_Control(SPI2, ENABLE);

		// Indicate size of data to send
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		// Close SPI Communication
		SPI_Control(SPI2, DISABLE);
	}

	return 0;
}
