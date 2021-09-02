/*
 * 007SPICmdHandling.c
 *
 *  Created on: Aug 29, 2021
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

#define NACK				0xA5
#define ACK					0xF5
#define LED_ON				1
#define LED_OFF				0

#define LED_PIN				9		// Arduino LED pin

//command codes
#define COMMAND_LED_CTRL	0x50
#define COMMAND_SENSOR_READ	0x51
#define COMMAND_LED_READ	0x52
#define COMMAND_PRINT		0x53
#define COMMAND_ID_READ		0x54

//arduino analog pins
#define ANALOG_PIN0			0
#define ANALOG_PIN1			1
#define ANALOG_PIN2			2
#define ANALOG_PIN3			3
#define ANALOG_PIN4			4

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

	// MISO
	spi2_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&spi2_gpio);

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

uint8_t verifyResponse(uint8_t ackByte)
{
	if(ackByte == ACK)
	{
		// ACK
		return 1;
	}
	return 0; // NACK
}

int main (void)
{

	gpio_config();
	spi_config();
	button_config();

	// Enable SS output
	SPI_SSOEControl(SPI2, SET);

	while(1)
	{
		uint8_t commandCode;
		uint8_t dummyByte = 0xFF;
		uint8_t dummyBuffer;
		uint8_t ackByte;	// Slave's acknowledge response
		uint8_t rcvByte;	// Slave's response
		uint8_t args[2];	// Command arguments

		SPI_Control(SPI2, ENABLE); // Enable SPI

		/////////////////////////
		// 1. COMMAND_LED_CTRL //
		/////////////////////////

		while(!GPIO_ReadPin(GPIOA, GPIO_PIN_0));	// Wait for button
		delay(500000); 								// Button debouncing

		commandCode = COMMAND_LED_CTRL;
		SPI_SendData(SPI2, &commandCode, 1);		// Send command

		// Do a dummy read to empty garbage buffer and clear RXNE
		SPI_ReceiveData(SPI2, &dummyBuffer, 1);

		// Send dummy byte to fetch response from slave
		SPI_SendData(SPI2, &dummyByte, 1);
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if(verifyResponse(ackByte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			SPI_SendData(SPI2, args, 2); 			// Send arguments

			// Print response
			printf("COMMAND_LED_CTRL: Done\n");
		}

		////////////////////////////
		// 2. COMMAND_SENSOR_READ //
		////////////////////////////

		while(!GPIO_ReadPin(GPIOA, GPIO_PIN_0));	// Wait for button
		delay(500000); 								// Button debouncing

		commandCode = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &commandCode, 1);		// Send command
		SPI_ReceiveData(SPI2, &dummyBuffer, 1);		 // Dummy read

		// Send dummy byte to fetch response from slave
		SPI_SendData(SPI2, &dummyByte, 1);
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if(verifyResponse(ackByte))
		{
			args[0] = ANALOG_PIN0;

			SPI_SendData(SPI2, args, 1);			// Send arguments
			SPI_ReceiveData(SPI2, &dummyBuffer, 1); // Dummy read

			delay(100000); 							// Wait for ADC

			// Send dummy byte to fetch response from slave
			SPI_SendData(SPI2, &dummyByte, 1);
			SPI_ReceiveData(SPI2, &rcvByte, 1);

			// Print response
			printf("COMMAND_SENSOR_READ: %d\n", rcvByte);

		}

		/////////////////////////
		// 3. COMMAND_LED_READ //
		/////////////////////////

		while(!GPIO_ReadPin(GPIOA, GPIO_PIN_0));	// Wait for button
		delay(500000); 								// Button debouncing

		commandCode = COMMAND_LED_READ;
		SPI_SendData(SPI2, &commandCode, 1);		// Send command
		SPI_ReceiveData(SPI2, &dummyBuffer, 1);		// Dummy read

		// Send dummy byte to fetch response from slave
		SPI_SendData(SPI2, &dummyByte, 1);
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if(verifyResponse(ackByte))
		{
			args[0] = LED_PIN;

			SPI_SendData(SPI2, args, 1);			// Send arguments
			SPI_ReceiveData(SPI2, &dummyBuffer, 1); // Dummy read

			// Send dummy byte to fetch response from slave
			SPI_SendData(SPI2, &dummyByte, 1);
			SPI_ReceiveData(SPI2, &rcvByte, 1);

			// Print response
			printf("COMMAND_LED_READ: %d\n", rcvByte);

		}

		//////////////////////
		// 4. COMMAND_PRINT //
		//////////////////////

		while(!GPIO_ReadPin(GPIOA, GPIO_PIN_0));	// Wait for button
		delay(500000); 								// Button debouncing

		commandCode = COMMAND_PRINT;
		SPI_SendData(SPI2, &commandCode, 1);		// Send command
		SPI_ReceiveData(SPI2, &dummyBuffer, 1);		// Dummy read

		// Send dummy byte to fetch response from slave
		SPI_SendData(SPI2, &dummyByte, 1);
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if(verifyResponse(ackByte))
		{
			char message[] = "Hello World!";
			uint8_t len = strlen(message);
			args[0] = len;

			SPI_SendData(SPI2, args, 1);			// Send arguments
			SPI_ReceiveData(SPI2, &dummyBuffer, 1); // Dummy read

			SPI_SendData(SPI2, (uint8_t*)message, len);

			// Print response
			printf("COMMAND_PRINT: %d\n", len);

		}

		////////////////////////
		// 5. COMMAND_ID_READ //
		////////////////////////

		while(!GPIO_ReadPin(GPIOA, GPIO_PIN_0));	// Wait for button
		delay(500000); 								// Button debouncing

		commandCode = COMMAND_ID_READ;
		SPI_SendData(SPI2, &commandCode, 1);		// Send command
		SPI_ReceiveData(SPI2, &dummyBuffer, 1);		// Dummy read

		// Send dummy byte to fetch response from slave
		SPI_SendData(SPI2, &dummyByte, 1);
		SPI_ReceiveData(SPI2, &ackByte, 1);

		if(verifyResponse(ackByte))
		{
			char id[11];
			id[10] = '\0';

			for(uint8_t i = 0; i < 10; i++)
			{
				SPI_SendData(SPI2, &dummyByte, 1);		// Send dummy bytes to clock the response back
				SPI_ReceiveData(SPI2, &id[i], 1); // Dummy read
			}

			// Print response
			printf("COMMAND_PRINT: %s\n", id);

		}

		// Close SPI Communication
		SPI_Control(SPI2, DISABLE);
	}

	return 0;
}
