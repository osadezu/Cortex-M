/*
 * 008I2CMasterTransmit.c
 *
 *  Created on: Sep 21, 2021
 *      Author: OSdZ
 */

#include <stdint.h>
#include <string.h>
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_i2c.h"

/*
 * I2C Alt Fun for STM32F407
 *
 * I2C1_SCL		PB6 (AF4)
 * I2C1_SDA		PB7 (AF4)
 *
 */

#define MY_ADDR		0x65
#define DEST_ADDR	0x68

I2C_Handle_t i2c1_handle;

void delay(uint32_t x)
{
	for (uint32_t i = 0; i < x; i++);
}

void gpio_config(void)
{
	GPIO_Handle_t i2c1_gpio;

	i2c1_gpio.pGPIOx = GPIOB;

	i2c1_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	i2c1_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_PIN_AF4;
	i2c1_gpio.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_OD;
	i2c1_gpio.GPIO_PinConfig.GPIO_PinPuPd = GPIO_PUPD_NO;
	i2c1_gpio.GPIO_PinConfig.GPIO_PinOSpeed = GPIO_OSPEED_HI;

	// SCL
	i2c1_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&i2c1_gpio);

	// SDA
	i2c1_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&i2c1_gpio);

}

void i2c_config(void)
{
	i2c1_handle.pI2Cx = I2C1;

	i2c1_handle.I2C_Config.I2C_AckControl = I2C_ACK_EN;
	i2c1_handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SM;
	i2c1_handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;		// Doesn't matter for this master example
	i2c1_handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;	// Doesn't matter for Sm

	I2C_Init(&i2c1_handle);
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
	char user_data[] = "Hello, this is a test string!\n";

	gpio_config();
	i2c_config();
	button_config();

	// Enable I2C Peripheral
	I2C_Control(I2C1, ENABLE);

	while(1)
	{
		while(!GPIO_ReadPin(GPIOA, GPIO_PIN_0));	// Wait for button
		delay(500000); 								// Button debouncing

		// Send data
		I2C_MasterSendData(&i2c1_handle, DEST_ADDR, (uint8_t*)user_data, strlen(user_data), I2C_WITH_STOP);

	}

	return 0;
}
