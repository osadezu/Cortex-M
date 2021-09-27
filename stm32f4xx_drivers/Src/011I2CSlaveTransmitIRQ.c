/*
 * 011I2CSlaveTransmitIRQ.c
 *
 *  Created on: Sep 27, 2021
 *      Author: OSdZ
 */

#include <stdio.h>
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

#define MY_ADDR			0x65

#define CMD_GET_LEN		0x51
#define CMD_GET_DATA	0x52

I2C_Handle_t i2c1_handle;

char transmit_data[32] = "Slave sending a test string!";

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
	i2c1_gpio.GPIO_PinConfig.GPIO_PinPuPd = GPIO_PUPD_PU;
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
	i2c1_handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
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
	gpio_config();
	i2c_config();
	button_config();

	// Enable SPI Peripheral
	I2C_Control(I2C1, ENABLE);

	// Enable interrupts
	I2C_IRQConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQConfig(IRQ_NO_I2C1_ER, ENABLE);

	// Enable IRQ events
	I2C_CallbackEventsControl(I2C1, ENABLE);


	while(1);
}

void I2C1_EV_IRQHandler(void)
{
	I2C_HandleEventIRQ(&i2c1_handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_HandleErrorIRQ(&i2c1_handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event)
{
	static uint8_t command = 0xff;
	static uint8_t pos = 0;

	if (event == I2C_EVENT_DATA_RCV)
	{
		// Receive instruction from master
		command = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

		printf("Received instruction: %X\n", command);
	}
	else if (event == I2C_EVENT_DATA_REQ)
	{
		if (command == CMD_GET_LEN)
		{
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen(transmit_data));

			printf("Sent length.\n");
		}
		else if (command == CMD_GET_DATA)
		{
			I2C_SlaveSendData(pI2CHandle->pI2Cx, transmit_data[pos++]);

			printf("Sent data.\n");
		}
	}
	else if (event == I2C_ERROR_AF)
	{
		printf("NACK.\n");

		command = 0xff;
		pos = 0;

	}
	else if (event == I2C_EVENT_STOP)
	{
		printf("Stop detected.\n");
	}
}
