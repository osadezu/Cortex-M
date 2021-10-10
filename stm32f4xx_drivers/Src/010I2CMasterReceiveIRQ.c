/*
 * 010I2CMasterReceiveIRQ.c
 *
 *  Created on: Sep 24, 2021
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
#define DEST_ADDR		0x68

#define CMD_GET_LEN		0x51
#define CMD_GET_DATA	0x52

I2C_Handle_t i2c1_handle;
uint8_t rxFlag;

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
	uint8_t command, len;
	uint8_t receive_buffer[32];

	receive_buffer[0] = '\0';

	gpio_config();
	i2c_config();
	button_config();

	// Enable I2C Peripheral
	I2C_Control(I2C1, ENABLE);

	// Enable interrupts
	I2C_IRQConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQConfig(IRQ_NO_I2C1_ER, ENABLE);

	while(1)
	{
		while(!GPIO_ReadPin(GPIOA, GPIO_PIN_0));	// Wait for button
		delay(500000); 								// Button debouncing

		// Check how many bytes will be sent
		command = CMD_GET_LEN;

		while(I2C_MasterSendDataIRQ(&i2c1_handle, DEST_ADDR, &command, 1, I2C_NO_STOP) != I2C_READY);
		while(I2C_MasterReceiveDataIRQ(&i2c1_handle, DEST_ADDR, &len, 1, I2C_NO_STOP) != I2C_READY);

		// Request data
		command = CMD_GET_DATA;

		while(I2C_MasterSendDataIRQ(&i2c1_handle, DEST_ADDR, &command, 1, I2C_NO_STOP) != I2C_READY);

		// TODO: Print from RX_DONE callback to remove this workaround
		rxFlag = RESET;

		while(I2C_MasterReceiveDataIRQ(&i2c1_handle, DEST_ADDR, receive_buffer, len, I2C_WITH_STOP) != I2C_READY);

		while(rxFlag != SET);		// Hang until Rx is done
		receive_buffer[len] = '\0'; // Terminate string with null

		printf("%s\n", (char*)receive_buffer);

		rxFlag = RESET;

	}
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
	if (event == I2C_EVENT_TX_DONE)
	{
		printf("Transmission complete.\n");
	}
	else if (event == I2C_EVENT_RX_DONE)
	{
		printf("Reception complete.\n");

		rxFlag = SET;
	}
	else if (event == I2C_ERROR_AF)
	{
		printf("Error: ACK Failure.\n");

		// Close communication
		I2C_CloseTransmission(pI2CHandle);

		// Generate Stop
		I2C_IssueStop(pI2CHandle->pI2Cx);

		// Hang here
		while(1);
	}
}
