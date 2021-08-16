/*
 * stm32f407xx_gpio.h
 *
 *  Created on: Aug 12, 2021
 *      Author: oscar
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include "stm32f407xx.h"


// GPIO Register States

/// @GPIO_PIN_MODE
/// GPIO pin possible modes
#define GPIO_MODE_IN		0	// Input
#define GPIO_MODE_OUT		1	// Output
#define GPIO_MODE_ALTFUN	2	// Alternate Function
#define GPIO_MODE_ANALOG	3	// Analog
#define GPIO_MODE_EX_RT		4	// External Interrupt: Rising Trigger
#define GPIO_MODE_EX_FT		5	// External Interrupt: Falling Trigger
#define GPIO_MODE_EX_RFT	6	// External Interrupt: Rising and Falling Trigger

/// @GPIO_PIN_OTYPE
/// GPIO pin output types
#define GPIO_OTYPE_PP		0	// Push-pull
#define GPIO_OTYPE_OD		1	// Open-drain

/// @GPIO_PIN_OSPEED
/// GPIO pin output speed
#define GPIO_OSPEED_LOW		0	// Low speed
#define GPIO_OSPEED_MED		1	// Medium speed
#define GPIO_OSPEED_HI		2	// High speed
#define GPIO_OSPEED_VH		3	// Very high speed

/// @GPIO_PIN_PUPD
/// GPIO pin pull-up / pull-down
#define GPIO_PUPD_NO		0	// No pull-up or pull-down
#define GPIO_PUPD_PU		1	// Pull-up
#define GPIO_PUPD_PD		2	// Pull-down

/// @GPIO_PINS
/// GPIO port pins
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/*************************************
 *  Peripheral Structure Definitions *
 *************************************/

// Configuration Structure for GPIO pins

typedef struct {
	uint8_t GPIO_PinNumber;			//> Value from @GPIO_PINS
	uint8_t GPIO_PinMode;			//> Value from @GPIO_PIN_MODE
	uint8_t GPIO_PinOSpeed;			//> Value from @GPIO_PIN_OSPEED
	uint8_t GPIO_PinPuPd;			//> Value from @GPIO_PIN_PUPD
	uint8_t GPIO_PinOType;			//> Value from @GPIO_PIN_OTYPE
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;


// Handle Structure for GPIO pins

typedef struct {
	GPIO_RegDef_t *pGPIOx;				// Pointer to base address of GPIO peripheral
	GPIO_PinConfig_t GPIO_PinConfig;	// GPIO pin configuration settings
} GPIO_Handle_t;


/****************************
 *  API Function Prototypes *
 ****************************/

// Peripheral Clock Control

void GPIO_ClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t state);					// GPIO Clock Enable / Disable


// Peripheral Initialize / De-initialize

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);									// Initialize port
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);									// Return port configuration to reset state


// Data Read / Write

uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t pin);					// Read from input pin
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);								// Read from input port
void GPIO_WriteToPin(GPIO_RegDef_t *pGPIOx, uint8_t pin, uint8_t value);	// Write to output pin
void GPIO_WriteToPort(GPIO_RegDef_t *pGPIOx, uint16_t value);				// Write to output port
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pin);					// Toggle Output pin


// IRQ Configuration and ISR Handling

void GPIO_IRQConfig(uint8_t irqNumber, uint8_t irqPriority, uint8_t state);	// Configure port IRQ
void GPIO_IRQHandle(uint8_t pin);											// Handle port IRQ


#endif /* INC_STM32F407XX_GPIO_H_ */
