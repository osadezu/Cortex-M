/*
 * stm32f407xx.h
 *
 *  Created on: Aug 11, 2021
 *      Author: OSdZ
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>


// Miscellaneous

#define ENABLE			1
#define DISABLE			2
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET


// Flash and SRAM Base Addresses

#define FLASH_BASE		0x08000000U		// Main Memory
#define ROM_BASE		0x1FFF0000U		// 30 KB
#define SRAM1_BASE		0x20000000U		// 112 KB
#define SRAM2_BASE		0x2001C000U		// 16 KB
#define SRAM_BASE		SRAM1_BASE


// APBx and AHBx Bus Peripheral Base Addresses

#define PERIPH_BASE		0x40000000U
#define APB1PERIPH_BASE	PERIPH_BASE
#define APB2PERIPH_BASE	0x40010000U
#define AHB1PERIPH_BASE	0x40020000U
#define AHB2PERIPH_BASE	0x50000000U


// APB1 Bus Peripheral Base Addresses

#define SPI2_BASE		((APB1PERIPH_BASE) + 0x3800U)
#define SPI3_BASE		((APB1PERIPH_BASE) + 0x3C00U)
#define USART2_BASE		((APB1PERIPH_BASE) + 0x4400U)
#define USART3_BASE		((APB1PERIPH_BASE) + 0x4800U)
#define UART4_BASE		((APB1PERIPH_BASE) + 0x4C00U)
#define UART5_BASE		((APB1PERIPH_BASE) + 0x5000U)
#define I2C1_BASE		((APB1PERIPH_BASE) + 0x5400U)
#define I2C2_BASE		((APB1PERIPH_BASE) + 0x5800U)
#define I2C3_BASE		((APB1PERIPH_BASE) + 0x5C00U)


// APB2 Bus Peripheral Base Addresses

#define USART1_BASE		((APB2PERIPH_BASE) + 0x1000U)
#define USART6_BASE		((APB2PERIPH_BASE) + 0x1400U)
#define SPI1_BASE		((APB2PERIPH_BASE) + 0x3000U)
#define EXTI_BASE		((APB2PERIPH_BASE) + 0x3C00U)
#define SYSCFG_BASE		((APB2PERIPH_BASE) + 0x3800U)


// AHB1 Bus Peripheral Base Addresses

#define GPIOA_BASE		((AHB1PERIPH_BASE) + 0x0000U)
#define GPIOB_BASE		((AHB1PERIPH_BASE) + 0x0400U)
#define GPIOC_BASE		((AHB1PERIPH_BASE) + 0x0800U)
#define GPIOD_BASE		((AHB1PERIPH_BASE) + 0x0C00U)
#define GPIOE_BASE		((AHB1PERIPH_BASE) + 0x1000U)
#define GPIOF_BASE		((AHB1PERIPH_BASE) + 0x1400U)
#define GPIOG_BASE		((AHB1PERIPH_BASE) + 0x1800U)
#define GPIOH_BASE		((AHB1PERIPH_BASE) + 0x1C00U)
#define GPIOI_BASE		((AHB1PERIPH_BASE) + 0x2000U)
#define RCC_BASE		((AHB1PERIPH_BASE) + 0x3800U)


// Peripheral Register Definition Structures

typedef struct {
	volatile uint32_t MODER;		// GPIO port mode register
	volatile uint32_t OTYPER;		// GPIO port output type register
	volatile uint32_t OSPEEDR;		// GPIO port output speed register
	volatile uint32_t PUPDR;		// GPIO port pull-up/pull-down register
	volatile uint32_t IDR;			// GPIO port input data register
	volatile uint32_t ODR;			// GPIO port output data register
	volatile uint32_t BSRR;			// GPIO port bit set/reset register
	volatile uint32_t LCKR;			// GPIO port configuration lock register
	volatile uint32_t AFR[2];		// GPIO alternate function register (AFRL: AFR[0] + AFRH: AFR[1])
} GPIO_RegDef_t;

typedef struct {
	volatile uint32_t CR;			// RCC clock control register
	volatile uint32_t PLLCFGR;  	// RCC PLL configuration register
	volatile uint32_t CFGR;			// RCC clock configuration register
	volatile uint32_t CIR;			// RCC clock interrupt register
	volatile uint32_t AHB1RSTR;		// RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;		// RCC AHB2 peripheral reset register
	volatile uint32_t AHB3RSTR;		// RCC AHB3 peripheral reset register
	uint32_t reserved0;
	volatile uint32_t APB1RSTR;		// RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;		// RCC APB2 peripheral reset register
	uint32_t reserved1;
	uint32_t reserved2;
	volatile uint32_t AHB1ENR;		// RCC AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;		// RCC AHB2 peripheral clock enable register
	volatile uint32_t AHB3ENR;		// RCC AHB3 peripheral clock enable register
	uint32_t reserved3;
	volatile uint32_t APB1ENR;		// RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;		// RCC APB2 peripheral clock enable register
	uint32_t reserved4;
	uint32_t reserved5;
	volatile uint32_t AHB1LPENR;	// RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;	// RCC AHB2 peripheral clock enable in low power mode register
	volatile uint32_t AHB3LPENR;	// RCC AHB3 peripheral clock enable in low power mode register
	uint32_t reserved6;
	volatile uint32_t APB1LPENR;	// RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;	// RCC APB2 peripheral clock enable in low power mode register
	uint32_t reserved7;
	uint32_t reserved8;
	volatile uint32_t BDCR;			// RCC Backup domain control register
	volatile uint32_t CSR;			// RCC clock control & status register
	uint32_t reserved9;
	uint32_t reserved10;
	volatile uint32_t SSCGR;		// RCC spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;	// RCC PLLI2S configuration register
} RCC_RegDef_t;


// Peripheral Definitions

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASE)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASE)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASE)
#define GPIOI 			((GPIO_RegDef_t*)GPIOI_BASE)

#define RCC 			((RCC_RegDef_t*)RCC_BASE)


// Peripheral Clock Enable / Disable Macros

#define GPIOA_CLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN()	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_CLK_EN()	(RCC->AHB1ENR |= (1 << 8))

#define SPI2_CLK_EN()	(RCC->APB1ENR |= (1 << 14))
#define SPI3_CLK_EN()	(RCC->APB1ENR |= (1 << 15))
#define USART2_CLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_CLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_CLK_EN()	(RCC->APB1ENR |= (1 << 19))
#define UART5_CLK_EN()	(RCC->APB1ENR |= (1 << 20))
#define I2C1_CLK_EN()	(RCC->APB1ENR |= (1 << 21))
#define I2C2_CLK_EN()	(RCC->APB1ENR |= (1 << 22))
#define I2C3_CLK_EN()	(RCC->APB1ENR |= (1 << 23))

#define USART1_CLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART6_CLK_EN()	(RCC->APB2ENR |= (1 << 5))
#define SPI1_CLK_EN()	(RCC->APB2ENR |= (1 << 12))
#define SYSCFG_CLK_EN()	(RCC->APB2ENR |= (1 << 14))

#define GPIOA_CLK_DI()	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DI()	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DI()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DI()	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DI()	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DI()	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DI()	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DI()	(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_CLK_DI()	(RCC->AHB1ENR &= ~(1 << 8))

#define SPI2_CLK_DI()	(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DI()	(RCC->APB1ENR &= ~(1 << 15))
#define USART2_CLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_CLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_CLK_DI()	(RCC->APB1ENR &= ~(1 << 19))
#define UART5_CLK_DI()	(RCC->APB1ENR &= ~(1 << 20))
#define I2C1_CLK_DI()	(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DI()	(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DI()	(RCC->APB1ENR &= ~(1 << 23))

#define USART1_CLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART6_CLK_DI()	(RCC->APB2ENR &= ~(1 << 5))
#define SPI1_CLK_DI()	(RCC->APB2ENR &= ~(1 << 12))
#define SYSCFG_CLK_DI()	(RCC->APB2ENR &= ~(1 << 14))


// Peripheral Reset Macros

#define GPIOA_RST()		do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_RST()		do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_RST()		do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_RST()		do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_RST()		do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_RST()		do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_RST()		do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_RST()		do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_RST()		do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));} while(0)

#endif /* INC_STM32F407XX_H_ */
