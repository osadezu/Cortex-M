/*
 * stm32f407xx.h
 *
 *  Created on: Aug 11, 2021
 *      Author: OSdZ
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>


/*****************************************
 *  ARM Cortex-Mx Processor Core Macros  *
 *****************************************/

// NVIC Base Address

#define NVIC_BASE		0xE000E100U


// NVIC Interrupt Set-enable Registers

#define NVIC_ISER0		((volatile uint32_t*) 0xE000E100U)
#define NVIC_ISER1		((volatile uint32_t*) 0xE000E104U)
#define NVIC_ISER2		((volatile uint32_t*) 0xE000E108U)


// NVIC Interrupt Clear-enable Registers

#define NVIC_ICER0		((volatile uint32_t*) 0XE000E180U)
#define NVIC_ICER1		((volatile uint32_t*) 0xE000E184U)
#define NVIC_ICER2		((volatile uint32_t*) 0xE000E188U)
// TODO: Consolidate ISER, ICER, etc in an NVIC_RegDef_t for consistency.


// Private Peripheral Register Definition Structures

typedef struct {
	volatile uint8_t PRI[240];	// Byte offset, STM32F407x uses only 82
} NVIC_IPR_RegDef_t;

// NVIC Interrupt Priority Registers

#define NVIC_IPR_BASE	0xE000E400U
#define NVIC_IPR		((NVIC_IPR_RegDef_t*)NVIC_IPR_BASE)


/***************************
 *  STM32F407x MCU Macros  *
 ***************************/

// Miscellaneous

#define ENABLE			1
#define DISABLE			2
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#define PRI_BITS		4				// Priority bits implemented by the MCU
#define PRI_LEVELS		16				// Priority levels implemented by the MCU


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
	volatile uint32_t MODER;		// +0x00 GPIO port mode register
	volatile uint32_t OTYPER;		// +0x04 GPIO port output type register
	volatile uint32_t OSPEEDR;		// +0x08 GPIO port output speed register
	volatile uint32_t PUPDR;		// +0x0C GPIO port pull-up/pull-down register
	volatile uint32_t IDR;			// +0x10 GPIO port input data register
	volatile uint32_t ODR;			// +0x14 GPIO port output data register
	volatile uint32_t BSRR;			// +0x18 GPIO port bit set/reset register
	volatile uint32_t LCKR;			// +0x1C GPIO port configuration lock register
	volatile uint32_t AFR[2];		// +0x20 : +0x24 GPIO alternate function registers (AFRL: AFR[0] + AFRH: AFR[1])
} GPIO_RegDef_t;

typedef struct {
	volatile uint32_t CR;			// +0x00 RCC clock control register
	volatile uint32_t PLLCFGR;  	// +0x04 RCC PLL configuration register
	volatile uint32_t CFGR;			// +0x08 RCC clock configuration register
	volatile uint32_t CIR;			// +0x0C RCC clock interrupt register
	volatile uint32_t AHB1RSTR;		// +0x10 RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;		// +0x14 RCC AHB2 peripheral reset register
	volatile uint32_t AHB3RSTR;		// +0x18 RCC AHB3 peripheral reset register
	uint32_t reserved0;				// +0x1C
	volatile uint32_t APB1RSTR;		// +0x20 RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;		// +0x24 RCC APB2 peripheral reset register
	uint32_t reserved1;				// +0x28
	uint32_t reserved2;				// +0x2C
	volatile uint32_t AHB1ENR;		// +0x30 RCC AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;		// +0x34 RCC AHB2 peripheral clock enable register
	volatile uint32_t AHB3ENR;		// +0x38 RCC AHB3 peripheral clock enable register
	uint32_t reserved3;				// +0x3C
	volatile uint32_t APB1ENR;		// +0x40 RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;		// +0x44 RCC APB2 peripheral clock enable register
	uint32_t reserved4;				// +0x48
	uint32_t reserved5;				// +0x4C
	volatile uint32_t AHB1LPENR;	// +0x50 RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;	// +0x54 RCC AHB2 peripheral clock enable in low power mode register
	volatile uint32_t AHB3LPENR;	// +0x58 RCC AHB3 peripheral clock enable in low power mode register
	uint32_t reserved6;				// +0x5C
	volatile uint32_t APB1LPENR;	// +0x60 RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;	// +0x64 RCC APB2 peripheral clock enable in low power mode register
	uint32_t reserved7;				// +0x68
	uint32_t reserved8;				// +0x6C
	volatile uint32_t BDCR;			// +0x70 RCC Backup domain control register
	volatile uint32_t CSR;			// +0x74 RCC clock control & status register
	uint32_t reserved9;				// +0x78
	uint32_t reserved10;			// +0x7C
	volatile uint32_t SSCGR;		// +0x80 RCC spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;	// +0x84 RCC PLLI2S configuration register
} RCC_RegDef_t;

typedef struct {
	volatile uint32_t IMR;			// +0x00 Interrupt mask register
	volatile uint32_t EMR;			// +0x04 Event mask register
	volatile uint32_t RTSR;			// +0x08 Rising trigger selection register
	volatile uint32_t FTSR;			// +0x0C Falling trigger selection register
	volatile uint32_t SWIER;		// +0x10 Software interrupt event register
	volatile uint32_t PR;			// +0x14 Pending register
} EXTI_RegDef_t;

typedef struct {
	volatile uint32_t MEMRMP;		// +0x00 SYSCFG memory remap register
	volatile uint32_t PMC;			// +0x04 SYSCFG peripheral mode configuration register
	volatile uint32_t EXTICR[4];	// +0x08 : +0x14 SYSCFG external interrupt configuration registers (EXTICR1 : EXTICR4)
	uint32_t reserved0;				// +0x18
	uint32_t reserved1;				// +0x1C
	volatile uint32_t CMPCR;		// +0x20 Compensation cell control register
} SYSCFG_RegDef_t;

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

#define EXTI			((EXTI_RegDef_t*)EXTI_BASE)

#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASE)


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


// IRQ Numbers

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40


// IRQ Priority Levels

#define IRQ_PRI_0			0	// Highest Priority
#define IRQ_PRI_1			1
#define IRQ_PRI_2			2
#define IRQ_PRI_3			3
#define IRQ_PRI_4			4
#define IRQ_PRI_5			5
#define IRQ_PRI_6			6
#define IRQ_PRI_7			7
#define IRQ_PRI_8			8
#define IRQ_PRI_9			9
#define IRQ_PRI_10			10
#define IRQ_PRI_11			11
#define IRQ_PRI_12			12
#define IRQ_PRI_13			13
#define IRQ_PRI_14			14
#define IRQ_PRI_15			15	// Lowest Priority


// Return number code for GPIO port
// Macro resolves to 0:8 value for PA:PI selection

#define GPIO_PORT_NUMBER(x)		(((uint32_t*)(x) - (uint32_t*)(GPIOA)) >> 8)


#endif /* INC_STM32F407XX_H_ */
