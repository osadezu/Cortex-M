/*
 * stm32f407xx.h
 *
 *  Created on: Aug 11, 2021
 *      Author: OSdZ
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


// Flash and SRAM base addresses

#define FLASH_BASE		0x08000000U		// Main Memory
#define ROM_BASE		0x1FFF0000U		// 30 KB
#define SRAM1_BASE		0x20000000U		// 112 KB
#define SRAM2_BASE		0x2001C000U		// 16 KB
#define SRAM_BASE		SRAM1_BASE


// APBx and AHBx Bus Peripheral base addresses

#define PERIPH_BASE		0x40000000U
#define APB1PERIPH_BASE	PERIPH_BASE
#define APB2PERIPH_BASE	0x40010000U
#define AHB1PERIPH_BASE	0x40020000U
#define AHB2PERIPH_BASE	0x50000000U


// APB1 Bus Peripheral base addresses

#define SPI2_BASE		((APB1PERIPH_BASE) + 0x3800U)
#define SPI3_BASE		((APB1PERIPH_BASE) + 0x3C00U)
#define USART2_BASE		((APB1PERIPH_BASE) + 0x4400U)
#define USART3_BASE		((APB1PERIPH_BASE) + 0x4800U)
#define UART4_BASE		((APB1PERIPH_BASE) + 0x4C00U)
#define UART5_BASE		((APB1PERIPH_BASE) + 0x5000U)
#define I2C1_BASE		((APB1PERIPH_BASE) + 0x5400U)
#define I2C2_BASE		((APB1PERIPH_BASE) + 0x5800U)
#define I2C3_BASE		((APB1PERIPH_BASE) + 0x5C00U)


// APB2 Bus Peripheral base addresses

#define USART1_BASE		((APB2PERIPH_BASE) + 0x1000U)
#define USART6_BASE		((APB2PERIPH_BASE) + 0x1400U)
#define SPI1_BASE		((APB2PERIPH_BASE) + 0x3000U)
#define EXTI_BASE		((APB2PERIPH_BASE) + 0x3C00U)
#define SYSCFG_BASE		((APB2PERIPH_BASE) + 0x3800U)


// AHB1 Bus Peripheral base addresses

#define GPIOA_BASE		((AHB1PERIPH_BASE) + 0x0000U)
#define GPIOB_BASE		((AHB1PERIPH_BASE) + 0x0400U)
#define GPIOC_BASE		((AHB1PERIPH_BASE) + 0x0800U)
#define GPIOD_BASE		((AHB1PERIPH_BASE) + 0x0C00U)
#define GPIOE_BASE		((AHB1PERIPH_BASE) + 0x1000U)
#define GPIOF_BASE		((AHB1PERIPH_BASE) + 0x1400U)
#define GPIOG_BASE		((AHB1PERIPH_BASE) + 0x1800U)
#define GPIOH_BASE		((AHB1PERIPH_BASE) + 0x1C00U)
#define GPIOI_BASE		((AHB1PERIPH_BASE) + 0x2000U)

#endif /* INC_STM32F407XX_H_ */
