/*
 * stm32f407xx.h
 *
 *  Created on: Aug 11, 2021
 *      Author: OSdZ
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>	// Define NULL


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

#define CLEAR			0
#define SET				1
#define RESET			CLEAR
#define DISABLE			CLEAR
#define ENABLE			SET
#define FLAG_CLEAR		CLEAR
#define FLAG_SET		SET

#define PRI_BITS		4				// Priority bits implemented by the MCU
#define PRI_LEVELS		16				// Priority levels implemented by the MCU

#define HSI_CLK_FREQ	16000000U		// 16 MHz internal RC oscillator
#define HSE_CLK_FREQ	8000000U		//  8 MHz crystal oscillator in discovery board


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
#define SYSCFG_BASE		((APB2PERIPH_BASE) + 0x3800U)
#define EXTI_BASE		((APB2PERIPH_BASE) + 0x3C00U)


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
	volatile uint32_t MEMRMP;		// +0x00 SYSCFG memory remap register
	volatile uint32_t PMC;			// +0x04 SYSCFG peripheral mode configuration register
	volatile uint32_t EXTICR[4];	// +0x08 : +0x14 SYSCFG external interrupt configuration registers (EXTICR1 : EXTICR4)
	uint32_t reserved0;				// +0x18
	uint32_t reserved1;				// +0x1C
	volatile uint32_t CMPCR;		// +0x20 Compensation cell control register
} SYSCFG_RegDef_t;

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
	volatile uint32_t IMR;			// +0x00 Interrupt mask register
	volatile uint32_t EMR;			// +0x04 Event mask register
	volatile uint32_t RTSR;			// +0x08 Rising trigger selection register
	volatile uint32_t FTSR;			// +0x0C Falling trigger selection register
	volatile uint32_t SWIER;		// +0x10 Software interrupt event register
	volatile uint32_t PR;			// +0x14 Pending register
} EXTI_RegDef_t;


typedef struct {
	volatile uint32_t CR1;			// +0x00 SPI control register 1
	volatile uint32_t CR2;			// +0x04 SPI control register 2
	volatile uint32_t SR;			// +0x08 SPI status register
	volatile uint32_t DR;			// +0x0C SPI data register
	volatile uint32_t CRCPR;		// +0x10 SPI CRC polynomial register
	volatile uint32_t RXCRCR;		// +0x14 SPI RX CRC register
	volatile uint32_t TXCRCR;		// +0x18 SPI TX CRC register
	volatile uint32_t I2SCFGR;		// +0x1C SPI_I2S configuration register
	volatile uint32_t I2SPR;		// +0x20 SPI_I2S prescaler register
} SPI_RegDef_t;


typedef struct {
	volatile uint32_t CR1;			// +0x00 I2C Control register 1
	volatile uint32_t CR2;			// +0x04 I2C Control register 2
	volatile uint32_t OAR1;			// +0x08 I2C Own address register 1
	volatile uint32_t OAR2;			// +0x0C I2C Own address register 2
	volatile uint32_t DR;			// +0x10 I2C Data Register
	volatile uint32_t SR1;			// +0x14 I2C Status Register 1
	volatile uint32_t SR2;			// +0x18 I2C Status Register 2
	volatile uint32_t CCR;			// +0x1C I2C Clock Control Register
	volatile uint32_t TRISE;		// +0x20 I2C TRISE Register
	volatile uint32_t FLTR;			// +0x24 I2C FLTR Register
} I2C_RegDef_t;


// Peripheral Definitions

#define RCC 			((RCC_RegDef_t*)RCC_BASE)

#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASE)

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASE)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASE)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASE)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASE)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASE)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASE)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASE)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASE)
#define GPIOI 			((GPIO_RegDef_t*)GPIOI_BASE)

#define EXTI			((EXTI_RegDef_t*)EXTI_BASE)

#define SPI1			((SPI_RegDef_t*)SPI1_BASE)
#define SPI2			((SPI_RegDef_t*)SPI2_BASE)
#define SPI3			((SPI_RegDef_t*)SPI3_BASE)

#define I2C1			((I2C_RegDef_t*)I2C1_BASE)
#define I2C2			((I2C_RegDef_t*)I2C2_BASE)
#define I2C3			((I2C_RegDef_t*)I2C3_BASE)


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

#define SPI1_RST()		do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));} while(0)
#define SPI2_RST()		do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));} while(0)
#define SPI3_RST()		do{(RCC->APB2RSTR |= (1 << 15)); (RCC->APB2RSTR &= ~(1 << 15));} while(0)

#define I2C1_RST()		do{(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));} while(0)
#define I2C2_RST()		do{(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));} while(0)
#define I2C3_RST()		do{(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));} while(0)

// IRQ Numbers

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
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


// Peripheral Register Field Definitions

// RCC clock configuration

#define RCC_CFGR_SW			0		// 1:0 System clock switch
#define RCC_CFGR_SWS		2		// 3:2 System clock switch status
#define RCC_CFGR_HPRE		4		// 7:4 AHB Prescaler
#define RCC_CFGR_PPRE1		10		// 12:10 APB Low speed prescaler (APB1)
#define RCC_CFGR_PPRE2		13		// 15:13 APB high-speed prescaler (APB2)

// SPI control register 1

#define SPI_CR1_CPHA		0		// 0: Clock Phase
#define SPI_CR1_CPOL		1		// 1: Clock Polarity
#define SPI_CR1_MSTR		2		// 2: Master Selection
#define SPI_CR1_BR			3		// 5:3 Baud Rate Control
#define SPI_CR1_SPE			6		// 6: SPI Enable
#define SPI_CR1_LSBFIRST	7		// 7: Frame format
#define SPI_CR1_SSI			8		// 8: Internal Slave Select
#define SPI_CR1_SSM			9		// 9: Software Slave Management
#define SPI_CR1_RXONLY		10		// 10: Receive Only
#define SPI_CR1_DFF			11		// 11: Data Frame Format
#define SPI_CR1_CRCNEXT		12		// 12: CRC transfer next
#define SPI_CR1_CRCEN		13		// 13: Hardware CRC calculation enable
#define SPI_CR1_BIDIOE		14		// 14: Output enable in bidirectional mode
#define SPI_CR1_BIDIMODE	15		// 15: Bidirectional data mode enable

// SPI control register 2

#define SPI_CR2_RXDMAEN		0		// 0: Rx buffer DMA enable
#define SPI_CR2_TXDMAEN		1		// 1: Tx buffer DMA enable
#define SPI_CR2_SSOE		2		// 2: SS output enable
#define SPI_CR2_FRF			4		// 4: Frame Format
#define SPI_CR2_ERRIE		5		// 5: Error interrupt enable
#define SPI_CR2_RXNEIE		6		// 6: RX buffer not empty interrupt enable
#define SPI_CR2_TXEIE		7		// 7: Tx buffer empty interrupt enable

// SPI status register

#define SPI_SR_RXNE			0		// 0: Receive buffer not empty
#define SPI_SR_TXE			1		// 1: Transmit buffer empty
#define SPI_SR_CHSIDE		2		// 2: Channel side
#define SPI_SR_UDR			3		// 3: Underrun flag
#define SPI_SR_CRCERR		4		// 4: CRC error flag
#define SPI_SR_MODF			5		// 5: Mode fault
#define SPI_SR_OVR			6		// 6: Overrun flag
#define SPI_SR_BSY			7		// 7: Busy flag
#define SPI_SR_FRE			8		// 8: Frame format error


// I2C control register 1

#define I2C_CR1_PE			0		// 0: Peripheral enable
#define I2C_CR1_SMBUS		1		// 1: SMBus mode
									// 2: Reserved
#define I2C_CR1_SMBTYPE		3		// 3: SMBus type
#define I2C_CR1_ENARP		4		// 4: ARP enable
#define I2C_CR1_ENPEC		5		// 5: PEC enable
#define I2C_CR1_ENGC		6		// 6: General call enable
#define I2C_CR1_NOSTRETCH	7		// 7: Clock stretching disable (slave mode)
#define I2C_CR1_START		8		// 8: Start generation
#define I2C_CR1_STOP		9		// 9: Stop generation
#define I2C_CR1_ACK			10		// 10: Acknowledge enable
#define I2C_CR1_POS			11		// 11: Acknowledge/PEC position (for data reception)
#define I2C_CR1_PEC			12		// 12: Packet error checking
#define I2C_CR1_ALERT		13		// 13: SMBus alert
									// 14: Reserved
#define I2C_CR1_SWRST		15		// 15: Software reset

// I2C control register 2

#define I2C_CR2_FREQ		0		// 5:0 Peripheral clock frequency
									// 7:6 Reserved
#define I2C_CR2_ITERREN		8		// 8: Error interrupt enable
#define I2C_CR2_ITEVTEN		9		// 9: Event interrupt enable
#define I2C_CR2_ITBUFEN		10		// 10: Buffer interrupt enable
#define I2C_CR2_DMAEN		11		// 11: DMA requests enable
#define I2C_CR2_LAST		12		// 12: DMA last transfer

// I2C own address register 1

#define I2C_OAR1_ADD_0		0		// 0: Interface Address (10-bit mode only)
#define I2C_OAR1_ADD		1		// 7:1 Interface Address
#define I2C_OAR1_ADD_98		8		// 9:8 Interface Address (10-bit mode only)
									// 13:10 Reserved
#define I2C_OAR1_BIT14		14		// 14: Should always be kept at 1 by software
#define I2C_OAR1_ADDMODE	15		// 15: Addressing mode

// I2C status register 1

#define I2C_SR1_SB			0		// 0: Start bit (Master mode)
#define I2C_SR1_ADDR		1		// 1: Address sent (master mode)/matched (slave mode)
#define I2C_SR1_BTF			2		// 2: Byte transfer finished
#define I2C_SR1_ADD10		3		// 3: 10-bit header sent (master mode)
#define I2C_SR1_STOPF		4		// 4: Stop detection (slave mode)
									// 5: Reserved
#define I2C_SR1_RXNE		6		// 6: Data register not empty (receivers)
#define I2C_SR1_TXE			7		// 7: Data register empty (transmitters)
#define I2C_SR1_BERR		8		// 8: Bus error
#define I2C_SR1_ARLO		9		// 9: Arbitration lost (master mode)
#define I2C_SR1_AF			10		// 10: Acknowledge failure
#define I2C_SR1_OVR			11		// 11: Overrun/underrun
#define I2C_SR1_PECERR		12		// 12: PEC error in reception
									// 13: Reserved
#define I2C_SR1_TIMEOUT		14		// 14: Timeout or Tlow error
#define I2C_SR1_SMBALERT	15		// 15: SMBus alert

// I2C status register 2

#define I2C_SR2_MSL			0		// 0: Master/slave
#define I2C_SR2_BUSY		1		// 1: Bus busy
#define I2C_SR2_TRA			2		// 2: Transmitter/receiver
									// 3: Reserved
#define I2C_SR2_GENCALL		4		// 4: General call address (slave mode)
#define I2C_SR2_SMBDEFAULT	5		// 5: SMBus device default address
#define I2C_SR2_SMBHOST		6		// 6: SMBus host header (slave mode)
#define I2C_SR2_DUALF		7		// 7: Dual flag (slave mode)
#define I2C_SR2_PEC			8		// 15:8 Packet error checking

// I2C clock control register

#define I2C_CCR_CCR			0		// 11:0 Clock control register in Fm/Sm mode (master mode)
									// 12:13 Reserved
#define I2C_CCR_DUTY		14		// 14: Fm mode duty cycle
#define I2C_CCR_FS			15		// 15: I2C master mode selection

// Number code for GPIO port
// Macro resolves to 0:8 value for PA:PI selection

#define GPIO_PORT_NUMBER(x)		(((uint32_t*)(x) - (uint32_t*)(GPIOA)) >> 8)
// TODO: Is this safe? (Fully hardware implementation dependent).


#endif /* INC_STM32F407XX_H_ */
