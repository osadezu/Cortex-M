/*
 * stm32f407xx_spi.h
 *
 *  Created on: Aug 23, 2021
 *      Author: OSdZ
 */

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_

#include "stm32f407xx.h"


// SPI Register States

/// @SPI_DEVICE_MODES
/// SPI Device Modes
#define SPI_MODE_SLAVE	0
#define SPI_MODE_MASTER	1

/// @SPI_BUS_CONFIG
/// SPI Bus Modes
#define SPI_BUS_FD		1	// Full Duplex (& Simplex Transmit Only)
#define SPI_BUS_HD		2	// Half Duplex
#define SPI_BUS_SXRX	3	// Simplex Receive Only

/// @SPI_SCKL_SPEED
/// SPI Serial Clock Speeds
#define SPI_SCLK_DIV2	0	// pclk / 2
#define SPI_SCLK_DIV4	1	// pclk / 4
#define SPI_SCLK_DIV8	2	// pclk / 8
#define SPI_SCLK_DIV16	3	// pclk / 16
#define SPI_SCLK_DIV32	4	// pclk / 32
#define SPI_SCLK_DIV64	5	// pclk / 64
#define SPI_SCLK_DIV128	6	// pclk / 128
#define SPI_SCLK_DIV256	7	// pclk / 256

/// @SPI_DFF
/// SPI Data Frame Format
#define SPI_DFF_8BIT	0
#define SPI_DFF_16BIT	1

/// @SPI_CLK_MODES
/// SPI Clock Modes
#define SPI_CLKMODE_0	0	// Clock LOW when idle, 1st clock transition is data capture edge
#define SPI_CLKMODE_1	1	// Clock LOW when idle, 2nd clock transition is data capture edge
#define SPI_CLKMODE_2	2	// Clock HIGH when idle, 1st clock transition is data capture edge
#define SPI_CLKMODE_3	3	// Clock HIGH when idle, 2nd clock transition is data capture edge

/// @SPI_SSM
/// SPI Software Slave Management
#define SPI_SSM_DISABLE	0	// Software slave management disabled
#define SPI_SSM_ENABLE	1	// Software slave management enabled


// SPI SR FLags
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)		// 0: Receive buffer not empty
#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)		// 1: Transmit buffer empty
#define SPI_CHSIDE_FLAG	(1 << SPI_SR_CHSIDE)	// 2: Channel side
#define SPI_UDR_FLAG	(1 << SPI_SR_UDR)		// 3: Underrun flag
#define SPI_CRCERR_FLAG	(1 << SPI_SR_CRCERR)	// 4: CRC error flag
#define SPI_MODF_FLAG	(1 << SPI_SR_MODF)		// 5: Mode fault
#define SPI_OVR_FLAG	(1 << SPI_SR_OVR)		// 6: Overrun flag
#define SPI_BSY_FLAG	(1 << SPI_SR_BSY)		// 7: Busy flag
#define SPI_FRE_FLAG	(1 << SPI_SR_FRE)		// 8: Frame format error


/*************************************
 *  Peripheral Structure Definitions *
 *************************************/

// Configuration Structure for SPIx peripheral

typedef struct {
	uint8_t SPI_DeviceMode;	//> From @SPI_DEVICE_MODES
	uint8_t SPI_BusConfig;	//> From @SPI_BUS_CONFIG
	uint8_t SPI_SclkSpeed;	//> From @SPI_SCKL_SPEED
	uint8_t SPI_DFF;		//> FROM @SPI_DFF
	uint8_t SPI_ClkMode;	//> FROM @SPI_CLK_MODES
	uint8_t SPI_SSM;		//> FROM @SPI_SSM
} SPI_Config_t;

// Handle Structure for SPIx peripheral

typedef struct {
	SPI_RegDef_t *pSPIx;		// Pointer to base address of SPI peripheral
	SPI_Config_t SPI_Config;	// SPI configuration settings
} SPI_Handle_t;


/****************************
 *  API Function Prototypes *
 ****************************/

// Peripheral Clock Control

void SPI_ClkCtrl(SPI_RegDef_t *pSPIx, uint8_t state);							// SPI Clock Enable / Disable


// Peripheral Initialize, De-initialize and Enable

void SPI_Init(SPI_Handle_t *pSPIHandle);										// Initialize peripheral
// TODO: Method to initialize unused SPI_Config_t members to avoid setting registers with garbage values.
void SPI_Deinit(SPI_RegDef_t *pSPIx);											// Return peripheral to reset state
void SPI_Control(SPI_RegDef_t *pSPIx, uint8_t state);							// Peripheral Enable / Disable

// Data Send and Receive

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t len);		// Send Data
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t len);	// Receive Data

// Miscellaneous SPI functions

void SPI_SSIControl(SPI_RegDef_t *pSPIx, uint8_t state);						// Set / Clear Internal Slave Select
void SPI_SSOEControl(SPI_RegDef_t *pSPIx, uint8_t state);						// Set / Reset SS Output Enable
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagName);				// Check status of a register

// IRQ Configuration and ISR Handling

void SPI_IRQConfig(uint8_t irqNumber, uint8_t state);							// Configure IRQ
void SPI_IRQPriorityConfig(uint8_t irqNumber, uint8_t irqPriority);				// Configure IRQ Priority
void SPI_IRQReset(SPI_Handle_t *pSPIHandle);									// Reset IRQ


#endif /* INC_STM32F407XX_SPI_H_ */
