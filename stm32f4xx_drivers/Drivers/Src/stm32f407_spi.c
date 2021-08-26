/*
 * stm32f407_spi.c
 *
 *  Created on: Aug 23, 2021
 *      Author: OSdZ
 */

#include "stm32f407xx_spi.h"

/******************************
 *  SPI Function Definitions  *
 ******************************/

/** @brief 		Enable or disable clock for SPI peripheral
 *
 * @param[in] 	*pSPIx	Base address of the SPI peripheral
 * @param[in] 	state	ENABLE or DISABLE
 *
 * @return		none
 */
void SPI_ClkCtrl(SPI_RegDef_t *pSPIx, uint8_t state)
{
	if (state == ENABLE)
	{
		if (pSPIx == SPI1) {
			SPI1_CLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_CLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_CLK_EN();
		}
	}
	else if (state == DISABLE)
	{
		if (pSPIx == SPI1) {
			SPI1_CLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_CLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_CLK_DI();
		}
	}
}

/** @brief 		Initialize SPI peripheral
 *
 * @param[in] 	*pSPIHandle	SPI handle structure with peripheral base address and configuration
 *
 * @return		none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Enable peripheral clock
	SPI_ClkCtrl(pSPIHandle->pSPIx, ENABLE);

	uint32_t prepCR1 = 0;
	uint8_t bits = 0;

	// Configure Device mode
	bits = 0x1 & pSPIHandle->SPI_Config.SPI_DeviceMode;
	prepCR1 |= (bits << SPI_CR1_MSTR);

	// Configure Bus mode
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_FD)
	{
		// BIDI cleared: unidirectional communication
		// prepCR1 &= ~(0x1 << SPI_CR1_BIDIMODE);
		// RXONLY cleared: transmit and receive
		// prepCR1 &= ~(0x1 << SPI_CR1_RXONLY);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_HD)
	{
		// BIDI Set: bidirectional communication
		prepCR1 |= (0x1 << SPI_CR1_BIDIMODE);
		// RXONLY cleared: transmit and receive
		// prepCR1 &= ~(0x1 << SPI_CR1_RXONLY);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_SXRX)
	{
		// BIDI cleared: unidirectional communication
		// prepCR1 &= ~(0x1 << SPI_CR1_BIDIMODE);
		// RXONLY set: Output disabled
		prepCR1 |= (0x1 << SPI_CR1_RXONLY);
	}

	// Configure serial clock speed
	bits = 0x7 & pSPIHandle->SPI_Config.SPI_SclkSpeed;
	prepCR1 |= (bits << SPI_CR1_BR);

	// Configure data frame format
	bits = 0x1 & pSPIHandle->SPI_Config.SPI_DFF;
	prepCR1 |= (bits << SPI_CR1_DFF);

	// Configure Clock Mode
	bits = 0x3 & pSPIHandle->SPI_Config.SPI_ClkMode;
	prepCR1 |= (bits << SPI_CR1_CPHA);	// 2-bit value sets both CPOL and CPHA

	// Configure software slave management
	bits = 0x1 & pSPIHandle->SPI_Config.SPI_SSM;
	prepCR1 |= (bits << SPI_CR1_SSM);

	// Write to CR1
	pSPIHandle->pSPIx->CR1 = 0xFFFF & prepCR1;
}

/** @brief 		Return SPI peripheral to reset state
 *
 * @param[in] 	*pSPIx	SPI periphjeral base address
 *
 * @return		none
 */
void SPI_Deinit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1) {
		SPI1_RST();
	} else if (pSPIx == SPI2) {
		SPI2_RST();
	} else if (pSPIx == SPI3) {
		SPI3_RST();
	}
}

/** @brief 		Enable or disable the SPI peripheral
 *
 * @param[in] 	*pSPIx	Base address of the SPI peripheral
 * @param[in] 	state	ENABLE or DISABLE
 *
 * @return		none
 */
void SPI_Control(SPI_RegDef_t *pSPIx, uint8_t state)
{
	if (state == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE); 								// Set SPE
	}
	else if (state == DISABLE)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_CLEAR);	// Wait for TXE = 1
		while(SPI_GetFlagStatus(pSPIx, SPI_BSY_FLAG) == FLAG_SET);		// Wait for BSY = 0
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);								// Clear SPE
	}
}

/** @brief 		Set / Clear Internal Slave Select
 *
 * @param[in] 	*pSPIx	Base address of the SPI peripheral
 * @param[in] 	state	SET or CLEAR
 *
 * @return		none
 */
void SPI_SSIControl(SPI_RegDef_t *pSPIx, uint8_t state)
{
	if (state == SET)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else if (state == CLEAR)
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/** @brief 		Set / Reset SS Output Enable
 *
 * @param[in] 	*pSPIx	Base address of the SPI peripheral
 * @param[in] 	state	SET or RESET
 *
 * @return		none
 */
void SPI_SSOEControl(SPI_RegDef_t *pSPIx, uint8_t state)
{
	if (state == SET)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else if (state == RESET)
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/** @brief 		Check status of a register
 *
 * @param[in] 	*pSPIx		SPI peripheral base address
 * @param[in] 	flagName	Pointer to data being sent
 *
 * @return		none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagName)
{
	if(pSPIx->SR & flagName)
	{
		return FLAG_SET;
	}
	return FLAG_CLEAR;
}

/** @brief 		Send data
 *
 * @param[in] 	*pSPIx		SPI peripheral base address
 * @param[in] 	*pTXBuffer	Pointer to data being sent
 * @param[in] 	len			Number of bytes to send
 *
 * @return		none
 *
 * @note		This is a blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t len)
{
	while(len > 0)
	{
		if(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_SET)
		{
			if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) // 16-bit data
			{
				pSPIx->DR = *((uint16_t*)pTXBuffer);
				len--;
				len--;
				pTXBuffer += 2;
			}
			else // 8-bit data
			{
				pSPIx->DR = *pTXBuffer;
				len--;
				pTXBuffer++;
			}
		}
	}
}
