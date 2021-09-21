/*
 * stm32f407_spi.c
 *
 *  Created on: Aug 23, 2021
 *      Author: OSdZ
 */

#include "stm32f407xx_spi.h"

// Static IRQ helper function prototypes

static void SPI_TxIRQHandle(SPI_Handle_t *pSPIHandle);							// Handle SPI IRQ Transmit
static void SPI_RxIRQHandle(SPI_Handle_t *pSPIHandle);							// Handle SPI IRQ Receive
static void SPI_OVRIRQHandle(SPI_Handle_t *pSPIHandle);							// Handle SPI IRQ Overrun Error

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
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_CLEAR);	// Wait for TXE = 1
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_BSY) == FLAG_SET);		// Wait for BSY = 0
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
 * @param[in] 	flagName	Bit position to check in register
 *
 * @return		uint8_t		Flag status
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagPosition)
{
	if(pSPIx->SR & flagPosition)
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
		if(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_SET) // Proceed if transmit buffer is empty
		{
			if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) // 16-bit data
			{
				pSPIx->DR = *((uint16_t*)pTXBuffer);
				len -= 2;
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

/** @brief 		Receive data
 *
 * @param[in] 	*pSPIx		SPI peripheral base address
 * @param[in] 	*pRXBuffer	Pointer to store received data
 * @param[in] 	len			Number of bytes to receive
 *
 * @return		none
 *
 * @note		This is a blocking call
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t len)
{
	while(len > 0)
	{
		if(SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == FLAG_SET) // Proceed if receive buffer has data
		{
			if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) // 16-bit data
			{
				*((uint16_t*)pRXBuffer) = pSPIx->DR;
				len -=2;
				pRXBuffer += 2;
			}
			else // 8-bit data
			{
				*pRXBuffer = pSPIx->DR;
				len--;
				pRXBuffer++;
			}
		}
	}
}



/***************************
 *  SPI IRQ Configuration  *
 ***************************/

/** @brief 		Configure SPI IRQ
 *
 * @param[in] 	irqNumber	MCU Specific IRQ position to configure
 * @param[in] 	state		ENABLE or DISABLE
 *
 * @return		none
 */
void SPI_IRQConfig(uint8_t irqNumber, uint8_t state)
{
	if (state == ENABLE)
	{
		if (irqNumber <= 31)		// NVIC ISER0
		{
			*NVIC_ISER0 |= (0x1 << irqNumber);
		}
		else if (irqNumber <= 63)	// NVIC ISER1
		{
			*NVIC_ISER1 |= (0x1 << (irqNumber % 32));
		}
		else if (irqNumber <= 95) 	// NVIC ISER2
		{
			*NVIC_ISER2 |= (0x1 << (irqNumber % 32));
		}
	}
	else if (state == DISABLE)
	{
		if (irqNumber <= 31)		// NVIC ISER0
		{
			*NVIC_ICER0 |= (0x1 << irqNumber);
		}
		else if (irqNumber <= 63)	// NVIC ISER1
		{
			*NVIC_ICER1 |= (0x1 << (irqNumber % 32));
		}
		else if (irqNumber <= 95) 	// NVIC ISER2
		{
			*NVIC_ICER2 |= (0x1 << (irqNumber % 32));
		}
	}
}

/** @brief 		Configure SPI IRQ PRiority
 *
 * @param[in] 	irqNumber		MCU Specific IRQ position to configure
 * @param[in] 	irqPriority		Priority to assign
 *
 * @return		none
 */
void SPI_IRQPriorityConfig(uint8_t irqNumber, uint8_t irqPriority)
{
	uint8_t bits = (PRI_LEVELS - 1) & irqPriority; // Mask bits up to implemented priority levels

	NVIC_IPR->PRI[irqNumber] &= ~ (0xF << (8 - PRI_BITS)); // Clear
	NVIC_IPR->PRI[irqNumber] |= (bits << (8 - PRI_BITS)); // Set
}

/** @brief 		Handle SPI IRQ
 *
 * @param[in] 	*pSPIHandle	SPI handle structure with peripheral base address and configuration
 *
 * @return		none
 *
 */
void SPI_IRQHandle(SPI_Handle_t *pSPIHandle)
{
	uint8_t eventFlag;
	uint8_t irqEnable;

	// Check for TXE interrupt
	eventFlag = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	irqEnable = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(eventFlag && irqEnable)
	{
		SPI_TxIRQHandle(pSPIHandle);
	}

	// Check for RXNE interrupt
	eventFlag = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	irqEnable = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(eventFlag && irqEnable)
	{
		SPI_RxIRQHandle(pSPIHandle);
	}

	// Check for overrun error interrupt
	eventFlag = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	irqEnable = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(eventFlag && irqEnable)
	{
		SPI_OVRIRQHandle(pSPIHandle);
	}
}

/** @brief 		IRQ based Send Data
 *
 * @param[in] 	*pSPIHandle	SPI handle structure with peripheral base address and configuration
 * @param[in] 	*pTXBuffer	Pointer to data being sent
 * @param[in] 	len			Number of bytes to send
 *
 * @return		uint8_t		Transmit state
 *
 */
uint8_t SPI_SendDataIRQ(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_TX)
	{
		// Save TxBuffer address and len information
		pSPIHandle->pTxBuffer = pTXBuffer;
		pSPIHandle->TxLen = len;

		// Mark SPI state busy to block from other API calls
		pSPIHandle->TxState = SPI_BUSY_TX;

		// Enable interrupt when TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// Data transmission is handled by ISR code
	}

	return state;
}

/** @brief 		IRQ based Receive Data
 *
 * @param[in] 	*pSPIHandle	SPI handle structure with peripheral base address and configuration
 * @param[in] 	*pTXBuffer	Pointer to store received data
 * @param[in] 	len			Number of bytes to receive
 *
 * @return		uint8_t		Receive state
 *
 */
uint8_t SPI_ReceiveDataIRQ(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_RX)
	{
		// Save RxBuffer address and len information
		pSPIHandle->pRxBuffer = pRXBuffer;
		pSPIHandle->RxLen = len;

		// Mark SPI state busy to block from other API calls
		pSPIHandle->RxState = SPI_BUSY_RX;

		// Enable interrupt when RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// Data reception is handled by ISR code
	}

	return state;
}

static void SPI_TxIRQHandle(SPI_Handle_t *pSPIHandle)
{
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) // 16-bit data
	{
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -= 2;
		pSPIHandle->pTxBuffer += 2;
	}
	else // 8-bit data
	{
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(pSPIHandle->TxLen == 0) // All frames sent, close transmission
	{
		SPI_CloseTransmission(pSPIHandle);
	}
}

static void SPI_RxIRQHandle(SPI_Handle_t *pSPIHandle)
{
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) // 16-bit data
	{
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer += 2;
	}
	else // 8-bit data
	{
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(pSPIHandle->RxLen == 0) // All frames sent, close transmission
	{
		SPI_CloseReception(pSPIHandle);
	}
}

static void SPI_OVRIRQHandle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->TxState != SPI_BUSY_TX) // Do not touch DR if application is receiving data
	{
		SPI_ClearOVRFlag(pSPIHandle);
	}	// If SPI was busy in Tx, then application must clear OVR flag manually

	// Notify application about overrun error
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR); // Must be implemented by the application
}

/** @brief 		Clear overrun error flag
 *
 * @param[in] 	*pSPIHandle	SPI handle structure with peripheral base address and configuration
 *
 * @return		none
 *
 */
void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle)
{
	uint8_t touch;
	touch = pSPIHandle->pSPIx->DR;	// Touch registers but no need to do anything with the values
	touch = pSPIHandle->pSPIx->SR;	// These two reads clear OVR flag
	(void) touch;
}

/** @brief 		End SPI Tx
 *
 * @param[in] 	*pSPIHandle	SPI handle structure with peripheral base address and configuration
 *
 * @return		none
 *
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	// Disable interrupt until new SPI_SendDataIRQ() call is made by application
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

	// Clear transmission arguments
	pSPIHandle->TxLen = 0;
	pSPIHandle->pTxBuffer = NULL;

	// SPI state ready to allow new calls
	pSPIHandle->TxState = SPI_READY;

	// Notify application that transmission is done
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_DONE); // Must be implemented by the application
}

/** @brief 		End SPI Rx
 *
 * @param[in] 	*pSPIHandle	SPI handle structure with peripheral base address and configuration
 *
 * @return		none
 *
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	// Disable interrupt until new SPI_ReceiveDataIRQ() call is made by application
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

	// Clear transmission arguments
	pSPIHandle->RxLen = 0;
	pSPIHandle->pRxBuffer = NULL;

	// SPI state ready to allow new calls
	pSPIHandle->RxState = SPI_READY;

	// Notify application that data reception is done
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_DONE); // Must be implemented by the application
}

/** @brief 		SPI IRQ Event Callback
 *
 * @param[in] 	*pSPIHandle	SPI handle structure with peripheral base address and configuration
 * @param[in] 	event		Event callback code
 *
 * @return		none
 *
 */
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event)
{
	// Weak palceholder. Must be implemented by the application
}
