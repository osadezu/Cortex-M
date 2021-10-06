/*
 * stm32f407xx_i2c.c
 *
 *  Created on: Sep 10, 2021
 *      Author: OSdZ
 */

#include "stm32f407xx_i2c.h"
#include "stm32f407xx_rcc.h"

// Static helper function prototypes

static void I2C_IssueStart(I2C_RegDef_t *pI2Cx);
static void I2C_IssueAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddress, uint8_t readWrite);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleIRQ_TXE(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleIRQ_RXNE(I2C_Handle_t *pI2CHandle);

/******************************
 *  I2C Function Definitions  *
 ******************************/

/** @brief 		Enable or disable ACK bit
 *
 * @param[in] 	*pI2Cx	Base address of the I2C peripheral
 * @param[in] 	state	ENABLE or DISABLE
 *
 * @return		none
 */
void I2C_AckControl(I2C_RegDef_t *pI2Cx, uint8_t status)
{
	if (status == DISABLE)
	{
		pI2Cx->CR1 &= ~(0x1 << I2C_CR1_ACK);	// ACK bit cleared
	}
	else	// Enable
	{
		pI2Cx->CR1 |= (0x1 << I2C_CR1_ACK);		// ACK bit set
	}
}

/** @brief 		Enable or disable clock for I2C peripheral
 *
 * @param[in] 	*pI2Cx	Base address of the I2C peripheral
 * @param[in] 	state	ENABLE or DISABLE
 *
 * @return		none
 */
void I2C_ClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t state)
{
	if (state == ENABLE)
	{
		if (pI2Cx == I2C1) {
			I2C1_CLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_CLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_CLK_EN();
		}
	}
	else if (state == DISABLE)
	{
		if (pI2Cx == I2C1) {
			I2C1_CLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_CLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_CLK_DI();
		}
	}
}

/** @brief 		Initialize I2C peripheral
 *
 * @param[in] 	*pI2CHandle	I2C handle structure with peripheral base address and configuration
 *
 * @return		none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	// Enable peripheral clock
	I2C_ClkCtrl(pI2CHandle->pI2Cx, ENABLE);

	uint32_t prepReg = 0;

	// Acknowledge enable
	prepReg |= (0x1 & pI2CHandle->I2C_Config.I2C_AckControl) << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = prepReg;

	prepReg = 0;

	// Configure FREQ
	prepReg |= (0x3F & (RCC_GetPclkFreq(PCLK1) / 1000000U)) << I2C_CR2_FREQ; // Must be value between 2-50 inclusive.
	pI2CHandle->pI2Cx->CR2 = prepReg;

	prepReg = 0;

	// Configure device own address
	prepReg |= (0x7F & pI2CHandle->I2C_Config.I2C_DeviceAddress) << I2C_OAR1_ADD;
	prepReg |= 0x1 << I2C_OAR1_BIT14;										// Should always be kept at 1 by software
	pI2CHandle->pI2Cx->OAR1 = prepReg;

	prepReg = 0;

	// Configure CCR
	uint16_t ccr_value;

	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SM)	// Standard mode
	{
		// I2C_CCR_FS remains 0
		ccr_value = RCC_GetPclkFreq(PCLK1) / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);

	}
	else													// Fast mode
	{
		prepReg |= 0x1 << I2C_CCR_FS;
		prepReg |= (0x1 & pI2CHandle->I2C_Config.I2C_FMDutyCycle) << I2C_CCR_DUTY;

		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)	// Tl/Th = 2
		{
			ccr_value = RCC_GetPclkFreq(PCLK1) / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			ccr_value = RCC_GetPclkFreq(PCLK1) / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}

	}

	prepReg |= (0xFFF & ccr_value) << I2C_CCR_CCR;
	pI2CHandle->pI2Cx->CCR = prepReg;

	prepReg = 0;

	// Configure TRISE
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SM)	// Standard mode
	{
		prepReg |= 0x3F & ((RCC_GetPclkFreq(PCLK1) / 1000000U) + 1);
		// I2C spec max rise time is 1000ns (or 1us) -> 1/1E6
		// Then add 1 per reference manual

	}
	else													// Fast mode
	{
		prepReg |= 0x3F & ((RCC_GetPclkFreq(PCLK1) * 300 / 1000000000U) + 1);
		// I2C spec max rise time is 300ns  -> 300/1E9
		// Then add 1 per reference manual
	}

	pI2CHandle->pI2Cx->TRISE = prepReg;

}

/** @brief 		Return I2C peripheral to reset state
 *
 * @param[in] 	*pI2Cx	I2C peripheral base address
 *
 * @return		none
 */
void I2C_Deinit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1) {
		I2C1_RST();
	} else if (pI2Cx == I2C2) {
		I2C2_RST();
	} else if (pI2Cx == I2C3) {
		I2C3_RST();
	}
}

/** @brief 		Enable or disable the I2C peripheral
 *
 * @param[in] 	*pI2Cx	Base address of the I2C peripheral
 * @param[in] 	state	ENABLE or DISABLE
 *
 * @return		none
 */
void I2C_Control(I2C_RegDef_t *pI2Cx, uint8_t state)
{
	if (state == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE); 								// Set PE
	}
	else if (state == DISABLE)
	{
		// TODO: When in Master mode, this method should fail, or wait until I2C is in slave mode, before clearing PE.
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);								// Clear SPE
	}
}

/** @brief 		Check status of a register
 *
 * @param[in] 	*pI2Cx			I2C peripheral base address
 * @param[in] 	flagPosition	Bit position to check in register
 *
 * @return		none
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint16_t flagPosition)
{
	if(pI2Cx->SR1 & flagPosition)
	{
		return FLAG_SET;
	}
	return FLAG_CLEAR;
}


/** @brief 		Generate stop condition
 *
 * @param[in] 	*pI2Cx			I2C peripheral base address
 *
 * @return		none
 */
void I2C_IssueStop(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= 0x1 << I2C_CR1_STOP;
}

/** @brief 		Master send data
 *
 * @param[in] 	*pI2CHandle		I2C handle structure with peripheral base address and configuration
 * @param[in] 	slaveAddress	Slave address
 * @param[in] 	*pTXBuffer		Pointer to data being sent
 * @param[in] 	len				Number of bytes to send
 * @param[in] 	withStop		I2C_WITH_STOP or I2C_NO_STOP to indicate if application will do a repeated start
 *
 * @return		none
 *
 * @note		This is a blocking call
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t slaveAddress, uint8_t *pTXBuffer, uint32_t len, uint8_t withStop)
{
	// Generate start condition
	I2C_IssueStart(pI2CHandle->pI2Cx);

	// Confirm start generation is complete (wait for SB flag)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// Generate address phase (Writing to DR after reading SR1 clears SB flag)
	I2C_IssueAddressPhase(pI2CHandle->pI2Cx, slaveAddress, I2C_WRITE);

	// Check that address phase is completed (wait for ADDR flag)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// Clear ADDR flag to stop clock stretching
	I2C_ClearADDRFlag(pI2CHandle);

	// Send data
	while(len > 0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));	// Wait for TXE flag

		pI2CHandle->pI2Cx->DR = *pTXBuffer;
		pTXBuffer++;
		len--;
	}

	// Wait for data send to complete
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));	// Wait for TXE flag
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));	// Wait for BTF flag

	// Generate stop condition, unless application will do repeat start
	if (withStop == I2C_WITH_STOP)
	{
		I2C_IssueStop(pI2CHandle->pI2Cx);
	}
}

/** @brief 		Master receive data
 *
 * @param[in] 	*pI2CHandle		I2C handle structure with peripheral base address and configuration
 * @param[in] 	slaveAddress	Slave address
 * @param[in] 	*pRXBuffer		Pointer in which to store to received data
 * @param[in] 	len				Number of bytes to receive
 * @param[in] 	withStop		I2C_WITH_STOP or I2C_NO_STOP to indicate if application will do a repeated start
 *
 * @return		none
 *
 * @note		This is a blocking call
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t slaveAddress, uint8_t *pRXBuffer, uint32_t len, uint8_t withStop)
{
	if (len == 0)
		return;

	// Generate start condition
	I2C_IssueStart(pI2CHandle->pI2Cx);

	// Confirm start generation is complete (wait for SB flag)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// Generate address phase (Writing to DR after reading SR1 clears SB flag)
	I2C_IssueAddressPhase(pI2CHandle->pI2Cx, slaveAddress, I2C_READ);

	// Check that address phase is completed (wait for ADDR flag)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	if (len == 1) // Receiving only one byte
	{
		// Disable Ack bit
		I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);

		// Clear ADDR flag to stop clock stretching
		I2C_ClearADDRFlag(pI2CHandle);

		// Wait for RXNE
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// Generate stop condition, unless application will do repeat start
		if (withStop == I2C_WITH_STOP)
			I2C_IssueStop(pI2CHandle->pI2Cx);

		*pRXBuffer = pI2CHandle->pI2Cx->DR;
	}
	else	// Receiving more than 1 byte
	{
		// Clear ADDR flag to stop clock stretching
		I2C_ClearADDRFlag(pI2CHandle);

		while(len > 0)
		{
			// Wait for RXNE
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if (len == 2) // Final two bytes
			{
				// Disable Ack bit
				I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);

				// TODO: Should this happen only after BTF is set?
				// Generate stop condition, unless application will do repeat start
				if (withStop == I2C_WITH_STOP)
					I2C_IssueStop(pI2CHandle->pI2Cx);
			}

			// Read byte
			*pRXBuffer = pI2CHandle->pI2Cx->DR;
			pRXBuffer++;
			len--;
		}
	}

	// Enable Ack bit if configured
	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_EN)
	{
		I2C_AckControl(pI2CHandle->pI2Cx, ENABLE);
	}

}

/** @brief 		Slave send data
 *
 * @param[in] 	*pI2Cx	I2C peripheral base address
 * @param[in] 	data	Byte to send
 *
 * @return		none
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}

/** @brief 		Slave receive data
 *
 * @param[in] 	*pI2Cx	I2C peripheral base address
 *
 * @return		uint8_t	Byte received
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}

static void I2C_IssueStart(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= 0x1 << I2C_CR1_START;
}

static void I2C_IssueAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddress, uint8_t readWrite)
{
	uint8_t frame = 0;

	frame = slaveAddress << 1;

	if (readWrite == I2C_WRITE)
	{
		frame &= ~0x1;		// Read/nWrite bit cleared
	}
	else
	{
		frame |= 0x1;		// Read/nWrite bit set
	}

	pI2Cx->DR = frame;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR2_MSL))	// Master mode
	{
		if (pI2CHandle->TxRxState == I2C_BUSY_RX)		// Rx operation
		{
			if (pI2CHandle->RxSize == 1)				// Receiving only one byte
			{
				// Disable Ack bit
				I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);
			}
		}
	}

	pI2CHandle->pI2Cx->SR1;	// Touch registers but no need to do anything with the values
	pI2CHandle->pI2Cx->SR2;	// These two reads clear ADDR flag

// TODO: Check disassembly to verify the above causes correct behavior, otherwise replace with:
//	uint32_t touch;
//	touch = pI2CHandle->pI2Cx->SR1;	// Touch registers but no need to do anything with the values
//	touch = pI2CHandle->pI2Cx->SR2;	// These two reads clear ADDR flag
//	(void) touch;

}

/** @brief 		IRQ based Master send data
 *
 * @param[in] 	*pI2CHandle		I2C handle structure with peripheral base address and configuration
 * @param[in] 	slaveAddress	Slave address
 * @param[in] 	*pTXBuffer		Pointer to data being sent
 * @param[in] 	len				Number of bytes to send
 * @param[in] 	withStop		I2C_WITH_STOP or I2C_NO_STOP to indicate if application will do a repeated start
 *
 * @return		uint8_t			Transmit state
 *
 * @note		This is a blocking call
 */
uint8_t I2C_MasterSendDataIRQ(I2C_Handle_t *pI2CHandle, uint8_t slaveAddress, uint8_t *pTXBuffer, uint32_t len, uint8_t withStop)
{
	uint8_t state = pI2CHandle->TxRxState;

	if((state != I2C_BUSY_TX) && (state != I2C_BUSY_RX)) // I2C is not busy with a transaction
	{
		pI2CHandle->pTxBuffer = pTXBuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_TX;
		pI2CHandle->DeviceAddr = slaveAddress;
		pI2CHandle->WithStop = withStop;

		// Generate start condition
		I2C_IssueStart(pI2CHandle->pI2Cx);

		// Enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// Enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		// Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return state;
}

/** @brief 		IRQ based Master receive data
 *
 * @param[in] 	*pI2CHandle		I2C handle structure with peripheral base address and configuration
 * @param[in] 	slaveAddress	Slave address
 * @param[in] 	*pRXBuffer		Pointer in which to store to received data
 * @param[in] 	len				Number of bytes to receive
 * @param[in] 	withStop		I2C_WITH_STOP or I2C_NO_STOP to indicate if application will do a repeated start
 *
 * @return		uint8_t			Receive state
 *
 * @note		This is a blocking call
 */
uint8_t I2C_MasterReceiveDataIRQ(I2C_Handle_t *pI2CHandle, uint8_t slaveAddress, uint8_t *pRXBuffer, uint32_t len, uint8_t withStop)
{
	uint8_t state = pI2CHandle->TxRxState;

	if((state != I2C_BUSY_TX) && (state != I2C_BUSY_RX)) // I2C is not busy with a transaction
	{
		pI2CHandle->pRxBuffer = pRXBuffer;
		pI2CHandle->RxLen = len;
		pI2CHandle->RxSize = len;
		pI2CHandle->TxRxState = I2C_BUSY_RX;
		pI2CHandle->DeviceAddr = slaveAddress;
		pI2CHandle->WithStop = withStop;

		// Generate start condition
		I2C_IssueStart(pI2CHandle->pI2Cx);

		// Enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// Enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		// Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return state;
}

/** @brief 		End I2C Tx
 *
 * @param[in] 	*pI2CHandle	I2C handle structure with peripheral base address and configuration
 *
 * @return		none
 *
 */
void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle)
{
	// Enable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	// Enable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
	pI2CHandle->TxRxState = I2C_READY;

}

/** @brief 		End I2C Rx
 *
 * @param[in] 	*pI2CHandle	I2C handle structure with peripheral base address and configuration
 *
 * @return		none
 *
 */
void I2C_CloseReception(I2C_Handle_t *pI2CHandle)
{
	// Enable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	// Enable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	pI2CHandle->TxRxState = I2C_READY;

	// Enable Ack bit if configured
	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_EN)
	{
		I2C_AckControl(pI2CHandle->pI2Cx, ENABLE);
	}

}

/** @brief 		Enable or disable callback events for Slave mode
 *
 * @param[in] 	*pI2Cx	Base address of the I2C peripheral
 * @param[in] 	state	ENABLE or DISABLE
 *
 * @return		none
 *
 */
void I2C_CallbackEventsControl(I2C_RegDef_t *pI2Cx, uint8_t status)
{
	if (status == ENABLE)
	{
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);	// Enable Buffer interrupts
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);	// Enable Event interrupts
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);	// Enable Error interrupts
	}
	else
	{
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);	// Disable Buffer interrupts
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);	// Disable Event interrupts
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);	// Disable Error interrupts
	}
}

static void I2C_MasterHandleIRQ_TXE(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->TxLen > 0)	// More data must go out
	{
		// Write to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->pTxBuffer++;
		pI2CHandle->TxLen--;
	}
}

static void I2C_MasterHandleIRQ_RXNE(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->RxSize == 1)	// Receiving single byte
	{
		// ACK is disabled by I2C_ClearADDRFlag() for 1 byte RX
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize > 1)	// Receiving more than one byte
	{
		if (pI2CHandle->RxLen == 2) // Final two bytes
		{
			// Disable ACK bit
			I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);

		}

		// Read byte
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxLen == 0)	// All bytes have been received
	{
		// TODO: should this stop happen before reading last byte? <<<Compare blocking reception>>>
		// Generate stop condition, unless application will do repeat start
		if (pI2CHandle->WithStop == I2C_WITH_STOP)
			I2C_IssueStop(pI2CHandle->pI2Cx);

		// Close reception and reset IRQ elements in handle struct
		I2C_CloseReception(pI2CHandle);

		// Notify Application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_RX_DONE);
	}
}

/** @brief 		Handle I2C Event IRQ for both master and slave modes
 *
 * @param[in] 	*pI2CHandle	I2C handle structure with peripheral base address and configuration
 *
 * @return		none
 *
 */
void I2C_HandleEventIRQ(I2C_Handle_t *pI2CHandle)
{
//	Not necessary to check for ITEVTEN again if interrupt was generated.
//	if (pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN))	// Event interrupt call
//	{
		if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB))		// Start condition was generated
		{
			// Master mode only

			if (pI2CHandle->TxRxState == I2C_BUSY_TX)
			{
				I2C_IssueAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DeviceAddr, I2C_WRITE);
			}
			else if (pI2CHandle->TxRxState == I2C_BUSY_RX)
			{
				I2C_IssueAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DeviceAddr, I2C_READ);
			}
		}

		if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR))	// Address phase done
		{
			// Master: Address sent and ACK'd
			// Slave: Address matched

			I2C_ClearADDRFlag(pI2CHandle);
		}

		if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF))	// Byte transfer finished
		{
			if (pI2CHandle->TxRxState == I2C_BUSY_TX)		// TXE must be 1
			{
				// Generate stop condition, unless application will do repeat start
				if (pI2CHandle->WithStop == I2C_WITH_STOP)
					I2C_IssueStop(pI2CHandle->pI2Cx);

				// Close transmission and reset IRQ elements in handle struct
				I2C_CloseTransmission(pI2CHandle);

				// Notify Application
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_TX_DONE);
			}
		}

		if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF))	// Stop detected
		{
			// Slave mode only
			// STOPF is cleared by writing CR1 after reading SR1
			pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
			// pI2CHandle->pI2Cx->CR1 |= 0x0000;

			// Notify Application
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_STOP);

		}

		if (pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN))			// Buffer interrupt call
		{
			if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))			// TXE interrupt
			{
				if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))		// In Master mode
				{
					if (pI2CHandle->TxRxState == I2C_BUSY_TX)			// Tx operation is underway
					{
						I2C_MasterHandleIRQ_TXE(pI2CHandle);
					}
				}
				else													// In Slave mode
				{
					if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))	// Confirm Transmit mode
					{
						I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_DATA_REQ);
					}
				}
			}

			if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE))			// RXNE interrupt
			{
				if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))		// In Master mode
				{
					if (pI2CHandle->TxRxState == I2C_BUSY_RX)			// Rx operation is underway
					{
						I2C_MasterHandleIRQ_RXNE(pI2CHandle);
					}
				}
				else													// In Slave mode
				{
					if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))	// Confirm Receive mode
					{
						I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_DATA_RCV);
					}
				}
			}
		}
//	}
}

/** @brief 		Handle I2C Error IRQ for both master and slave modes
 *
 * @param[in] 	*pI2CHandle	I2C handle structure with peripheral base address and configuration
 *
 * @return		none
 *
 */
void I2C_HandleErrorIRQ(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR))	// Bus error
	{
		// Clear flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		// Notify Application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);

	}

	if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO))	// Arbitration lost
	{
		// Clear flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		// Notify Application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);

	}

	if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF))	// ACK Failure
	{
		// Clear flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		// Notify Application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);

	}

	if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR))	// Overrun/underrun
	{
		// Clear flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		// Notify Application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);

	}

	if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT))	// Timeout
	{
		// Clear flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		// Notify Application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);

	}
}

/***************************
 *  I2C IRQ Configuration  *
 ***************************/

/** @brief 		Configure I2C IRQ
 *
 * @param[in] 	irqNumber	MCU Specific IRQ position to configure
 * @param[in] 	state		ENABLE or DISABLE
 *
 * @return		none
 */
void I2C_IRQConfig(uint8_t irqNumber, uint8_t state)
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

/** @brief 		Configure I2C IRQ PRiority
 *
 * @param[in] 	irqNumber		MCU Specific IRQ position to configure
 * @param[in] 	irqPriority		Priority to assign
 *
 * @return		none
 */
void I2C_IRQPriorityConfig(uint8_t irqNumber, uint8_t irqPriority)
{
	uint8_t bits = (PRI_LEVELS - 1) & irqPriority; // Mask bits up to implemented priority levels

	NVIC_IPR->PRI[irqNumber] &= ~ (0xF << (8 - PRI_BITS)); // Clear
	NVIC_IPR->PRI[irqNumber] |= (bits << (8 - PRI_BITS)); // Set
}

/** @brief 		I2C IRQ Event Callback
 *
 * @param[in] 	*pI2CHandle	I2C handle structure with peripheral base address and configuration
 * @param[in] 	event		Event callback code
 *
 * @return		none
 *
 */
__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event)
{
	// Weak palceholder. Must be implemented by the application
}
