/*
 * stm32f407xx_i2c.c
 *
 *  Created on: Sep 10, 2021
 *      Author: OSdZ
 */

#include "stm32f407xx_i2c.h"
#include "stm32f407xx_rcc.h"

/******************************
 *  I2C Function Definitions  *
 ******************************/

static void I2C_IssueStart(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= 0x1 << I2C_CR1_START;
}

static void I2C_IssueStop(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= 0x1 << I2C_CR1_STOP;
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
	pI2CHandle->pI2Cx->SR1;	// Touch registers but no need to do anything with the values
	pI2CHandle->pI2Cx->SR2;	// These two reads clear ADDR flag

// TODO: Check disassembly to verify the above causes correct behavior, otherwise replace with:
//	uint32_t touch;
//	touch = pI2CHandle->pI2Cx->SR1;	// Touch registers but no need to do anything with the values
//	touch = pI2CHandle->pI2Cx->SR2;	// These two reads clear ADDR flag
//	(void) touch;

}

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
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t flagPosition)
{
	if(pI2Cx->SR1 & flagPosition)
	{
		return FLAG_SET;
	}
	return FLAG_CLEAR;
}

/** @brief 		Master send data
 *
 * @param[in] 	*pI2CHandle		I2C handle structure with peripheral base address and configuration
 * @param[in] 	slaveAddress	Slave address
 * @param[in] 	*pTXBuffer		Pointer to data being sent
 * @param[in] 	len				Number of bytes to send
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
		I2C_IssueStop(pI2CHandle->pI2Cx);
}

/** @brief 		Master receive data
 *
 * @param[in] 	*pI2CHandle		I2C handle structure with peripheral base address and configuration
 * @param[in] 	slaveAddress	Slave address
 * @param[in] 	*pRXBuffer		Pointer in which to store to received data
 * @param[in] 	len				Number of bytes to receive
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
