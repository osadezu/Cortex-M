/*
 * stm32f407xx_i2c.h
 *
 *  Created on: Sep 10, 2021
 *      Author: OSdZ
 */

#ifndef INC_STM32F407XX_I2C_H_
#define INC_STM32F407XX_I2C_H_

#include "stm32f407xx.h"

// I2C Macros

#define I2C_WRITE			0
#define I2C_READ			1

#define I2C_NO_STOP			0	// Application will do a repeated start
#define I2C_WITH_STOP		1	// No Repeat Start


/// @I2C_SCLSpeed
/// I2C Clock Speed
#define I2C_SCL_SM			100000	// 100 KHz
#define I2C_SCL_FM_2K		200000	// 200 KHz
#define I2C_SCL_FM_4K		400000	// 400 KHz

/// @I2C_AckControl
/// I2C Acknowledge Control
#define I2C_ACK_DI			0		// No acknowledge return
#define I2C_ACK_EN			1		// Acknowledge after byte is received

/// @I2C_FMDutyCycle
/// I2C Fast Mode duty cycle
#define I2C_FM_DUTY_2		0		// Fm mode Tlow/Thi = 2
#define I2C_FM_DUTY_16_9	1		// Fm mode Tlow/Thi = 16/9

// I2C SR1 FLags

#define I2C_FLAG_SB         (1 << I2C_SR1_SB)           // 0: Start bit (Master mode)
#define I2C_FLAG_ADDR       (1 << I2C_SR1_ADDR)         // 1: Address sent (master mode)/matched (slave mode)
#define I2C_FLAG_BTF        (1 << I2C_SR1_BTF)          // 2: Byte transfer finished
#define I2C_FLAG_ADD10      (1 << I2C_SR1_ADD10)        // 3: 10-bit header sent (master mode)
#define I2C_FLAG_STOPF      (1 << I2C_SR1_STOPF)        // 4: Stop detection (slave mode)
#define I2C_FLAG_RXNE       (1 << I2C_SR1_RXNE)         // 6: Data register not empty (receivers)
#define I2C_FLAG_TXE        (1 << I2C_SR1_TXE)          // 7: Data register empty (transmitters)
#define I2C_FLAG_BERR       (1 << I2C_SR1_BERR)         // 8: Bus error
#define I2C_FLAG_ARLO       (1 << I2C_SR1_ARLO)         // 9: Arbitration lost (master mode)
#define I2C_FLAG_AF         (1 << I2C_SR1_AF)           // 10: Acknowledge failure
#define I2C_FLAG_OVR        (1 << I2C_SR1_OVR)          // 11: Overrun/underrun
#define I2C_FLAG_PECERR     (1 << I2C_SR1_PECERR)       // 12: PEC error in reception
#define I2C_FLAG_TIMEOUT    (1 << I2C_SR1_TIMEOUT)      // 14: Timeout or Tlow error
#define I2C_FLAG_SMBALERT   (1 << I2C_SR1_SMBALERT)     // 15: SMBus alert

//// I2C SR2 FLags
//
//#define I2C_FLAG_MSL        (1 << I2C_SR2_MSL)          // 0: Master/slave
//#define I2C_FLAG_BUSY       (1 << I2C_SR2_BUSY)         // 1: Bus busy
//#define I2C_FLAG_TRA        (1 << I2C_SR2_TRA)          // 2: Transmitter/receiver
//#define I2C_FLAG_GENCALL    (1 << I2C_SR2_GENCALL)      // 4: General call address (slave mode)
//#define I2C_FLAG_SMBDEFAULT (1 << I2C_SR2_SMBDEFAULT)   // 5: SMBus device default address
//#define I2C_FLAG_SMBHOST    (1 << I2C_SR2_SMBHOST)      // 6: SMBus host header (slave mode)
//#define I2C_FLAG_DUALF      (1 << I2C_SR2_DUALF)        // 7: Dual flag (slave mode)
//#define I2C_FLAG_PEC        (1 << I2C_SR2_PEC)          // 15:8 Packet error checking

// IRQ-based I2C operation states
#define I2C_READY		0
#define I2C_BUSY_TX		1
#define I2C_BUSY_RX		2

// I2C IRQ Callback Events
#define I2C_EVENT_TX_DONE	0
#define I2C_EVENT_RX_DONE	1
#define I2C_EVENT_STOP		2

#define I2C_EVENT_DATA_REQ	3	// Slave mode callback: Master requests data
#define I2C_EVENT_DATA_RCV	4	// Slave mode callback: Master sent data

#define I2C_ERROR_BERR		5
#define I2C_ERROR_ARLO		6
#define I2C_ERROR_AF		7
#define I2C_ERROR_OVR		8
#define I2C_ERROR_TIMEOUT	9


/*************************************
 *  Peripheral Structure Definitions *
 *************************************/

// Configuration Structure for I2Cx peripheral

typedef struct {
	uint32_t I2C_SCLSpeed;		//> From @I2C_SCLSpeed
	uint8_t  I2C_DeviceAddress;	// Given by application
	uint8_t  I2C_AckControl;	//> From @I2C_AckControl
	uint16_t I2C_FMDutyCycle;	//> From @I2C_FMDutyCycle
} I2C_Config_t;

// Handle Structure for I2Cx peripheral

typedef struct {
	I2C_RegDef_t	*pI2Cx;		// Pointer to base address of I2C peripheral
	I2C_Config_t	I2C_Config;	// I2C configuration settings
	uint8_t 		*pTxBuffer;	// Application Tx buffer address for IRQ transmit
	uint8_t 		*pRxBuffer;	// Application Rx buffer address for IRQ transmit
	uint32_t		TxLen;		// Tx data length for IRQ transmit
	uint32_t		RxLen;		// Rx data length for IRQ receive
	uint8_t			TxRxState;	// IRQ Tx or Rx state
	uint8_t			DeviceAddr;	// Address for the I2C transaction
	uint32_t		RxSize;		// Size of Rx buffer
	uint8_t			WithStop;	// Control logic for repeated start
} I2C_Handle_t;

/****************************
 *  API Function Prototypes *
 ****************************/

// Peripheral Clock Control

void I2C_ClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t state);							// I2C Clock Enable / Disable

// Peripheral Initialize, De-initialize and Enable

void I2C_Init(I2C_Handle_t *pI2CHandle);										// Initialize peripheral
// TODO: Method to initialize unused I2C_Config_t members to avoid setting registers with garbage values.
void I2C_Deinit(I2C_RegDef_t *pI2Cx);											// Return peripheral to reset state
void I2C_Control(I2C_RegDef_t *pI2Cx, uint8_t state);							// Peripheral Enable / Disable

// Miscellaneous I2C functions

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint16_t flagPosition);			// Check status of a register
void I2C_AckControl(I2C_RegDef_t *pI2Cx, uint8_t status);						// Enable or disable ACK bit
void I2C_IssueStop(I2C_RegDef_t *pI2Cx);										// Generate stop condition
void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle);							// End I2C Tx
void I2C_CloseReception(I2C_Handle_t *pI2CHandle);								// End I2C Rx
void I2C_CallbackEventsControl(I2C_RegDef_t *pI2Cx, uint8_t status);			// Enable or disable callback events for Slave mode

// Data Send and Receive

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t slaveAddress,
						uint8_t *pTXBuffer, uint32_t len, uint8_t withStop);	// Master send data
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t slaveAddress,
						uint8_t *pRXBuffer, uint32_t len, uint8_t withStop);	// Master receive data
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);						// Slave send data
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);								// Slave receive data
uint8_t I2C_MasterSendDataIRQ(I2C_Handle_t *pI2CHandle, uint8_t slaveAddress,
						uint8_t *pTXBuffer, uint32_t len, uint8_t withStop);	// IRQ based Master send data
uint8_t I2C_MasterReceiveDataIRQ(I2C_Handle_t *pI2CHandle, uint8_t slaveAddress,
						uint8_t *pRXBuffer, uint32_t len, uint8_t withStop);	// IRQ based Master receive data

// IRQ Configuration and IRQ Handling

void I2C_IRQConfig(uint8_t irqNumber, uint8_t state);							// Configure IRQ
void I2C_IRQPriorityConfig(uint8_t irqNumber, uint8_t irqPriority);				// Configure IRQ Priority
void I2C_HandleEventIRQ(I2C_Handle_t *pI2CHandle);								// Handle I2C Event IRQ
void I2C_HandleErrorIRQ(I2C_Handle_t *pI2CHandle);								// Handle I2C Error IRQ

// Application Callbacks
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event);		// I2C IRQ Event Callback

#endif /* INC_STM32F407XX_I2C_H_ */
