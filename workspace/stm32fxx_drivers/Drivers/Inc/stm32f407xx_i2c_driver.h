/*******************************************************************************
 * File		: stm32f407xx_i2c_driver.h
 * Brief	: STM32F407xx MCU specific I2C driver header file
 * Author	; Kyungjae Lee
 * Date		: Jun 09, 2023
 *
 * Note		: This code includes only the features that are necessary for my
 * 			  personal projects.
 * ****************************************************************************/

#ifndef STM32F407XX_I2C_DRIVER_H
#define STM32F407XX_I2C_DRIVER_H

#include "stm32f407xx.h"

/*******************************************************************************
 * I2Cx peripheral structures
 ******************************************************************************/

/* I2Cx peripheral configuration structure */
typedef struct
{
	uint32_t I2C_SCLSpeed;		/* Available values @I2C_SCLSpeed	 		*/
	uint8_t I2C_DeviceAddress;	/* Value will be entered by user			*/
	uint8_t I2C_ACKEnable;		/* Available values @I2C_ACKEnable	 		*/
	uint8_t I2C_FMDutyCycle;	/* Available values @I2C_FMDutyCycle 		*/
} I2C_Config_TypeDef;

/* I2Cx peripheral handle structure */
typedef struct
{
	I2C_TypeDef 		*pI2Cx;	/* Base address of I2Cx(x:1,2,3) peripheral */
	I2C_Config_TypeDef 	I2C_Config;
	uint8_t				*pTxBuffer; 	/* Application Tx buffer address 	*/
	uint8_t				*pRxBuffer; 	/* Application Rx buffer address 	*/
	uint32_t			TxLen; 			/* Number of bytes left to transmit */
	uint32_t			RxLen; 			/* Number of bytes left to receive 	*/
	uint8_t				TxRxState; 		/* Available values @I2C_TxRxState	*/
	uint8_t				DevAddr; 		/* Slave/device address          	*/
	uint32_t			RxSize; 		/* Total size of data to receive 	*/
	uint8_t				RepeatedStart; 	/* Repeated start enable/disable 	*/
} I2C_Handle_TypeDef;

/**
 * @I2C_SCLSpeed
 * Note: Any clock frequency greater than 100 KHz is considered "fast mode".
 */
#define I2C_SCL_SPEED_SM		100000	/* 100 KHz */
#define I2C_SCL_SPEED_FM_2K		200000	/* 200 KHz */
#define I2C_SCL_SPEED_FM_4K		400000	/* 400 KHz */

/**
 * @I2C_ACKEnable
 * Note: ACKing is disabled by default.
 */
#define I2C_ACK_DISABLE			0
#define I2C_ACK_ENABLE			1

/**
 * @I2C_FMDutyCycle (Fm mode duty cycle)
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/**
 * I2C_TxRxState (I2C application communication state)
 * Note: Unlike the SPI communication, I2C communication is half-duplex.
 * 		 Therefore, only one communication state variable is necessary.
 */
#define I2C_READY				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2

/**
 * I2C application event macros
 */
#define I2C_EV_TX_CMPLT			0
#define I2C_EV_RX_CMPLT			1
#define I2C_EV_STOP				2
#define I2C_ERROR_BERR			3
#define I2C_ERROR_ARLO			4
#define I2C_ERROR_AF  			5
#define I2C_ERROR_OVR 			6
#define I2C_ERROR_TIMEOUT		7
#define I2C_EV_TX				8
#define I2C_EV_RX				9


/*******************************************************************************
 * APIs supported by the I2C driver
 * (See function definitions for more information)
 ******************************************************************************/

/* Peripheral clock setup */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t state);

/**
 * Init and De-init
 */
void I2C_Init(I2C_Handle_TypeDef *pI2CHandle);
void I2C_DeInit(I2C_TypeDef *pI2Cx);	/* Utilize RCC_AHBxRSTR (AHBx peripheral reset register) */

/**
 * Data send and receive
 */
void I2C_MasterTxBlocking(I2C_Handle_TypeDef *pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t repeatedStartState);
void I2C_MasterRxBlocking(I2C_Handle_TypeDef *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t repeatedStartState);
uint8_t I2C_MasterTxInterrupt(I2C_Handle_TypeDef *pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t repeatedStartState);
uint8_t I2C_MasterRxInterrupt(I2C_Handle_TypeDef *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t repeatedStartState);
void I2C_CloseTx(I2C_Handle_TypeDef *pI2CHandle);
void I2C_CloseRx(I2C_Handle_TypeDef *pI2CHandle);
void I2C_SlaveTx(I2C_TypeDef *pI2Cx, uint8_t data);
uint8_t I2C_SlaveRx(I2C_TypeDef *pI2Cx);

/**
 * IRQ configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t irqNumber, uint8_t state);
void I2C_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void I2C_EV_IRQHandling(I2C_Handle_TypeDef *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_TypeDef *pI2CHandle);

/**
 * Other peripheral control APIs
 */
void I2C_PeriControl(I2C_TypeDef *pI2Cx, uint8_t state);
void I2C_ManageACK(I2C_TypeDef *pI2Cx, uint8_t state);
void I2C_GenerateSTARTCondition(I2C_TypeDef *pI2Cx);
void I2C_GenerateSTOPCondition(I2C_TypeDef *pI2Cx);
void I2C_SlaveEnableDisableCallbackEvents(I2C_TypeDef *pI2Cx, uint8_t state);

/**
 * Application callback functions (Must be implemented by application)
 * Note: Since the driver does not know in which application this function will
 * 		 be implemented, it is good idea to give a weak function definition.
 */
void I2C_ApplicationEventCallback(I2C_Handle_TypeDef *pI2CHandle, uint8_t appEvent);


#endif /* STM32F407XX_I2C_DRIVER_H */
