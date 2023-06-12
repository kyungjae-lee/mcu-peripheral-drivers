/*******************************************************************************
 * Filename		: stm32f407xx_i2c_driver.h
 * Description	: STM32F407xx MCU specific I2C driver header file
 * Author		: Kyungjae Lee
 * History		: Jun 09, 2023 - Created file
 ******************************************************************************/

#ifndef STM32F407XX_I2C_DRIVER_H
#define STM32F407XX_I2C_DRIVER_H

#include "stm32f407xx.h"

/*******************************************************************************
 * I2Cx peripheral structures
 ******************************************************************************/

/* I2Cx peripheral configuration structure */
typedef struct
{
	uint8_t I2C_SCLSpeed;		/* Available values @I2C_SCLSpeed	 		*/
	uint8_t I2C_DeviceAddress;	/* Value will be entered by user			*/
	uint8_t I2C_ACKEnable;		/* Available values @I2C_ACKEnable	 		*/
	uint8_t I2C_FMDutyCycle;	/* Available values @I2C_FMDutyCycle 		*/
} I2C_Config_TypeDef;

/* I2Cx peripheral handle structure */
typedef struct
{
	I2C_TypeDef 		*pI2Cx;	/* Base address of I2Cx(x:1,2,3) peripheral */
	I2C_Config_TypeDef 	I2C_Config;
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
 * Note: Standard practice for choosing the size of 'length' variable is
 * 		 uint32_t or greater.
 */
void I2C_MasterTx(I2C_Handle_TypeDef *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t slaveAddr);


/**
 * IRQ configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t irqNumber, uint8_t state);
void I2C_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);

/**
 * Other peripheral control APIs
 */
/* Enable or disable the peripheral */
void I2C_PeriControl(I2C_TypeDef *pI2Cx, uint8_t state);

/**
 * Application callback functions (Must be implemented by application)
 * Note: Since the driver does not know in which application this function will
 * 		 be implemented, it is good idea to give a weak function definition.
 */
__WEAK void I2C_ApplicationEventCallback(I2C_Handle_TypeDef *pI2CHandle, uint8_t appEvent);


#endif /* STM32F407XX_I2C_DRIVER_H */
