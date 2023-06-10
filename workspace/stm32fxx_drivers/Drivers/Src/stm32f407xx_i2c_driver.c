/*******************************************************************************
 * Filename		: stm32f407xx_i2c_driver.c
 * Description	: STM32F407xx MCU specific I2C driver source file
 * Author		: Kyungjae Lee
 * History		: Jun 09, 2023 - Created file
 ******************************************************************************/

#include "stm32f407xx.h"

/*******************************************************************************
 * APIs supported by the SPI driver
 * (See function definitions for more information)
 ******************************************************************************/

/**
 * I2C_PeriClockControl()
 * Desc.	:
 * Param.	:
 * Returns	:
 * Note		:
 */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t state)
{
}

/**
 * I2C_Init()
 * Desc.	:
 * Param.	:
 * Returns	:
 * Note		:
 */
void I2C_Init(I2C_Handle_TypeDef *pI2CHandle)
{
}

/**
 * I2C_DeInit()
 * Desc.	:
 * Param.	:
 * Returns	:
 * Note		:
 */
void I2C_DeInit(I2C_TypeDef *pI2Cx)
{
}

/**
 * I2C_IRQInterruptConfig()
 * Desc.	:
 * Param.	:
 * Returns	:
 * Note		:
 */
void I2C_IRQInterruptConfig(uint8_t irqNumber, uint8_t state)
{
}

/**
 * I2C_IRQPriorityConfig()
 * Desc.	:
 * Param.	:
 * Returns	:
 * Note		:
 */
void I2C_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
}

/**
 * I2C_PeriControl()
 * Desc.	:
 * Param.	:
 * Returns	:
 * Note		:
 */
void I2C_PeriControl(I2C_TypeDef *pI2Cx, uint8_t state)
{
}

/**
 * Application callback functions (Must be implemented by application)
 * Note: Since the driver does not know in which application this function will
 * 		 be implemented, it is good idea to give a weak function definition.
 */
/**
 * I2C_ApplicationEventCallback()
 * Desc.	:
 * Param.	:
 * Returns	:
 * Note		:
 */
__WEAK void I2C_ApplicationEventCallback(I2C_Handle_TypeDef *pI2CHandle, uint8_t appEvent)
{
}
