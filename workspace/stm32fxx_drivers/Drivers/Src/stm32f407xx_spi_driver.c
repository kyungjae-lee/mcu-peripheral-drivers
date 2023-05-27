/**
 * Filename		: stm32f407xx_spi_driver.c
 * Description	: STM32F407xx MCU specific GPIO driver source file
 * Author		: Kyungjae Lee
 * Created on	: May 27, 2023
 */

#include "stm32f407xx_spi_driver.h"

/*****************************************************************************************
 * APIs supported by the SPI driver (See function definitions for more information)
 ****************************************************************************************/

/**
 * Peripheral clock setup
 */

/**
 * SPI_PeriClockControl()
 * Desc.	: Enables or disables peripheral clock for SPIx
 * Param.	: @pSPIx - base address of SPIx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Returns	: None
 * Note		: N/A
 */
void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t state)
{
}

/**
 * Init and De-init
 */
void SPI_Init(SPI_Handle_TypeDef *pSPIHandle)
{
}

void SPI_DeInit(SPI_TypeDef *pSPIx)	/* Utilize RCC_AHBxRSTR (AHBx peripheral reset register) */
{
}

/**
 * Data send and receive
 * - Blocking type: Non-interrupt based
 * - Non-blocking type: interrupt based
 * - DMA based (Will not be considered in this project)
 *
 * Note: Standard practice for choosing the size of 'length' variable is uint32_t or greater
 */
void SPI_TxData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
}

void SPI_RxData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
}

/**
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t irqNumber, uint8_t state)
{
}
void SPI_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
}
void SPI_IRQHandling(SPI_Handle_TypeDef *pSPIHandle)
{
}
