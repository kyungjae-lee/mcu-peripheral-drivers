/*
 * Filename		: stm32f407xx_spi_driver.h
 * Description	: STM32F407xx MCU specific SPI driver header file
 * Author		: Kyungjae Lee
 * Created on	: May 21, 2023
 */

#ifndef STM32F407XX_SPI_DRIVER_H
#define STM32F407XX_SPI_DRIVER_H

#include "stm32f407xx.h"

/**
 * SPIx peripheral configuration structure
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SCLKSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_TypeDef;

/**
 * SPIx peripheral handle structure
 */
typedef struct
{
	SPI_TypeDef *pSPIx;	/* Holds the base address of the SPIx(x:0,1,2) peripheral */
	SPI_Config_TypeDef SPI_Config;
} SPI_Handle_TypeDef;

/*****************************************************************************************
 * APIs supported by the SPI driver (See function definitions for more information)
 ****************************************************************************************/

/**
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t state);

/**
 * Init and De-init
 */
void SPI_Init(SPI_Handle_TypeDef *pSPIHandle);
void SPI_DeInit(SPI_TypeDef *pSPIx);	/* Utilize RCC_AHBxRSTR (AHBx peripheral reset register) */

/**
 * Data send and receive
 * - Blocking type: Non-interrupt based
 * - Non-blocking type: interrupt based
 * - DMA based (Will not be considered in this project)
 *
 * Note: Standard practice for choosing the size of 'length' variable is uint32_t or greater
 */
void SPI_TxData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_RxData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t len);

/**
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t irqNumber, uint8_t state);
void SPI_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void SPI_IRQHandling(SPI_Handle_TypeDef *pSPIHandle);


#endif /* STM32F407XX_SPI_DRIVER_H */
