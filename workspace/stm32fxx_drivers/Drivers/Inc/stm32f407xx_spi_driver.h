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
	uint8_t SPI_DeviceMode;		/* Available values @SPI_DeviceMode 		*/
	uint8_t SPI_BusConfig;		/* Available values @SPI_BusConfig	 		*/
	uint8_t SPI_SCLKSpeed;		/* Available values @SPI_SCKLSpeed	 		*/
	uint8_t SPI_DFF;			/* Available values @SPI_DFF		 		*/
	uint8_t SPI_CPOL;			/* Available values @SPI_CPOL		 		*/
	uint8_t SPI_CPHA;			/* Available values @SPI_CPHA		 		*/
	uint8_t SPI_SSM;			/* Available values @SPI_SSM		 		*/
} SPI_Config_TypeDef;

/**
 * SPIx peripheral handle structure
 */
typedef struct
{
	SPI_TypeDef *pSPIx;	/* Holds the base address of the SPIx(x:0,1,2) peripheral */
	SPI_Config_TypeDef SPI_Config;
} SPI_Handle_TypeDef;

/**
 * @SPI_DeviceMode
 * Note: SPI_CR1 MSTR bit[2] - Master selection
 */
#define SPI_DEVICE_MODE_SLAVE			0
#define SPI_DEVICE_MODE_MASTER			1

/**
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FULL_DUPLEX		0
#define SPI_BUS_CONFIG_HALF_DUPLEX		1
//#define SPI_BUS_CONFIG_SIMPLEX_TX_ONLY	2	// Simply removing Rx line = Same config as FD
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY	2

/**
 * @SPI_CLKSpeed
 * Note: SPI_CR1 BR[5:3] - Baud rate control
 */
#define SPI_SCLK_SPEED_PRESCALAR_2		0
#define SPI_SCLK_SPEED_PRESCALAR_4		1
#define SPI_SCLK_SPEED_PRESCALAR_8		2
#define SPI_SCLK_SPEED_PRESCALAR_16		3
#define SPI_SCLK_SPEED_PRESCALAR_32		4
#define SPI_SCLK_SPEED_PRESCALAR_64		5
#define SPI_SCLK_SPEED_PRESCALAR_128	6
#define SPI_SCLK_SPEED_PRESCALAR_256	7

/**
 * @SPI_DFF
 * Note: SPI_CR1 DFF bit[11] - Data frame format/
 */
#define SPI_DFF_8BITS					0	/* Default */
#define SPI_DFF_16BITS					1

/**
 * @SPI_CPOL
 * Note: SPI_CR1 CPOL bit[1] - Clock polarity
 */
#define SPI_CPOL_LOW					0	/* Default */
#define SPI_CPOL_HIGH					1

/**
 * @SPI_CPHA
 * Note: SPI_CR1 CPHA bit[0] - Clock phase
 */
#define SPI_CPHA_LOW					0	/* Default */
#define SPI_CPHA_HIGH					1

/**
 * @SPI_SSM
 * Note: SPI_CR1 SSM bit[9] - Software slave management
 */
#define SPI_SSM_DI						0	/* Default */
#define SPI_SSM_EN						1


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

/**
 * Other peripheral control APIs
 */
void SPI_PeriControl(SPI_TypeDef *pSPIx, uint8_t state);
void SPI_SSIConfig(SPI_TypeDef *pSPIx, uint8_t state);
void SPI_SSOEConfig(SPI_TypeDef *pSPIx, uint8_t state);

#endif /* STM32F407XX_SPI_DRIVER_H */
