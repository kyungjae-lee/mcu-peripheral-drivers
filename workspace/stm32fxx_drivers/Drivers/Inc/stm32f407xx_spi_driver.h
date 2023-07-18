/*******************************************************************************
 * File		: stm32f407xx_spi_driver.h
 * Brief	: STM32F407xx MCU specific SPI driver header file
 * Author	; Kyungjae Lee
 * Date		: May 21, 2023
 *
 * Note		: This code includes only the features that are necessary for my
 * 			  personal projects.
 * ****************************************************************************/

#ifndef STM32F407XX_SPI_DRIVER_H
#define STM32F407XX_SPI_DRIVER_H

#include "stm32f407xx.h"

/*******************************************************************************
 * SPIx peripheral structures
 ******************************************************************************/

/* SPIx peripheral configuration structure */
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

/* SPIx peripheral handle structure */
typedef struct
{
	SPI_TypeDef 		*pSPIx;		/* Holds the base address of the SPIx(x:0,1,2) peripheral */
	SPI_Config_TypeDef 	SPI_Config;	/* SPIx peripheral configuration structure */
	uint8_t	volatile	*pTxBuffer;	/* App's Tx buffer address */
	uint8_t	volatile	*pRxBuffer;	/* App's Rx buffer address */
	uint32_t			TxLen;		/* Number of bytes left to transmit */
	uint32_t			RxLen;		/* Number of bytes left to receive */
	uint8_t				TxState;	/* Available values @SPI_ApplicationStateus */
	uint8_t				RxState;	/* Available values @SPI_ApplicationStateus */
} SPI_Handle_TypeDef;

/**
 * @SPI_ApplicationStatus
 */
#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

/**
 * @SPI_ApplicationEvents
 */
#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR					3
#define SPI_EVENT_CRCERR				4

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


/*******************************************************************************
 * APIs supported by the SPI driver
 * (See function definitions for more information)
 ******************************************************************************/

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
void SPI_TxBlocking(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_RxBlocking(SPI_TypeDef *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_TxInterrupt(SPI_Handle_TypeDef *pSPIHandle, uint8_t volatile *pTxBuffer, uint32_t len);
uint8_t SPI_RxInterrupt(SPI_Handle_TypeDef *pSPIHandle, uint8_t volatile *pRxBuffer, uint32_t len);

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
void SPI_ClearOVRFlag(SPI_TypeDef *pSPIx);
void SPI_CloseTx(SPI_Handle_TypeDef *pSPIHandle);
void SPI_CloseRx(SPI_Handle_TypeDef *pSPIHandle);

/**
 * Application callback functions (Must be implemented by application)
 * Note: Since the driver does not know in which application this function will be
 * 	     implemented, it is good idea to give a weak function definition.
 */
void SPI_ApplicationEventCallback(SPI_Handle_TypeDef *pSPIHandle, uint8_t appEvent);


#endif /* STM32F407XX_SPI_DRIVER_H */
