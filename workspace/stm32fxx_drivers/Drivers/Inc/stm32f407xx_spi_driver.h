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
 * APIs supported by the GPIO driver (See function definitions for more information)
 ****************************************************************************************/



#endif /* STM32F407XX_SPI_DRIVER_H */
