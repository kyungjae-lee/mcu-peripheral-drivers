/**
 * Filename		: stm32f407xx_spi_driver.c
 * Description	: STM32F407xx MCU specific SPI driver source file
 * Author		: Kyungjae Lee
 * Created on	: May 27, 2023
 */

#include "stm32f407xx.h"

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
	if (state == ENABLE)
	{
		if (pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if (pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if (pSPIx == SPI3)
			SPI3_PCLK_EN();
		else if (pSPIx == SPI4)
			SPI4_PCLK_EN();
	}
	else
	{
		if (pSPIx == SPI1)
			SPI1_PCLK_DI();
		else if (pSPIx == SPI2)
			SPI2_PCLK_DI();
		else if (pSPIx == SPI3)
			SPI3_PCLK_DI();
		else if (pSPIx == SPI4)
			SPI4_PCLK_DI();
	}
}

/**
 * Init and De-init
 */

/**
 * SPI_Init()
 * Desc.	:
 * Param.	: @pSPIHandle - pointer to the SPI handle structure
 * Returns	: None
 * Note		: For the serial communication peripherals (e.g., SPI, I2C, ...), there
 * 			  may be
 * 			  - one or more control registers where configurable parameters are stored
 * 			  - one or more data registers where user data is stored
 * 			  - one or more status registers where various status flags are stored.
 */
void SPI_Init(SPI_Handle_TypeDef *pSPIHandle)
{
	/* Enable peripheral clock */
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/**
	 * Configure SPIx_CR1 register
	 */

	uint32_t temp = 0;	/* Temporary register to store SPIx_CR1 configuration information */

	/* Configure device mode (MSTR bit[2] - Master selection) */
	temp |= pSPIHandle->SPI_Config.SPI_DeviceMode << 2;

	/* Configure bus config
	 *
	 * Full Duplex: BIDIMODE=0, RXONLY=0
	 * Simplex (unidirectional receive-only): BIDIMODE=0, RXONLY=1
	 * Half-Duplex, Tx: BIDIMODE=1, BIDIOE=1
	 * Half-Duplex, Rx: BIDIMODE=1, BIDIOE=0
	 */
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX)
	{
		/* Clear BIDIMODE bit[15] - 2-line unidirectional data mode selected */
		temp &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX)
	{
		/* Set BIDIMODE bit[15] - 1-line bidirectional data mode selected */
		temp |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
	{
		/* Clear BIDIMODE bit[15] - 2-line unidirectional data mode selected */
		temp &= ~(1 << SPI_CR1_BIDIMODE);

		/* Set RXONLY bit[10] - Output disabled (Receive-only mode) */
		temp |= (1 << SPI_CR1_RXONLY);
	}

	/* Configure SPI serial clock speed (baud rate) */
	temp |= pSPIHandle->SPI_Config.SPI_SCLKSpeed << SPI_CR1_BR;

	/* Configure DFF */
	temp |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	/* Configure CPOL */
	temp |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	/* Configure CPHA */
	temp |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	/* Configure SSM */
	temp |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = temp;
}

/**
 * SPI_DeInit()
 * Desc.	:
 * Param.	: @pSPIx - base address of SPIx peripheral
 * Returns	: None
 * Note		: N/A
 */
void SPI_DeInit(SPI_TypeDef *pSPIx)	/* Utilize RCC_AHBxRSTR (AHBx peripheral reset register) */
{
	/* Set and clear the corresponding bit of RCC_APBxRSTR to reset */
	if (pSPIx == SPI1)
		SPI1_RESET();
	else if (pSPIx == SPI2)
		SPI2_RESET();
	else if (pSPIx == SPI3)
		SPI3_RESET();
	else if (pSPIx == SPI4)
		SPI4_RESET();
}

/**
 * Data send and receive
 * - Blocking type: Non-interrupt based
 * - Non-blocking type: interrupt based
 * - DMA based (Will not be considered in this project)
 *
 * Note: Standard practice for choosing the size of 'length' variable is uint32_t or greater
 */

/**
 * SPI_TxData()
 * Desc.	: Send from @pSPIx the data of length @len stored in @pTxBuffer
 * Param.	: @pSPIx - base address of SPIx peripheral
 * 			  @pTxBuffer - address of the Tx buffer
 * 			  @len - length of the data to transmit
 * Returns	: None
 * Note		: This is a blocking function. This function will not return until
 *            the data is fully sent out.
 */
void SPI_TxData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while (len > 0)
	{
		/* Wait until TXE (Tx buffer empty) bit is set */
		while (!(pSPIx->SR & (0x1 << SPI_SR_TXE)));	/* Blocking (Polling for the TXE flag to set) */

		/* Check DFF (Data frame format) bit in SPIx_CR1 */
		if (pSPIx->CR1 & (0x1 << SPI_CR1_DFF))
		{
			/* 16-bit DFF */
			/* Load the data into DR */
			pSPIx->DR = *((uint16_t *)pTxBuffer);	/* Make it 16-bit data */

			/* Decrement the length (2 bytes) */
			len--;
			len--;

			/* Adjust the buffer pointer */
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			/* 8-bit DFF */
			/* Load the data into DR */
			pSPIx->DR = *pTxBuffer;

			/* Decrement the length (1 byte) */
			len--;

			/* Adjust the buffer pointer */
			pTxBuffer++;
		}
	}
}

/**
 * ()
 * Desc.	:
 * Param.	: @
 * Returns	: None
 * Note		: N/A
 */
void SPI_RxData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
}

/**
 * IRQ configuration and ISR handling
 */

/**
 * ()
 * Desc.	:
 * Param.	: @
 * Returns	: None
 * Note		: N/A
 */
void SPI_IRQInterruptConfig(uint8_t irqNumber, uint8_t state)
{
}

/**
 * ()
 * Desc.	:
 * Param.	: @
 * Returns	: None
 * Note		: N/A
 */
void SPI_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
}

/**
 * ()
 * Desc.	:
 * Param.	: @
 * Returns	: None
 * Note		: N/A
 */
void SPI_IRQHandling(SPI_Handle_TypeDef *pSPIHandle)
{
}

/**
 * SPI_PeriControl()
 * Desc.	: Enables or disables SPI peripheral @pSPIx
 * Param.	: @pSPIx - base address of SPIx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Returns	: None
 * Note		: N/A
 */
void SPI_PeriControl(SPI_TypeDef *pSPIx, uint8_t state)
{
	if (state == ENABLE)
		pSPIx->CR1 |= (0x1 << SPI_CR1_SPE);		/* Enable */
	else
		pSPIx->CR1 &= ~(0x1 << SPI_CR1_SPE);	/* Disable */
}

/**
 * SPI_SSIConfig()
 * Desc.	: Sets or resets SPI CR1 register's SSI (Internal Slave Select) bit
 * Param.	: @pSPIx - base address of SPIx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Returns	: None
 * Note		: This bit has an effect only when the SSM bit is set.
 * 			  The value of this bit is forced onto the NSS pin and
 * 			  the IO value of the NSS pin is ignored.
 * 			  This bit is not used in I2S mode and SPI TI mode.
 */
void SPI_SSIConfig(SPI_TypeDef *pSPIx, uint8_t state)
{
	if (state == ENABLE)
		pSPIx->CR1 |= (0x1 << SPI_CR1_SSI);		/* Enable */
	else
		pSPIx->CR1 &= ~(0x1 << SPI_CR1_SSI);	/* Disable */
}

