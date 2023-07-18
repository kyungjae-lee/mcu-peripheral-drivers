/*******************************************************************************
 * File		: stm32f407xx_spi_driver.c
 * Brief	: STM32F407xx MCU specific SPI driver source file
 * Author	; Kyungjae Lee
 * Date		: May 21, 2023
 *
 * Note		: This code includes only the features that are necessary for my
 * 			  personal projects.
 * ****************************************************************************/

#include "stm32f407xx.h"

/* Private helper functions */
static void SPI_TXE_InterruptHandle(SPI_Handle_TypeDef *pSPIHandle);
static void SPI_RXNE_InterruptHandle(SPI_Handle_TypeDef *pSPIHandle);
static void SPI_OVR_InterruptHandle(SPI_Handle_TypeDef *pSPIHandle);

/*******************************************************************************
 * APIs supported by the SPI driver
 * (See function definitions for more information)
 ******************************************************************************/

/**
 * SPI_PeriClockControl()
 * Brief	: Enables or disables peripheral clock for SPIx
 * Param	: @pSPIx - base address of SPIx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
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
} /* End of SPI_PeriClockControl */

/**
 * SPI_Init()
 * Brief	: Initializes SPI peripheral
 * Param	: @pSPIHandle - pointer to the SPI handle structure
 * Retval	: None
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
} /* End of SPI_Init */

/**
 * SPI_DeInit()
 * Brief	: De-initializes SPI peripheral
 * Param	: @pSPIx - base address of SPIx peripheral
 * Retval	: None
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
} /* End of SPI_DeInit */

/**
 * Data send and receive
 * - Blocking type: Non-interrupt based
 * - Non-blocking type: interrupt based
 * - DMA based (Will not be considered in this project)
 *
 * Note: Standard practice for choosing the size of 'length' variable is uint32_t or greater
 */

/**
 * SPI_TxBlocking()
 * Brief	: Send from @pSPIx the data of length @len stored in @pTxBuffer
 * Param	: @pSPIx - base address of SPIx peripheral
 * 			  @pTxBuffer - address of the Tx buffer
 * 			  @len - length of the data to transmit
 * Retval	: None
 * Note		: This is a blocking function. This function will not return until
 *            the data is fully sent out.
 */
void SPI_TxBlocking(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t len)
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
} /* End of SPI_TxBlocking */

/**
 * SPI_RxBlocking()
 * Brief	: Receive the data of length @len stored in @pRxBuffer
 * Param	: @pSPIx - base address of SPIx peripheral
 * 			  @pRxBuffer - address of the Rx buffer
 * 			  @len - length of the data to transmit
 * Retval	: None
 * Note		: This is a blocking function. This function will not return until
 *            the data is fully received.
 */
void SPI_RxBlocking(SPI_TypeDef *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while (len > 0)
	{
		/* Wait until RXNE (Rx buffer not empty) bit is set */
		while (!(pSPIx->SR & (0x1 << SPI_SR_RXNE)));	/* Blocking (Polling for the RXNE flag to set) */

		/* Check DFF (Data frame format) bit in SPIx_CR1 */
		if (pSPIx->CR1 & (0x1 << SPI_CR1_DFF))
		{
			/* 16-bit DFF */
			/* Read the data from DR into RxBuffer */
			*((uint16_t *)pRxBuffer) = pSPIx->DR;

			/* Decrement the length (2 bytes) */
			len--;
			len--;

			/* Adjust the buffer pointer */
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			/* 8-bit DFF */
			/* Read the data from DR into RxBuffer */
			*pRxBuffer = pSPIx->DR;

			/* Decrement the length (1 byte) */
			len--;

			/* Adjust the buffer pointer */
			pRxBuffer++;
		}
	}
} /* End of SPI_RxBlocking */

/**
 * SPI_TxInterrupt()
 * Brief	: Fills out the necessary information (@pTxBuffer, @len, SPI state) to
 * 			  transmit data, set TXEIE bit and returns
 * Param	: @pSPIHandle - pointer to SPI handle structure
 * 			  @pTxBuffer - address of the Tx buffer
 * 			  @len - length of the data to transmit
 * Retval	: Tx state of the application
 * Note		: This is a non-blocking function. This function does not write
 * 			  data into the data register. Writing will be taken care of by
 * 			  the interrupt handler.
 */
uint8_t SPI_TxInterrupt(SPI_Handle_TypeDef *pSPIHandle, uint8_t volatile *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX)
	{
		/* 1. Save the Tx buffer address and length info in some global variables */
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		/* 2. Mark the SPI state as busy in transmission so that no other code
		 *    can take over the same SPI peripheral until transmission is over.
		 */
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		/* 3. Enable the SPI_CR2 TXEIE control bit to get interrupt whenever TXE flag
		 *    is set in SR.
		 */
		pSPIHandle->pSPIx->CR2 |= (0x1 << SPI_CR2_TXEIE);
	}

	/* 4. Data transmission will be handled by the ISR code. */

	return state;
}

/**
 * SPI_RxInterrupt()
 * Brief	: Fills out the necessary information (@pTxBuffer, @len, SPI state) to
 * 			  receive data, set RXNEIE bit and returns
 * Param	: @pSPIHandle - pointer to SPI handle structure
 * 			  @pRxBuffer - address of the Rx buffer
 * 			  @len - length of the data to transmit
 * Retval	: Rx state of the application
 * Note		: This is a non-blocking function. This function does not read
 * 			  data from the data register. Reading will be taken care of by
 * 			  the interrupt handler.
 */
uint8_t SPI_RxInterrupt(SPI_Handle_TypeDef *pSPIHandle, uint8_t volatile *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX)
	{
		/* 1. Save the Rx buffer address and length info in some global variables */
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		/* 2. Mark the SPI state as busy in reception so that no other code
		 *    can take over the same SPI peripheral until reception is over.
		 */
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		/* 3. Enable the SPI_CR2 RXNEIE control bit to get interrupt whenever RXNE flag
		 *    is set in SR.
		 */
		pSPIHandle->pSPIx->CR2 |= (0x1 << SPI_CR2_RXNEIE);
	}

	/* 4. Data reception will be handled by the ISR code. */

	return state;
} /* End of SPI_RxInterrupt */

/**
 * SPI_PeriControl()
 * Brief	: Enables or disables SPI peripheral @pSPIx
 * Param	: @pSPIx - base address of SPIx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: N/A
 */
void SPI_PeriControl(SPI_TypeDef *pSPIx, uint8_t state)
{
	if (state == ENABLE)
		pSPIx->CR1 |= (0x1 << SPI_CR1_SPE);		/* Enable */
	else
		pSPIx->CR1 &= ~(0x1 << SPI_CR1_SPE);	/* Disable */
} /* End of SPI_PeriControl */

/**
 * SPI_SSIConfig()
 * Brief	: Sets or resets SPI CR1 register's SSI (Internal Slave Select) bit
 * Param	: @pSPIx - base address of SPIx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
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
} /* End of SPI_SSIConfig */

/**
 * SPI_SSOEConfig()
 * Brief	: Sets or resets SPI CR2 register's SSOE (Slave Select Output Enable) bit
 * Param	: @pSPIx - base address of SPIx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: When SSOE = 1, SS output is disabled in master mode and the cell
 * 			  can work in multimaster configuration
 * 			  When SSOE = 0, SS output is enabled in master mode and when the cell
 * 			  is enabled. The cell cannot work in a multimaster environment.
 */
void SPI_SSOEConfig(SPI_TypeDef *pSPIx, uint8_t state)
{
	if (state == ENABLE)
		pSPIx->CR2 |= (0x1 << SPI_CR2_SSOE);	/* Enable */
	else
		pSPIx->CR2 &= ~(0x1 << SPI_CR2_SSOE);	/* Disable */
} /* End of SPI_SSOEConfig */

/**
 * SPI_IRQInterruptConfig()
 * Brief	: Configures IRQ interrupts (processor; NVIC_ISERx, NVIC_ICERx)
 * Param	: @irqNumber - IRQ number
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: Reference - Cortex-M4 Devices Generic User Guide
 * 			  Clearing ISERx bit won't disable the interrupt.
 * 			  To disable interrupt ICERx bit has to be set!
 */
void SPI_IRQInterruptConfig(uint8_t irqNumber, uint8_t state)
{
	if (state == ENABLE)
	{
		/* Configure NVIC_ISERx register */
		if (irqNumber <= 31)
			*NVIC_ISER0 |= (0x1 << irqNumber);
		else if (32 <= irqNumber && irqNumber <= 63)
			*NVIC_ISER1 |= (0x1 << irqNumber % 32);
		else if (64 <= irqNumber && irqNumber <= 95)
			*NVIC_ISER2 |= (0x1 << irqNumber % 32);
	}
	else
	{
		/* Configure NVIC_ICERx register */
		if (irqNumber <= 31)
			*NVIC_ICER0 |= (0x1 << irqNumber);
		else if (32 <= irqNumber && irqNumber <= 63)
			*NVIC_ICER1 |= (0x1 << irqNumber % 32);
		else if (64 <= irqNumber && irqNumber <= 95)
			*NVIC_ICER2 |= (0x1 << irqNumber % 32);
	}
} /* End of SPI_IRQInterruptConfig */

/**
 * SPI_OIRQPriorityConfig()
 * Brief	: Configures IRQ priority (processor; NVIC_IPRx)
 * Param	: @irqNumber - IRQ number
 * 			  @irqPriotity - IRQ priority (Make sure this parameter is of
 * 			  				 type uint32_t. Due to the number of bits it
 * 			  				 needs to be shifted during the calculation,
 * 							 declaring it as uint8_t did not do its job.)
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: Reference - Cortex-M4 Devices Generic User Guide
 * 			  STM32F407xx MCUs use only 4 most significant bits within
 * 			  each 8-bit section of IPRx. Make sure to account for
 * 			  this offset when calculating the place to write the
 * 			  priority.
 */
void SPI_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
	/* Find out the IPR register */
	uint8_t iprNumber = irqNumber / 4;
	uint8_t iprSection = irqNumber % 4;
	uint8_t bitOffset = (iprSection * 8) + (8 - NUM_PRI_BITS_USED);
	*(NVIC_IPR_BASE + iprNumber) |= (irqPriority << bitOffset);
} /* End of SPI_IRQPriorityConfig */

/**
 * SPI_IRQHandling()
 * Brief	: Handles SPI interrupt
 * Param	: @pSPIHandle - pointer to SPI handle structure
 * Retval	: None
 * Note		: SPI interrupt event flags (TXE, RXNE, MODF, OVR, CRCERR, FRE)
 */
void SPI_IRQHandling(SPI_Handle_TypeDef *pSPIHandle)
{
	uint8_t temp1, temp2;

	/* Check for TXE event */
	temp1 = pSPIHandle->pSPIx->SR & (0x1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (0x1 << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		/* Handle TXE interrupt */
		SPI_TXE_InterruptHandle(pSPIHandle);	/* Helper function */
	}

	/* Check for RXNE event */
	temp1 = pSPIHandle->pSPIx->SR & (0x1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (0x1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		/* Handle RXNE interrupt */
		SPI_RXNE_InterruptHandle(pSPIHandle);	/* Helper function */
	}

	/* Check for OVR (overrun) event (OVR flag will be set when receiving data
	 * when Rx buffer is still not empty. In this condition, all the subsequently
	 * received data bytes will be discarded. So, the OVR flag must be cleared.
	 */
	temp1 = pSPIHandle->pSPIx->SR & (0x1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (0x1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		/* Handle OVR interrupt */
		SPI_OVR_InterruptHandle(pSPIHandle);	/* Helper function */
	}

	/* Will not consider MODF, CRCERR events here */
} /* End of SPI_IRQHandling */

/**
 * Private helper functions
 */

/**
 * SPI_TXE_InterruptHandle()
 * Brief	: Handles TXE interrupt
 * Param	: @pSPIHandle - pointer to SPI handle structure
 * Retval	: None
 * Note		: This is a private helper function and it not intended to be
 * 			  called by the user application. Thus defined as static function.
 */
static void SPI_TXE_InterruptHandle(SPI_Handle_TypeDef *pSPIHandle)
{
	/* Check DFF (Data frame format) bit in SPIx_CR1 */
	if (pSPIHandle->pSPIx->CR1 & (0x1 << SPI_CR1_DFF))
	{
		/* 16-bit DFF */
		/* Load the data into DR */
		pSPIHandle->pSPIx->DR = *((uint16_t *)(pSPIHandle->pTxBuffer));	/* Make it 16-bit data */

		/* Decrement the length (2 bytes) */
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;

		/* Adjust the buffer pointer */
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}
	else
	{
		/* 8-bit DFF */
		/* Load the data into DR */
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);

		/* Decrement the length (1 byte) */
		pSPIHandle->TxLen--;

		/* Adjust the buffer pointer */
		(pSPIHandle->pTxBuffer)++;
	}

	if (!pSPIHandle->TxLen)
	{
		/* TxLen is 0, so close the SPI transmission and inform the application
		 * Tx is over.
		 */
		SPI_CloseTx(pSPIHandle);

		/* Inform the application */
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
			/* This callback function must be implemented by the application */
	}
} /* End of SPI_TXE_InterruptHandle */

/**
 * SPI_RXNE_InterruptHandle()
 * Brief	: Handles RXNE interrupt
 * Param	: @pSPIHandle - pointer to SPI handle structure
 * Retval	: None
 * Note		: This is a private helper function and it not intended to be
 * 			  called by the user application. Thus defined as static function.
 */
static void SPI_RXNE_InterruptHandle(SPI_Handle_TypeDef *pSPIHandle)
{
	/* Check DFF (Data frame format) bit in SPIx_CR1 */
	if (pSPIHandle->pSPIx->CR1 & (0x1 << SPI_CR1_DFF))
	{
		/* 16-bit DFF */
		/* Copy the contents of DR into Rx buffer */
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;

		/* Decrement the length (2 bytes) */
		pSPIHandle->RxLen -= 2;

		/* Adjust the buffer pointer */
		(uint16_t *)(pSPIHandle->pRxBuffer)++;
	}
	else
	{
		/* 8-bit DFF */
		/* Copy the contents of DR into Rx buffer */
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;

		/* Decrement the length (1 byte) */
		pSPIHandle->RxLen--;

		/* Adjust the buffer pointer */
		(pSPIHandle->pRxBuffer)++;
	}

	if (!pSPIHandle->RxLen)
	{
		/* TxLen is 0, so close the SPI reception and inform the application
		 * Rx is over.
		 */
		SPI_CloseRx(pSPIHandle);

		/* Inform the application */
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
			/* This callback function must be implemented by the application */
	}
} /* End of SPI_RXNE_InterruptHandle */

/**
 * SPI_OVR_InterruptHandle()
 * Brief	: Handles OVR interrupt
 * Param	: @pSPIHandle - pointer to SPI handle structure
 * Retval	: None
 * Note		: This is a private helper function and it not intended to be
 * 			  called by the user application. Thus defined as static function.
 */
static void SPI_OVR_InterruptHandle(SPI_Handle_TypeDef *pSPIHandle)
{
	uint8_t temp;

	/* Clear the OVR flag */
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
		/* In case you need to read data from a variable/register without having to store it,
		 * you can typecast it to void. It will suppress compiler 'unused variable 'warnings.
		 */

	/* Inform the application */
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR);
		/* This callback function must be implemented by the application */

} /* End of SPI_OVR_InterruptHandle */

/**
 * SPI_ClearOVRFlag
 * Brief	: Clears the OVR flag
 * Param	: @pSPIx - base address of SPIx peripheral
 * Retval	: None
 * Note		: N/A
 */
void SPI_ClearOVRFlag(SPI_TypeDef *pSPIx)
{
	uint8_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
		/* In case you need to read data from a variable/register without having to store it,
		 * you can typecast it to void. It will suppress compiler 'unused variable 'warnings.
		 */
} /* SPI_ClearOVRFlag */

/**
 * SPI_CloseTx()
 * Brief	: Closes Tx operation
 * Param	: @pSPIHandle - pointer to SPI handle structure
 * Retval	: None
 * Note		: Application call this function to abruptly close the SPI communication.
 */
void SPI_CloseTx(SPI_Handle_TypeDef *pSPIHandle)
{
	/* Disable TXE interrupt */
	pSPIHandle->pSPIx->CR2 &= ~(0x1 << SPI_CR2_TXEIE);

	/* Reset Tx buffer information and state */
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
} /* End of SPI_CloseTx */

/**
 * SPI_CloseRx()
 * Brief	: Closes Rx operation
 * Param	: @pSPIHandle - pointer to SPI handle structure
 * Retval	: None
 * Note		: Application call this function to abruptly close the SPI communication.
 */
void SPI_CloseRx(SPI_Handle_TypeDef *pSPIHandle)
{
	/* Disable RXNE interrupt */
	pSPIHandle->pSPIx->CR2 &= ~(0x1 << SPI_CR2_RXNEIE);

	/* Reset Rx buffer information and state */
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
} /* End of SPI_CloseRx */

/**
 * SPI_ApplicationEventCallback()
 * Brief	: Notifies the application of the event occurred
 * Param	: @pSPIHandle - pointer to SPI handle structure
 * 			  @appEvent - SPI event occurred
 * Retval	: None
 * Note		: This function must be implemented by the application. Since the driver
 * 			  does not know in which application this function will be implemented,
 * 			  the driver defines it as a weak function. The application may override
 * 			  this function.
 * 			  If the application does not implement this function, the following
 * 			  definition will be executed.
 */
__WEAK void SPI_ApplicationEventCallback(SPI_Handle_TypeDef *pSPIHandle, uint8_t appEvent)
{
	/* Intentionally left blank (Application will override this function) */
} /* End of SPI_ApplicationEventCallback */
