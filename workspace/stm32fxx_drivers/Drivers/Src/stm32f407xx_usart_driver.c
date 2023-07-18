/*******************************************************************************
 * File		: stm32f407xx_usart_driver.c
 * Brief	: STM32F407xx MCU specific USART driver source file
 * Author	; Kyungjae Lee
 * Date		: Jun 20, 2023
 *
 * Note		: This code includes only the features that are necessary for my
 * 			  personal projects.
 * ****************************************************************************/

#include "stm32f407xx.h"

/*******************************************************************************
 * APIs supported by the I2C driver
 * (See function definitions for more information)
 ******************************************************************************/

/**
 * USART_PeriClockControl()
 * Brief	: Enables or disables USART peripheral clock
 * Param	: @pUSARTx - base address of USARTx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: N/A
 */
void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t state)
{
	if (state == ENABLE)
	{
		if (pUSARTx == USART1)
			USART1_PCLK_EN();
		else if (pUSARTx == USART2)
			USART2_PCLK_EN();
		else if (pUSARTx == USART3)
			USART3_PCLK_EN();
		else if (pUSARTx == UART4)
			UART4_PCLK_EN();
		else if (pUSARTx == UART5)
			UART5_PCLK_EN();
		else if (pUSARTx == USART6)
			USART6_PCLK_EN();
	}
	else
	{
		if (pUSARTx == USART1)
			USART1_PCLK_DI();
		else if (pUSARTx == USART2)
			USART2_PCLK_DI();
		else if (pUSARTx == USART3)
			USART3_PCLK_DI();
		else if (pUSARTx == UART4)
			UART4_PCLK_DI();
		else if (pUSARTx == UART5)
			UART5_PCLK_DI();
		else if (pUSARTx == USART6)
			USART6_PCLK_DI();
	}
} /* End of USART_PeriClockControl */

/**
 * USART_Init()
 * Brief	: Initializes USART peripheral
 * Param	: @pUSARTHandle - pointer to USART handle structure
 * Retval	: None
 * Note		: N/A
 */
void USART_Init(USART_Handle_TypeDef *pUSARTHandle)
{
	/* Temporary variable */
	uint32_t temp = 0;

	/* Configure USART_CR1 ---------------------------------------------------*/

	/* Enable USART peripheral clock */
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	/* Enable USART Tx and Rx engines */
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_RX)
	{
		/* Enable receiver (RE=1) */
		temp |= (0x1 << USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TX)
	{
		/* Enable transmitter (TE=1) */
		temp |= (0x1 << USART_CR1_TE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		/* Enable both the receiver and transmitter (RE=1, TE=1) */
		temp |= ((0x1 << USART_CR1_RE) | (0x1 << USART_CR1_TE));
	}

	/* Configure the word length */
	temp |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	/* Configure parity */
	if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EVEN)
	{
		/* Enable parity */
		temp |= (0x1 << USART_CR1_PCE);

		/* Select even parity (optional since even parity is selected by
		 * default when parity is enabled
		 */
		temp &= ~(0x1 << USART_CR1_PS);
	}
	else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_ODD)
	{
		/* Enable parity */
		temp |= (0x1 << USART_CR1_PCE);

		/* Select odd parity (optional since even parity is selected by
		 * default when parity is enabled
		 */
		temp |= (0x1 << USART_CR1_PS);
	}

	/* Write to USART_CR1 register */
	pUSARTHandle->pUSARTx->CR1 = temp;

	/* Configure USART_CR2 ---------------------------------------------------*/

	temp = 0;

	/* Configure the number of stop bits */
	temp |= pUSARTHandle->USART_Config.USART_NumOfStopBits << USART_CR2_STOP;

	/* Write to USART_CR2 register */
	pUSARTHandle->pUSARTx->CR2 = temp;

	/* Configure USART_CR3 ---------------------------------------------------*/

	temp = 0;

	/* Configure USART hardware flow control */
	if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		/* Enable CTS flow control */
		temp |= (0x1 << USART_CR3_CTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		/* Enable RTS flow control */
		temp |= (0x1 << USART_CR3_RTSE);
	}
	else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		/* Enable both the CTS and RTS flow control */
		temp |= (0x1 << USART_CR3_CTSE);
		temp |= (0x1 << USART_CR3_RTSE);
	}

	/* Write to USART_CR3 register */
	pUSARTHandle->pUSARTx->CR3 = temp;

	/* Configure USART_BRR (Baud rate register) ------------------------------*/

	/* Configure baudrate */
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

} /* End of USART_Init() */

/**
 * USART_DeInit()
 * Brief	: Deinitializes USART peripheral
 * Param	: @pUSARTx - base address of USARTx peripheral
 * Retval	: None
 * Note		: N/A
 */
void USART_DeInit(USART_TypeDef *pUSARTx)
{

} /* End of USART_DeInit */

/**
 * USART_TxBlocking()
 * Brief	: Handles blocking-based USART transmission
 * Param	: @pUSARTHandle - pointer to USART handle structure
 * 			  @pTxBuffer - pointer to Tx buffer
 * 			  @len -
 * Retval	: None
 * Note		: N/A
 */
void USART_TxBlocking(USART_Handle_TypeDef *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint16_t *pData;

	/* Repeat until @len number of bytes are transmitted */
	for (uint32_t i = 0; i < len; i++)
	{
		/* Wait until USART_SR_TXE flag is set.
		 * Wait until Transmit Data Register (TDR) is empty.
		 */
		while (!(pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_TXE)));

		/* Check word length configuration */
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			/* If 9-bit, load DR with 2 bytes with all bits other than the
			 * first 9 bits masked.
			 */
			pData = (uint16_t *)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pData & (uint16_t)0x01FF);

			/* Check parity bit configuration */
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				/* No parity bit is is used in this transfer. So, 9-bit user
				 * data will be sent. (Need 2 bytes to transfer 9 bits)
				 */
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				/* Parity bit is used in this transfer. So, 8-bit of user data
				 * will be sent.
				 * The 9th bit will be replaced by parity bit by the hardware.
				 */
				pTxBuffer++;
			}
		}
		else
		{
			/* 8-bit data transfer */
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

			/* Increment the Tx buffer address */
			pTxBuffer++;
		}
	}

	/* Wait till USART_SR_TC flag is set */
	while (!(pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_TC)));

} /* End of USART_TxBlocking */

/**
 * USART_RxBlocking()
 * Brief	: Handles blocking-based USART reception
 * Param	: @pUSARTHandle - pointer to USART handle structure
 * 			  @pRxBuffer - pointer to Rx buffer
 * 			  @len -
 * Retval	: None
 * Note		: N/A
 */
void USART_RxBlocking(USART_Handle_TypeDef *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
	/* Repeat until @len number of bytes are received */
	for (uint32_t i = 0; i < len; i++)
	{
		/* Wait until USART_SR_RXNE flag is set */
		while (!(pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_RXNE)));

		/* Check the word length configuration */
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			/* Expected to receive 9-bit data in a frame */

			/* Check the parity bit configuration */
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				/* No parity bit is is used in this transfer. So, read only the
				 * first 9 bits.
				 */
				*((uint16_t *)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

				/* Increment the Rx buffer address two times */
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				/* Parity bit is used in this transfer. So, 8-bit of user data
				 * will be received.
				 * The 9th bit will be replaced by parity bit by the hardware.
				 */
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
		else
		{
			/* 8-bit data transfer */
			pUSARTHandle->pUSARTx->DR = (*pRxBuffer & (uint8_t)0xFF);

			/* Increment the Tx buffer address */
			pRxBuffer++;
		}
	}
} /* End of USART_RxBlocking */

/**
 * USART_TxInterrupt()
 * Brief	: Handles interrupt-based USART transmission
 * Param	: @pUSARTHandle - pointer to USART handle structure
 * 			  @pTxBuffer - pointer to Tx buffer
 * 			  @len -
 * Retval	: None
 * Note		: N/A
 */
uint8_t USART_TxInterrupt(USART_Handle_TypeDef *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t txState = pUSARTHandle->TxBusyState;

	if (txState != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		/* Enable interrupt for TXE */
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_TXEIE);

		/* Enable interrupt for TC */
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_TCIE);
	}

	return txState;
} /* End of USART_TxInterrupt */

/**
 * USART_RxInterrupt()
 * Brief	: Handles interrupt-based USART reception
 * Param	: @pUSARTHandle - pointer to USART handle structure
 * 			  @pTxBuffer - pointer to Tx buffer
 * 			  @len -
 * Retval	: None
 * Note		: N/A
 */
uint8_t USART_RxInterrupt(USART_Handle_TypeDef *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t rxState = pUSARTHandle->RxBusyState;

	if (rxState != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		/* Enable interrupt for RXNE */
		pUSARTHandle->pUSARTx->CR1 |= (0x1 << USART_CR1_RXNEIE);
	}

	return rxState;
} /* End of USART_RxInterrupt */

/**
 * USART_IRQInterruptConfig()
 * Brief	: Enables or disables USART interrupts
 * Param	: @irqNumber - IRQ number
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: N/A
 */
void USART_IRQInterruptConfig(uint8_t irqNumber, uint8_t state)
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
} /* End of USART_IRQInterruptConfig */

/**
 * USART_IRQPriorityConfig()
 * Brief	: Configures USART IRQ interrupt priorities
 * Param	: @irqNumber - IRQ number
 * 			  @irqPriotity - IRQ priority (Make sure this parameter is of
 * 			  				 type uint32_t. Due to the number of bits it
 * 			  				 needs to be shifted during the calculation,
 * 							 declaring it as uint8_t did not do its job.
 * Retval	: None
 * Note		: N/A
 */
void USART_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
	/* Find out the IPR register */
	uint8_t iprNumber = irqNumber / 4;
	uint8_t iprSection = irqNumber % 4;
	uint8_t bitOffset = (iprSection * 8) + (8 - NUM_PRI_BITS_USED);
	*(NVIC_IPR_BASE + iprNumber) |= (irqPriority << bitOffset);
} /* End of USART_IRQPriorityConfig */

/**
 * USART_IRQHandling()
 * Brief	: Handles USART event IRQ
 * Param	: @pUSARTHandle - pointer to USART handle structure
 * Retval	: None
 * Note		: This function will first decode the event that occurred, and
 * 			  handle the event accordingly.
 * 			  The compiler will generate unused variable for 'temp3'. (In
 * 			  general, it is a good practice to check if an interrupt is
 * 			  enabled, along with its status bit. Currently, to generalize
 * 			  the code for all UARTs, we are not checking the CTSIE bit.)
 */
void USART_IRQHandling(USART_Handle_TypeDef *pUSARTHandle)
{
	uint32_t temp1, temp2, temp3;
	uint16_t *pData;

	/* Check for TC (Transmission Complete) flag -----------------------------*/

	/* Check the state of TC bit in SR */
	temp1 = pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_TC);

	/* Check the state of TCEIE bit */
	temp2 = pUSARTHandle->pUSARTx->CR1 & (0x1 << USART_CR1_TCIE);

	if (temp1 && temp2)
	{
		/* Handle interrupt triggered by TC flag */

		/* Close transmission and call application callback if TxLen is zero */
		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			/* If TxLen = 0, close the data transmission */
			if (! pUSARTHandle->TxLen)
			{
				/* Clear TC flag */
				pUSARTHandle->pUSARTx->SR &= ~(0x1 << USART_SR_TC);

				/* Clear TCIE control bit */

				/* Reset the application status */
				pUSARTHandle->TxBusyState = USART_READY;

				/* Reset Tx buffer address to NULL */
				pUSARTHandle->pTxBuffer = NULL;

				/* Reset TxLen to zero */
				pUSARTHandle->TxLen = 0;

				/* Notify the application of the event USART_EV_TX_CMPLT */
				USART_ApplicationEventCallback(pUSARTHandle, USART_EV_TX_CMPLT);
			}
		}
	}

	/* Check for TXE (Transmit data register Empty) flag ---------------------*/

	/* Check the state of TXE bit in SR */
	temp1 = pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_TXE);

	/* Check the state of TXIEIE bit in CR1 */
	temp2 = pUSARTHandle->pUSARTx->CR1 & (0x1 << USART_CR1_TXEIE);

	if (temp1 && temp2)
	{
		/* Handle interrupt triggered by TXE flag */

		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			/* Keep transmitting data until TxLen reaches to zero */
			if (pUSARTHandle->TxLen > 0)
			{
				/* Check whether the word length is 8-bit or 9-bit in a frame */
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					/* If 9-bit, load DR with 2 bytes with all bits other than the
					 * first 9 bits masked.
					 */
					pData = (uint16_t *)pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pData & (uint16_t)0x01FF);

					/* Check parity bit configuration */
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						/* Parity bit is not used, so 9-bit user data will be sent.
						 * Increment the Tx buffer address twice. */
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 2;
					}
					else
					{
						/* Parity bit is used, so 8-bit user data will be sent.
						 * The 9th bit will be replaced by the parity bit by the hardware.
						 */
					}
				}
				else
				{
					/* 8-bit data transmission */
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);

					/* Increment the Tx buffer address */
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen -= 1;
				}
			}

			if (pUSARTHandle->TxLen == 0)
			{
				/* Clear TXEIE bit (disable interrupt triggered by TXE flag */
				pUSARTHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_TXEIE);
			}
		}
	}

	/* Check for RXNE flag ---------------------------------------------------*/

	temp1 = pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (0x1 << USART_CR1_RXNEIE);

	if (temp1 && temp2)
	{
		/* Handle interrupt triggered by RXNE flag */

		if (pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if (pUSARTHandle-> RxLen > 0)
			{
				/* Check whether the word length is 8-bit or 9-bit in a frame */
				if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					/* Receiving 9-bit data in a frame */

					/* Check parity bit configuration */
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						/* Parity is not used, so all 9 bits will be user data */

						/* Read only the least significant 9 bits */
						*((uint16_t *)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

						/* Increment the Rx buffer address two times */
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 2;
					}
					else
					{
						/* Parity is used, so 8-bit will be user data and 1 bit parity */
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 1;
					}
				}
				else
				{
					/* Receiving 8-bit data in a frame */

					/* Check parity bit configuration */
					if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						/* Parity is not used, so all 8-bit will be user data */

						/* Read 8 bits from DR */
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
					}
					else
					{
						/* Parity is used, so 7-bit will be user data and 1 bit parity */

						/* Read only the least significant 7 bits */
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
					}

					/* Increment the Rx buffer address */
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen -= 1;
				}
			} /* End of if (pUSARTHandle-> RxLen > 0) */

			if (pUSARTHandle->RxLen == 0)
			{
				/* Clear RXNEIE bit (disable interrupt triggered by RXNE flag */
				pUSARTHandle->pUSARTx->CR1 &= ~(0x1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;

				/* Notify the application of the event USART_EV_RX_CMPLT */
				USART_ApplicationEventCallback(pUSARTHandle, USART_EV_RX_CMPLT);
			}
		}
	}

	/* Check for CTS flag ----------------------------------------------------*/
	/* Note: CTS feature is not applicable for UART4 and UART5 */

	/* Check the state of CTS bit in SR */
	temp1 = pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_CTS);

	/* Check the state of CTSE bit in CR3 */
	temp2 = pUSARTHandle->pUSARTx->CR3 & (0x1 << USART_CR3_CTSE);

	/* Check the state of CTSIE bit in CR3 */
	temp3 = pUSARTHandle->pUSARTx->CR3 & (0x1 << USART_CR3_CTSIE);

	if (temp1 && temp2)
	{
		/* Clear CTS flag in SR */
		pUSARTHandle->pUSARTx->SR &= ~(0x1 << USART_SR_CTS);

		/* Notify the application of the event USART_EV_CTS */
		USART_ApplicationEventCallback(pUSARTHandle, USART_EV_CTS);
	}

	/* Check for IDLE detection flag -----------------------------------------*/

	/* Check the state of IDLE bit in SR */
	temp1 = pUSARTHandle->pUSARTx->SR & (0x1 << USART_SR_IDLE);

	/* Check the state of IDLEIE bit in CR1 */
	temp2 = pUSARTHandle->pUSARTx->CR1 & (0x1 << USART_CR1_IDLEIE);

	if (temp1 && temp2)
	{
		/* Clear IDLE flag. (Check the reference manual for clear sequence) */
		temp1 = pUSARTHandle->pUSARTx->SR &= ~(0x1 << USART_SR_IDLE);

		/* Notify the application of the event USART_EV_IDLE */
		USART_ApplicationEventCallback(pUSARTHandle, USART_EV_IDLE);
	}

	/* Check for Overrun detection flag --------------------------------------*/

	/* Check the state of ORE flag in SR */
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	/* Check the state of RXNEIE bit in CR1 */
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;

	if (temp1 && temp2)
	{
		/* Need to clear the ORE flag here. Instead, give an API for the
		 * application to clear the ORE flag.
		 */

		/* Notify the application of the event USART_EV_IDLE */
		USART_ApplicationEventCallback(pUSARTHandle, USART_ER_ORE);
	}

	/* Check for Error flag --------------------------------------------------*/
	/* Note: Noise flag, overrun error and framing error in multibuffer
	 *		 communication. Multibuffer communication is not dealt in this
	 *		 driver. Please refer to the MCU reference manual.
	 *
	 *		 The following code will be executed in multibuffer mode only!
	 */

	temp2 = pUSARTHandle->pUSARTx->CR3 & (0x1 << USART_CR3_EIE);

	if (temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->SR;

		if (temp1 & (0x1 << USART_SR_FE))
		{
			/* This bit is set by hardware when a de-synchronization, excessive
			 * noise or break character is detected. It is cleared by a software
			 * sequence (i.e., reading USART_SR followed by reading USART_DR).
			 */
			USART_ApplicationEventCallback(pUSARTHandle, USART_ER_FE);
		}

		if (temp1 & (0x1 << USART_SR_NF))
		{
			/* This bit si set by hardware when noise is detected on a received
			 * frame. It is cleared software sequence (i.e., reading USART_SR
			 * followed by reading USART_DR).
			 */
			USART_ApplicationEventCallback(pUSARTHandle, USART_ER_NE);
		}

		if (temp1 & (0x1 << USART_SR_ORE))
		{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ER_ORE);
		}
	}
} /* End of USART_IRQHandling */

/**
 * USART_PeriClockControl()
 * Brief	: Enables or disables USART peripheral
 * Param	: @pUSARTx - base address of USARTx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: N/A
 */
void USART_PeriControl(USART_TypeDef *pUSARTx, uint8_t state)
{
	if (state == ENABLE)
	{
		pUSARTx->CR1 |= (0x1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(0x1 << USART_CR1_UE);
	}
} /* End of USART_PeriControl */

/**
 * USART_SetBaudRate()
 * Brief	: Sets the baurate for USART communication
 * Param	: @pUSARTx - base address of USARTx peripheral
 * 			  @baudrate - baudrate in bps
 * Retval	: None
 * Note		: N/A
 */
void USART_SetBaudRate(USART_TypeDef *pUSARTx, uint32_t baudrate)
{
	uint32_t PCLKx;		/* APB bus clock frequency */
	uint32_t usartdiv;
	uint32_t mantissa, fraction;
	uint32_t temp = 0;

	/* Get the APB bus clock frequency into 'PCLKx' */
	if (pUSARTx == USART1 || pUSARTx == USART6)
	{
		/* USART1 and USART6 are connected to APB2 bus */
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		/* USART2, USART3, UART4, UART5 are connected to APB1 bus */
		PCLKx = RCC_GetPCLK1Value();
	}

	/* Check the oversampling value (OVER8) */
	if (pUSARTx->CR1 & (0x1 << USART_CR1_OVER8))
	{
		/* OVER8=1, oversampling by 8 */
		usartdiv = ((25 * PCLKx) / (2 * baudrate));
	}
	else
	{
		/* OVER8=0, oversampling by 16 */
		usartdiv = ((25 * PCLKx) / (4 * baudrate));
	}

	/* Calculate the mantissa part */
	mantissa = usartdiv/100;

	/* Write the mantissa part in the appropriate bit position in 'temp' */
	temp |= (mantissa << USART_BRR_DIV_MANTISSA);

	/* Extract the fraction part */
	fraction = (usartdiv - (mantissa * 100));

	/* Calculate the final fraction */
	if (pUSARTx->CR1 & (0x1 << USART_CR1_OVER8))
	{
		/* OVER8=1, oversampling by 8 */
		fraction = (((fraction * 8) + 50) / 100) & ((uint8_t)0x07);
	}
	else
	{
		/* OVER8=0, oversampling by 16 */
		fraction = (((fraction * 16) + 50) / 100) & ((uint8_t)0x0F);
	}

	/* Write the fraction part in the appropriate bit position in 'temp' */
	temp |= fraction;

	/* Copy the contents of 'temp' into USART_BRR register */
	pUSARTx->BRR = temp;
} /* End of USART_SetBaudRate */

/**
 * USART_ApplicationEventCallback()
 * Brief	: Notifies the application of the event occurred
 * Param	: @pUSARTHandle - pointer to USART handle structure
 * 			  @appEvent - USART event occurred
 * Retval	: None
 * Note		: This function must be implemented by the application. Since the driver
 * 			  does not know in which application this function will be implemented,
 * 			  the driver defines it as a weak function. The application may override
 * 			  this function.
 * 			  If the application does not implement this function, the following
 * 			  definition will be executed.
 */
__WEAK void USART_ApplicationEventCallback(USART_Handle_TypeDef *pUSARTHandle, uint8_t appEvent)
{
	/* Implemented in the application */
} /* End of USART_ApplicationEventCallback */
