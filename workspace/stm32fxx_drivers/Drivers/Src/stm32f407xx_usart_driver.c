/*******************************************************************************
 * Filename		: stm32f407xx_usart_driver.c
 * Description	: STM32F407xx MCU specific USART driver source file
 * Author		: Kyungjae Lee
 * History		: Jun 20, 2023 - Created file
 ******************************************************************************/

#include "stm32f407xx.h"


/*******************************************************************************
 * APIs supported by the I2C driver
 * (See function definitions for more information)
 ******************************************************************************/

/**
 * USART_PeriClockControl()
 * Desc.	: Enables or disables USART peripheral clock
 * Param.	: @pUSARTx - base address of USARTx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Return	: None
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
 * Desc.	: Initializes USART peripheral
 * Param.	: @pUSARTHandle - pointer to USART handle structure
 * Return	: None
 * Note		: N/A
 */
void USART_Init(USART_Handle_TypeDef *pUSARTHandle)
{
	/* Temporary variable */
	uint32_t temp = 0;

	/* Configure USART_CR1 ****************************************************/

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

	/* Configure USART_CR2 ****************************************************/

	temp = 0;

	/* Configure the number of stop bits */
	temp |= pUSARTHandle->USART_Config.USART_NumOfStopBits << USART_CR2_STOP;

	/* Write to USART_CR2 register */
	pUSARTHandle->pUSARTx->CR2 = temp;

	/* Configure USART_CR3 ****************************************************/

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

	/* Configure USART_BRR (Baud rate register) *******************************/

	/* Configure baudrate */
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

} /* End of USART_Init() */

/**
 * USART_DeInit()
 * Desc.	: Deinitializes USART peripheral
 * Param.	: @pUSARTx - base address of USARTx peripheral
 * Return	: None
 * Note		: N/A
 */
void USART_DeInit(USART_TypeDef *pUSARTx)
{

} /* End of USART_DeInit */

/**
 * USART_TxBlocking()
 * Desc.	: Handles blocking-based USART transmission
 * Param.	: @pUSARTHandle - pointer to USART handle structure
 * 			  @pTxBuffer - pointer to Tx buffer
 * 			  @len -
 * Return	: None
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
 * Desc.	: Handles blocking-based USART reception
 * Param.	: @pUSARTHandle - pointer to USART handle structure
 * 			  @pRxBuffer - pointer to Rx buffer
 * 			  @len -
 * Return	: None
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
 * Desc.	: Handles interrupt-based USART transmission
 * Param.	: @pUSARTHandle - pointer to USART handle structure
 * 			  @pTxBuffer - pointer to Tx buffer
 * 			  @len -
 * Return	: None
 * Note		: N/A
 */
uint8_t USART_TxInterrupt(USART_Handle_TypeDef *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	return 0;
} /* End of USART_TxInterrupt */

/**
 * USART_RxInterrupt()
 * Desc.	: Handles interrupt-based USART reception
 * Param.	: @pUSARTHandle - pointer to USART handle structure
 * 			  @pTxBuffer - pointer to Tx buffer
 * 			  @len -
 * Return	: None
 * Note		: N/A
 */
uint8_t USART_RxInterrupt(USART_Handle_TypeDef *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{
	return 0;
} /* End of USART_RxInterrupt */

/**
 * USART_IRQInterruptConfig()
 * Desc.	: Enables or disables USART interrupts
 * Param.	: @irqNumber - IRQ number
 * 			  @state - ENABLE or DISABLE macro
 * Return	: None
 * Note		: N/A
 */
void USART_IRQInterruptConfig(uint8_t irqNumber, uint8_t state)
{

} /* End of USART_IRQInterruptConfig */

/**
 * USART_IRQPriorityConfig()
 * Desc.	: Configures USART IRQ interrupt priorities
 * Param.	: @irqNumber - IRQ number
 * 			  @irqPriotity - IRQ priority (Make sure this parameter is of
 * 			  				 type uint32_t. Due to the number of bits it
 * 			  				 needs to be shifted during the calculation,
 * 							 declaring it as uint8_t did not do its job.
 * Return	: None
 * Note		: N/A
 */
void USART_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{

} /* End of USART_IRQPriorityConfig */

/**
 * USART_IRQHandling()
 * Desc.	: Handles USART event IRQ
 * Param.	: @pUSARTHandle - pointer to USART handle structure
 * Return	: None
 * Note		: This function will first decode the event that occurred, and
 * 			  handle the event accordingly.
 */
void USART_IRQHandling(USART_Handle_TypeDef *pUSARTHandle)
{

} /* End of USART_IRQHandling */

/**
 * USART_PeriClockControl()
 * Desc.	: Enables or disables USART peripheral
 * Param.	: @pUSARTx - base address of USARTx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Return	: None
 * Note		: N/A
 */
void USART_PeriControl(USART_TypeDef *pUSARTx, uint8_t state)
{

} /* End of USART_PeriControl */

/**
 * USART_SetBaudRate()
 * Desc.	: Sets the baurate for USART communication
 * Param.	: @pUSARTx - base address of USARTx peripheral
 * 			  @baudrate - baudrate in bps
 * Return	: None
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
