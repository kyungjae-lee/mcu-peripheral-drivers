/*******************************************************************************
 * File		: stm32f407xx_i2c_driver.c
 * Brief	: STM32F407xx MCU specific I2C driver source file
 * Author	; Kyungjae Lee
 * Date		: Jun 09, 2023
 * ****************************************************************************/

#include "stm32f407xx.h"

/* Declaration of I2C peripheral driver private functions */
static void I2C_ExecuteAddressPhaseRead(I2C_TypeDef *pI2Cx, uint8_t slaveAddr);
static void I2C_ExecuteAddressPhaseWrite(I2C_TypeDef *pI2Cx, uint8_t slaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_TypeDef *pI2CHandle);


/*******************************************************************************
 * APIs supported by the I2C driver
 * (See function definitions for more information)
 ******************************************************************************/

/**
 * I2C_PeriControl()
 * Brief	: Enables or disables I2C peripheral
 * Param	: @pI2Cx - base address of I2Cx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: N/A
 */
void I2C_PeriControl(I2C_TypeDef *pI2Cx, uint8_t state)
{
	if (state == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
} /* End of I2C_PeriControl */

/**
 * I2C_PeriClockControl()
 * Brief	: Enable or disable clock for I2C peripheral
 * Param	: @pI2Cx - base address of I2Cx peripheral
 * Retval	: None
 * Note		: N/A
 */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t state)
{
	if (state == ENABLE)
	{
		if (pI2Cx == I2C1)
			I2C1_PCLK_EN();
		else if (pI2Cx == I2C2)
			I2C2_PCLK_EN();
		else if (pI2Cx == I2C3)
			I2C3_PCLK_EN();
	}
	else
	{
		if (pI2Cx == I2C1)
			I2C1_PCLK_DI();
		else if (pI2Cx == I2C2)
			I2C2_PCLK_DI();
		else if (pI2Cx == I2C3)
			I2C3_PCLK_DI();
	}
} /* End of I2C_PeriClockControl */

/**
 * I2C_Init()
 * Brief	: Initializes passed I2C peripheral
 * Param	: @pI2CHandle - pointer to I2C peripheral handle
 * Retval	: None
 * Note		: N/A
 */
void I2C_Init(I2C_Handle_TypeDef *pI2CHandle)
{
	uint32_t temp = 0;

	/* Enable I2C1 peripheral clock */
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	/* Configure I2C_CR1 (Enable ACK) */
	//temp |= pI2CHandle->I2C_Config.I2C_ACKEnable << I2C_CR1_ACK;
	//pI2CHandle->pI2Cx->CR1 = temp;
	// Commented out since ACK bit cannot be set when PE=0. This will be done
	// in the application.
	// (Consider moving this logic into 'I2C_PeriControl()' function. )

	/* Configure I2C_CR2 (FREQ[5:0])
	 * The FREQ bits must be configured with the APB clock frequency value (I2C
	 * peripheral connected to APB). The FREQ field is used by the peripheral
	 * to generate data setup and hold times compliant with the I2C
	 * specifications. The minimum allowed frequency is 2 MHz, the maximum
	 * frequency is limited by the maximum APB frequency and cannot exceed
	 * 50 MHz (peripheral intrinsic maximum limit).
	 *
	 * 0b000000: Not allowed
	 * 0b000001: Not allowed
	 * 0b000010: 2 MHz
	 * ...
	 * 0b110010: 50 MHz
	 * Higher than 0b101010: Not allowed
	 */
	temp = 0;
	temp |= RCC_GetPCLK1Value() / 1000000;	/* e.g., 16 MHz -> 16 */
	pI2CHandle->pI2Cx->CR2 |= (temp & 0x3F); 	/* Masking for safety purpose */

	/* Configure OAR1 (Device own address) */
	temp = 0;
	temp |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD71;
		/* bit[0] of OAR1 is used only when the own address is of 10-bit long
		 * instead of 7-bit. big[15] of OAR1 determines the length of the
		 * address (7 or 10). 7-bit length by default.
		 */
	temp |= (0x1 << 14);
		/* bit[14] should always be kept at 1 by software (Reference manual) */
	pI2CHandle->pI2Cx->OAR1 |= temp;

	/* Configure CCR (Calaulate CCR) */
	uint16_t ccrVal = 0;
	temp = 0;

	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/* Standard mode (I2C_CCR bit[15] == 0) */
		ccrVal = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		temp |= (ccrVal & 0xFFF); /* 12-bit value */
	}
	else
	{
		/* Fast mode (I2C_CCR bit[15] == 1) */
		temp |= (0x1 << I2C_CCR_FS);
		temp |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccrVal = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			ccrVal = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}

		temp |= (ccrVal & 0xFFF); /* 12-bit value */
	}

	pI2CHandle->pI2Cx->CCR |= temp;

	/* TRISE configuration */
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/* Standard mode */
		temp = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		/* Fast mode */
		temp = ((RCC_GetPCLK1Value() * 300U) / 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (temp & 0x3F);
} /* End of I2C_Init */

/**
 * I2C_DeInit()
 * Brief	: Deinitializes passed I2C peripheral
 * Param	: @pI2Cx - base address of I2Cx peripheral
 * Retval	: None
 * Note		: N/A
 */
void I2C_DeInit(I2C_TypeDef *pI2Cx)
{
	/* Do nothing */
} /* End of I2C_DeInit */

/**
 * I2C_MasterTxBlocking()
 * Brief	: Handles blocking-based I2C transmission
 * Param	: @pI2CHandle - pointer to I2C peripheral handle
 * 			  @pTxBuffer - address of the Tx buffer
 * 			  @len - length of the data to transmit
 * 			  @slaveAddr - slave address
 * 			  @repeatedStartState - I2C_REPEATED_START_EN or DI
 * Retval	: None
 * Note		: N/A
 */
void I2C_MasterTxBlocking(I2C_Handle_TypeDef *pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t repeatedStartState)
{
	/* 1. Generate the START condition */
	I2C_GenerateSTARTCondition(pI2CHandle->pI2Cx);	/* Helper function private to driver */

	/* 2. Confirm that START generation is completed by checking the SB flag in
	 * 	  the SR1
	 * Note: Until SB (Start Bit) gets cleared, SCL will be stretched
	 * 		 (pulled to LOW)
	 * 		 According to the reference manual, SB bit can be cleared by
	 * 		 reading the SR1 register followed by writing the DR register,
	 * 		 or by hardware when PE = 0.
	 */
	while ( !(pI2CHandle->pI2Cx->SR1 & (0x01 << I2C_SR1_SB)) );
		/* Reading SR1 register is done here. Writing the DR register will
		 * be done in Step 3.
		 */

	/* 3. Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	 */
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, slaveAddr);

	/* 4. Wait until the address phase is completed by checking the ADDR flag
	 *	  of SR1
	 */
	while ( !(pI2CHandle->pI2Cx->SR1 & (0x01 << I2C_SR1_ADDR)) );

	/* 5. Clear the ADDR flag according to its software sequence (ADDR bit is
	 * 	  cleared by software reading SR1 register followed by reading SR2, or
	 * 	  by hardware when PE = 0.)
	 * Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	 */
	I2C_ClearADDRFlag(pI2CHandle);

	/* 6. Send the data until @len becomes 0 */
	while (len > 0)
	{
		/* Wait until the TxE is set */
		while ( !(pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_TxE)) );

		/* Copy @pTxBuffer contents to DR */
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	/* 7. When @len becomes zero wait for TXE=1 and BTF=1 before generating the
	 * 	  STOP condition.
	 * Note: TXE=1, BTF=1, means that both SR and DR are empty and next
	 * 		 transmission should begin when BTF=1 SCL will be stretched
	 * 		 (pulled to LOW)
	 */
	while ( !(pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_TxE)) );

	while ( !(pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_BTF)) );


	/* 8. Generate STOP condition and master does not need to wait for the
	 * 	  completion of stop condition.
	 * Note: generating STOP, automatically clears the BTF.
	 */
	if (repeatedStartState == I2C_REPEATED_START_DI)
	{
		I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
	}
} /* End of I2C_MasterTxBlocking */

/**
 * I2C_MasterRxBlocking()
 * Brief	: Handles blocking-based I2C reception
 * Param	: @pI2CHandle - pointer to I2C peripheral handle
 * 			  @pTxBuffer - address of the Tx buffer
 * 			  @len - length of the data to transmit
 * 			  @slaveAddr - slave address
 * 			  @repeatedStartState - I2C_REPEATED_START_EN or DI
 * Retval	: None
 * Note		: N/A
 */
void I2C_MasterRxBlocking(I2C_Handle_TypeDef *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t repeatedStartState)
{
	/* 1. Generate the START condition */
	I2C_GenerateSTARTCondition(pI2CHandle->pI2Cx);	/* Helper function private to driver */

	/* 2. Confirm that START generation is completed by checking the SB flag in
	 * 	  the SR1
	 * Note: Until SB (Start Bit) gets cleared, SCL will be stretched
	 * 		 (pulled to LOW)
	 * 		 According to the reference manual, SB bit can be cleared by
	 * 		 reading the SR1 register followed by writing the DR register,
	 * 		 or by hardware when PE = 0.
	 */
	while ( !(pI2CHandle->pI2Cx->SR1 & (0x01 << I2C_SR1_SB)) );
		/* Reading SR1 register is done here. Writing the DR register will
		 * be done in Step 3.
		 */
	/* 3. Send the address of the slave with r/w bit set to R(1) (total 8 bits)
	 */
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, slaveAddr);

	/* 4. Wait until the address phase is completed by checking the ADDR flag
	 *	  of SR1
	 */
	while ( !(pI2CHandle->pI2Cx->SR1 & (0x01 << I2C_SR1_ADDR)) );

	/**
	 * 5. Read data from the slave
	 */

	/* 5.1. Read the last byte */
	if (len == 1)
	{
		/* Disable ACK */
		I2C_ManageACK(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		/* Clear the ADDR flag */
		I2C_ClearADDRFlag(pI2CHandle);

		/* Wait until the RxNE flag is set */
		while ( !(pI2CHandle->pI2Cx->SR1 & (0x01 << I2C_SR1_RxNE)) );

		/* Generate STOP condition */
		if (repeatedStartState == I2C_REPEATED_START_DI)
		{
			I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
		}

		/* Read data into the Rx buffer */
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	/* 5.2. Read non-last byte */
	if (len > 1)
	{
		/* Clear the ADDR flag */
		I2C_ClearADDRFlag(pI2CHandle);
			/* Now, the data reception begins! */

		/* Read the data until @len becomes zero */
		for (uint32_t i = len; i > 0; i--)
		{
			/* Wait until RxNE flag is set */
			while ( !(pI2CHandle->pI2Cx->SR1 & (0x01 << I2C_SR1_RxNE)) );

			/* Read the second to last byte */
			if (i == 2)
			{
				/* Disable ACK */
				I2C_ManageACK(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				/* Generate STOP condition */
				if (repeatedStartState == I2C_REPEATED_START_DI)
				{
					I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
				}
			}

			/* Read the data from the DR into Rx buffer */
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			/* Increment the buffer address */
			pRxBuffer++;
		}

	}

	/* Enable ACK (Before entering this function, ACK was enabled.
	 * Make sure to re-enable ACK to restore the previous condition.) */
	if (pI2CHandle->I2C_Config.I2C_ACKEnable == I2C_ACK_ENABLE)
		I2C_ManageACK(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
} /* End of I2C_MasterRxBlocking */

/**
 * I2C_MasterTxInterrupt()
 * Brief	: Handles interrupt-based I2C transmission
 * Param	: @pI2CHandle - pointer to I2C peripheral handle
 * 			  @pTxBuffer - address of the Tx buffer
 * 			  @len - length of the data to transmit
 * 			  @slaveAddr - slave address
 * 			  @repeatedStartState - I2C_REPEATED_START_EN or DI
 * Retval	: None
 * Note		: N/A
 */
uint8_t I2C_MasterTxInterrupt(I2C_Handle_TypeDef *pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t repeatedStartState)
{
	uint8_t busyState = pI2CHandle->TxRxState;

	if ((busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState= I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->RepeatedStart = repeatedStartState;

		/* Generate START condition */
		I2C_GenerateSTARTCondition(pI2CHandle->pI2Cx);
			/* If successful, SB bit will bet set here */

		/**
		 * Enable all available interrupts by setting the control bits
		 */
		/* Enable ITBUFEN control bit */
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITBUFEN);
		/* Enable ITEVTEN control bit */
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITEVTEN);
		/* Enable ITERREN control bit */
		pI2CHandle->pI2Cx->CR2 |= (0x1 << I2C_CR2_ITERREN);
	}

	return busyState;
} /* End of I2C_MasterTxInterrupt */

/**
 * I2C_MasterRxInterrupt()
 * Brief	: Handles interrupt-based I2C reception
 * Param	: @pI2CHandle - pointer to I2C peripheral handle
 * 			  @pTxBuffer - address of the Tx buffer
 * 			  @len - length of the data to transmit
 * 			  @slaveAddr - slave address
 * 			  @repeatedStartState - I2C_REPEATED_START_EN or DI
 * Retval	: None
 * Note		: N/A
 */
uint8_t I2C_MasterRxInterrupt(I2C_Handle_TypeDef *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr, uint8_t repeatedStartState)
{
	uint8_t busyState = pI2CHandle->TxRxState;

	if ((busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = len;
		pI2CHandle->TxRxState= I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = len;
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->RepeatedStart = repeatedStartState;

		/* Generate START condition */
		I2C_GenerateSTARTCondition(pI2CHandle->pI2Cx);
			/* If successful, SB bit will bet set here */

		/**
		 * Enable all available interrupts by setting the control bits
		 */
		/* Enable ITBUFEN control bit */
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		/* Enable ITEVTEN control bit */
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		/* Enable ITERREN control bit */
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}

	return busyState;
} /* End of I2C_MasterRxInterrupt */

/**
 * I2C_SlaveTx()
 * Brief	: Handles transmission of a byte of data upon master's read event
 * Param	: @pI2Cx - base address of I2Cx peripheral
 * 		      @data	- a byte of data to send
 * Retval	: None
 * Note		: N/A
 */
void I2C_SlaveTx(I2C_TypeDef *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
} /* End of I2C_SlaveTx */

/**
 * I2C_SlaveRx()
 * Brief	: Handles reception of a byte of data upon master's write event
 * Param	: @pI2Cx - base address of I2Cx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: Received byte of data
 * Note		: N/A
 */
uint8_t I2C_SlaveRx(I2C_TypeDef *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
} /* End of I2C_SlaveRx */

/**
 * I2C_ManageACK()
 * Brief	: Enables or disables @I2Cx peripheral's ACKing
 * Param	: @pI2Cx - base address of I2Cx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: N/A
 */
void I2C_ManageACK(I2C_TypeDef *pI2Cx, uint8_t state)
{
	if (state == I2C_ACK_ENABLE)
	{
		/* Enable ACK */
		pI2Cx->CR1 |= (0x1 << I2C_CR1_ACK);
	}
	else
	{
		/* Disable ACK */
		pI2Cx->CR1 &= ~(0x1 << I2C_CR1_ACK);
	}
} /* End of I2C_ManageACK */

/**
 * I2C_IRQInterruptConfig()
 * Brief	: Enables or disables I2C IRQ interrupts
 * Param	: @irqNumber - IRQ number
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: N/A
 */
void I2C_IRQInterruptConfig(uint8_t irqNumber, uint8_t state)
{
	if (state == ENABLE)
	{
		/* Configure NVIC_ISERx register */
		if (irqNumber <= 31)
			*NVIC_ISER0 |= (0x1 << irqNumber);
		else if (32 <= irqNumber && irqNumber <= 64)
			*NVIC_ISER1 |= (0x1 << irqNumber % 32);
		else if (65 <= irqNumber && irqNumber <= 96)
			*NVIC_ISER2 |= (0x1 << irqNumber % 32);
	}
	else
	{
		/* Configure NVIC_ICERx register */
		if (irqNumber <= 31)
			*NVIC_ICER0 |= (0x1 << irqNumber);
		else if (32 <= irqNumber && irqNumber <= 64)
			*NVIC_ICER1 |= (0x1 << irqNumber % 32);
		else if (65 <= irqNumber && irqNumber <= 96)
			*NVIC_ICER2 |= (0x1 << irqNumber % 32);
	}
} /* End of I2C_IRQInterruptConfig */

/**
 * I2C_IRQPriorityConfig()
 * Brief	: Configures I2C IRQ interrupt priorities
 * Param	: @irqNumber - IRQ number
 * 			  @irqPriotity - IRQ priority (Make sure this parameter is of
 * 			  				 type uint32_t. Due to the number of bits it
 * 			  				 needs to be shifted during the calculation,
 * 							 declaring it as uint8_t did not do its job.
 * Retval	: None
 * Note		: N/A
 */
void I2C_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
	/* Find out the IPR register */
	uint8_t iprNumber = irqNumber / 4;
	uint8_t iprSection = irqNumber % 4;
	uint8_t bitOffset = (iprSection * 8) + (8 - NUM_PRI_BITS_USED);
	*(NVIC_IPR_BASE + iprNumber) |= (irqPriority << bitOffset);
} /* End of I2C_IRQPriorityConfig */

/**
 * I2C_EV_IRQHandling()
 * Brief	: Handles I2C event IRQ (common for both master and slave modes)
 * Param	: @pI2CHandle - pointer to I2C peripheral handle
 * Retval	: None
 * Note		: This function will first decode the event that occurred, and
 * 			  handle the event accordingly.
 */
void I2C_EV_IRQHandling(I2C_Handle_TypeDef *pI2CHandle)
{
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (0x1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (0x1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_SB);

	/* 1. Handle the interrupt generated by SB event
	 * Note: SB event is available only in master mode. Therefore, this block
	 * 		 will note be executed in slave mode. (For slave, SB bit is always
	 * 		 zero. Meaning that in I2C, slaves cannot start communication.)
	 */
	if (temp1 && temp3)
	{
		/* SB flag is set */

		/* Execute the address phase */
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_ADDR);
	/* 2. Handle the interrupt generated by ADDR event
	 * Note: When in master mode - Address is sent
	 * 		 When in slave mode - Address matched with own address
	 */
	if (temp1 && temp3)
	{
		/* ADDR flag is set
		 *
		 * When ADDR flag is set, clock will be stretched and both the master
		 * and slave will be in wait state. So it is important to clear it
		 * according to the logic.
		 */
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_BTF);
	/* 3. Handle the interrupt generated by BTF event */
	if (temp1 && temp3)
	{
		/* BTF (Byte Transfer Finished) flag is set
		 *
		 * When BTF flag is set:
		 *
		 * 1. If TxE=1 during transmission
		 * 	  - Both SR and DR are empty
		 * 	  - Clock will be stretched
		 * 2. If RxNe=1 during reception
		 * 	  - Both SR and DR are full
		 * 	  - Clock will be stretched
		 */
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			/* If TxE=1 */
			if (pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_TxE))
			{
				/**
				 * BTF=1 & TxE=1; Close I2C data transmission
				 */

				/* Check if the Tx length is 0 */
				if (pI2CHandle->TxLen == 0)
				{
					/* 1. Generate STOP condition */
					if (pI2CHandle->RepeatedStart == I2C_REPEATED_START_DI)
					{
						I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
					}

					/* 2. Reset all the members of the handle structure */
					I2C_CloseTx(pI2CHandle);

					/* 3. Notify the application about transmission complete */
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			/* Do nothing */
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_STOPF);
	/* 4. Handle the interrupt generated by STOPF event
	 * Note: STOP event detection (STOPF) is available only in slave mode.
	 * 		 (In master mode, STOPF flag will not be set.)
	 * 		 STOPF flag is set by hardware when a STOP condition is detected
	 * 		 on the bus by the slave after an Acknowledgement (ACK=1).
	 * 		 STOPF flag is cleared by software reading the SR1 register
	 * 		 followed by a write in the CR1 register,
	 * 		 or by hardware when PE=0.
	 *
	 */
	if (temp1 && temp3)
	{
		/* STOPF flag is set */

		/* Clear STOPF (i.e., Read SR1, write to CR1)
		 * Note: Reading SR1 is already done in the previous line of code
		 */
		/* Write to CR1 */
		pI2CHandle->pI2Cx->CR1 |= 0x0000; /* Write without affecting CR1 */

		/* Notify the application of the STOP detection */
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_TxE);
	/* 5. Handle the interrupt generated by TxE event */
	if (temp1 && temp2 && temp3)
	{
		/* TxE flag is set, perform the data transmission
		 * Note: This is an indication that DR is empty, and software can
		 * 		 put data into DR in order to send the data byte to the
		 * 		 external world.
		 */

		/* Check for device mode */
		if (pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_MSL))
		{
			/* Device is master */

			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				if (pI2CHandle->TxLen > 0)
				{
					/* 1. Load the data into DR */
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

					/* 2. Decrement TxLen */
					pI2CHandle->TxLen--;

					/* 3. Increment the Tx buffer address */
					pI2CHandle->pTxBuffer++;
				}
			}
		}
		else
		{
			/* Device is slave */

			if (pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_TRA))
			{
				/* Slave in transmitter mode */

				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX);
			}

		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (0x1 << I2C_SR1_RxNE);
	/* 6. Handle the interrupt generated by RxNE event */
	if (temp1 && temp2 && temp3)
	{
		/* RxNE flag is set */

		/* Check for device mode */
		if (pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_MSL))
		{
			/* Device is master */

			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				/* Perform the data reception */

				if (pI2CHandle->RxSize == 1)
				{
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
					pI2CHandle->RxLen--;
				}

				if (pI2CHandle->RxSize > 1)
				{
					if (pI2CHandle->RxLen == 2)
					{
						/* Clear the ACK bit */
						I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);
					}

					/* Read DR */
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
					pI2CHandle->pRxBuffer++;
					pI2CHandle->RxLen--;
				}

				if (pI2CHandle->RxLen == 0)
				{
					/**
					 * No more data to receive. Close the I2C data reception
					 * and notify the application of the communication termination.
					 */

					/* 1. Generate STOP condition */
					if (pI2CHandle->RepeatedStart == I2C_REPEATED_START_DI)
					{
						I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
					}

					/* 2. Close I2C Rx */
					I2C_CloseRx(pI2CHandle);

					/* 3. Notify the application */
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
				}
			}
		}
		else
		{
			/* Device is slave */

			if (!(pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_TRA)))
			{
				/* Slave in transmitter mode */

				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX);
			}
		}
	}
} /* End of I2C_EV_IRQHandling */

/**
 * I2C_ER_IRQHandling()
 * Brief	: Handles I2C error IRQ (common for both master and slave modes)
 * Param	: @pI2CHandle - pointer to I2C peripheral handle
 * Retval	: None
 * Note		: N/A
 */
void I2C_ER_IRQHandling(I2C_Handle_TypeDef *pI2CHandle)
{
	uint32_t temp1, temp2;

	/* Check the status of SR2 ITERREN control bit */
	temp2 = (pI2CHandle->pI2Cx->CR2) & (0x1 << I2C_CR2_ITERREN);

	/* Check for bus error (BERR) *********************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & (0x1 << I2C_SR1_BERR);
	if (temp1 && temp2)
	{
		/**
		 * Handle bus error
		 */

		/* Clear the bus error flag */
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_BERR);

		/* Notify the application of this error */
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	/* Check for arbitration lost (ARLO) **************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & (0x1 << I2C_SR1_ARLO);
	if (temp1 && temp2)
	{
		/**
		 * Handle arbitration lost error
		 */

		/* Clear the arbitration lost error flag */
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_ARLO);

		/* Notify the application of this error */
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	/* Check for ACK failure (AF) *********************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & (0x1 << I2C_SR1_AF);
	if (temp1 && temp2)
	{
		/**
		 * Handle ACK failure error
		 */

		/* Clear the ACK failure error flag */
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_AF);

		/* Notify the application of this error */
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	/* Check for overrun/underrun (OVR) ***************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & (0x1 << I2C_SR1_OVR);
	if (temp1 && temp2)
	{
		/**
		 * Handle overrun/underrun error
		 */

		/* Clear the overrun/underrun error flag */
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_OVR);

		/* Notify the application of this error */
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	/* Check for timeout error (TIMEOUT) **************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & (0x1 << I2C_SR1_TIMEOUT);
	if (temp1 && temp2)
	{
		/**
		 * Handle timeout error
		 */

		/* Clear the timeout error flag */
		pI2CHandle->pI2Cx->SR1 &= ~(0x1 << I2C_SR1_TIMEOUT);

		/* Notify the application of this error */
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
} /* End of I2C_ER_IRQHandling */

/**
 * I2C_CloseTx()
 * Brief	: Handles closing I2C Tx operation
 * Param	: @pI2CHandle - pointer to I2C peripheral handle
 * Retval	: None
 * Note		: N/A
 */
void I2C_CloseTx(I2C_Handle_TypeDef *pI2CHandle)
{
	/* Disable ITBUFEN control bit */
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITBUFEN);

	/* Disable ITEVTEN control bit */
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITEVTEN);

	/* Reset I2C handle members */
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
} /* End of I2C_CloseTx */

/**
 * I2C_CloseRx()
 * Brief	: Handles closing I2C Rx operation
 * Param	: @pI2CHandle - pointer to I2C peripheral handle
 * Retval	: None
 * Note		: N/A
 */
void I2C_CloseRx(I2C_Handle_TypeDef *pI2CHandle)
{
	/* Disable ITBUFEN control bit */
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITBUFEN);

	/* Disable ITEVTEN control bit */
	pI2CHandle->pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITEVTEN);

	/* Reset I2C handle members */
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	/* Enable ACK */
	if (pI2CHandle->I2C_Config.I2C_ACKEnable == I2C_ACK_ENABLE)
	{
		I2C_ManageACK(pI2CHandle->pI2Cx, ENABLE);
	}
} /* End of I2C_CloseRx */

/**
 * I2C_GenerateSTARTCondition()
 * Brief	: Generates I2C START condition
 * Param	: @pI2Cx - base address of I2Cx peripheral
 * Retval	: None
 * Note		: N/A
 */
void I2C_GenerateSTARTCondition(I2C_TypeDef *pI2Cx)
{
	pI2Cx->CR1 |= (0x1 << I2C_CR1_START);
} /* End of I2C_GenerateSTARTCondition */

/**
 * I2C_GenerateSTOPCondition()
 * Brief	: Generates I2C START condition
 * Param	: @pI2Cx - base address of I2Cx peripheral
 * Retval	: None
 * Note		: N/A
 */
void I2C_GenerateSTOPCondition(I2C_TypeDef *pI2Cx)
{
	pI2Cx->CR1 |= (0x1 << I2C_CR1_STOP);
} /* End of I2C_GenerateSTOPCondition */

/**
 * I2C_SlaveEnableDisableCallbackEvents()
 * Brief	: Enables or disables @pI2Cx's (as a slave) callback events
 * Param	: @pI2Cx - base address of I2Cx peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: N/A
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_TypeDef *pI2Cx, uint8_t state)
{
	if (state == ENABLE)
	{
		/**
		 * Enable all available interrupts by setting the control bits
		 */
		/* Enable ITBUFEN control bit */
		pI2Cx->CR2 |= (0x1 << I2C_CR2_ITBUFEN);
		/* Enable ITEVTEN control bit */
		pI2Cx->CR2 |= (0x1 << I2C_CR2_ITEVTEN);
		/* Enable ITERREN control bit */
		pI2Cx->CR2 |= (0x1 << I2C_CR2_ITERREN);
	}
	else
	{
		/**
		 * Disable all available interrupts by setting the control bits
		 */
		/* Disable ITBUFEN control bit */
		pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITBUFEN);
		/* Disable ITEVTEN control bit */
		pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITEVTEN);
		/* Disable ITERREN control bit */
		pI2Cx->CR2 &= ~(0x1 << I2C_CR2_ITERREN);
	}
} /* End of I2C_SlaveEnableDisableCallbackEvents */

/**
 * I2C_ApplicationEventCallback()
 * Brief	: Notifies the application of the event occurred
 * Param	: @pI2CHandle - pointer to I2C handle structure
 * 			  @appEvent - I2C event occurred
 * Retval	: None
 * Note		: This function must be implemented by the application. Since the driver
 * 			  does not know in which application this function will be implemented,
 * 			  the driver defines it as a weak function. The application may override
 * 			  this function.
 * 			  If the application does not implement this function, the following
 * 			  definition will be executed.
 */
__WEAK void I2C_ApplicationEventCallback(I2C_Handle_TypeDef *pI2CHandle, uint8_t appEvent)
{
	/* Implemented in the application */
} /* End of I2C_ApplicationEventCallback */


/*******************************************************************************
 * I2C driver private functions
 ******************************************************************************/

/**
 * I2C_ExecuteAddressPhaseRead()
 * Brief	: Executes address phase for I2C read operation
 * Param	: @pI2Cx - base address of I2Cx peripheral
 * 			  @slaveAddr - slave address
 * Retval	: None
 * Note		: N/A
 */
static void I2C_ExecuteAddressPhaseRead(I2C_TypeDef *pI2Cx, uint8_t slaveAddr)
{
	/* Shift 7-bit slave address to left 1 bit position to make room for
	 * r/w bit. Slave address (1 byte) = 7-bit slave address + r/w bit (1).
	 */
	slaveAddr <<= 1;
	slaveAddr |= 1;		/* Set bit[0] */
	pI2Cx->DR = slaveAddr;
} /* End of I2C_ExecuteAddressPhaseRead */

/**
 * I2C_ExecuteAddressPhaseWrite()
 * Brief	: Executes address phase for I2C write operation
 * Param	: @pI2Cx - base address of I2Cx peripheral
 * 			  @slaveAddr - slave address
 * Retval	: None
 * Note		: N/A
 */
static void I2C_ExecuteAddressPhaseWrite(I2C_TypeDef *pI2Cx, uint8_t slaveAddr)
{
	/* Shift 7-bit slave address to left 1 bit position to make room for
	 * r/w bit. Slave address (1 byte) = 7-bit slave address + r/w bit (0).
	 */
	slaveAddr <<= 1;
	slaveAddr &= ~(1);	/* Clear bit[0] */
	pI2Cx->DR = slaveAddr;
} /* End of I2C_ExecuteAddressPhaseWrite */

/**
 * I2C_ClearADDRFlag()
 * Brief	: Clears the ADDR bit of I2Cx SR1 register
 * Param	: @pI2CHandle - pointer to I2C peripheral handle
 * Retval	: None
 * Note		: ADDR bit is cleared by software reading SR1 register followed by
 * 			  reading SR2, or by hardware when PE = 0.
 * 			  This function is a private helper function.
 */
static void I2C_ClearADDRFlag(I2C_Handle_TypeDef *pI2CHandle)
{
	uint32_t dummyRead;

	/* Check for the device mode */
	if (pI2CHandle->pI2Cx->SR2 & (0x1 << I2C_SR2_MSL))
	{
		/**
		 * Device is in master mode
		 */

		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if (pI2CHandle->RxSize == 1)
			{
				/* Disable ACK first */
				I2C_ManageACK(pI2CHandle->pI2Cx, DISABLE);

				/* Clear ADDR flag (Read SR1, read SR2) */
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;	/* To suppress 'unused variable' warning */
			}
		}
		else
		{
			/* Clear ADDR flag (Read SR1, read SR2) */
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;	/* To suppress 'unused variable' warning */
		}
	}
	else
	{
		/**
		 * Device is in slave mode
		 */

		/* Clear ADDR flag (Read SR1, read SR2) */
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;	/* To suppress 'unused variable' warning */
	}
} /* End of I2C_ClearADDRFlag */
