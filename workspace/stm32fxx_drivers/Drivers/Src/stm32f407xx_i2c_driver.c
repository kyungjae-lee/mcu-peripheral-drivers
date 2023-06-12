/*******************************************************************************
 * Filename		: stm32f407xx_i2c_driver.c
 * Description	: STM32F407xx MCU specific I2C driver source file
 * Author		: Kyungjae Lee
 * History		: Jun 09, 2023 - Created file
 ******************************************************************************/

#include "stm32f407xx.h"

/* RCC_CFGR HPRE[7:4] */
uint16_t AHB_Prescalar[8] = {2, 4, 8, 16, 64, 128, 256, 512};

/* RCC_CFGR PPRE1[10:8] */
uint16_t APB1_Prescalar[4] = {2, 4, 8, 16};

/* Declaration of I2C peripheral driver private functions */
static void I2C_GenerateSTARTCondition(I2C_TypeDef *pI2Cx);
static void I2C_GenerateSTOPCondition(I2C_TypeDef *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_TypeDef *pI2Cx, uint8_t slaveAddr);
static void I2C_ClearADDRFlag(I2C_TypeDef *pI2Cx);

/*******************************************************************************
 * APIs supported by the I2C driver
 * (See function definitions for more information)
 ******************************************************************************/

/**
 * I2C_PeriClockControl()
 * Desc.	:
 * Param.	: @pI2Cx - base address of I2Cx peripheral
 * Return	:
 * Note		:
 */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t state)
{
}

/**
 * RCC_GetPLLOutputClk()
 * Desc.	:
 * Param.	: None
 * Return	: None
 * Note		: N/A
 */
uint32_t RCC_GetPLLOutputClk(void)
{
	return 0;
}


/**
 * RCC_GetPCLK1Value()
 * Desc.	:
 * Param.	:
 * Return	:
 * Note		: Similarly 'RCC_GetPCLKxValue()' (x = 1,2,...) can be defined if
 * 			  there are other I2C peripherals connected to other buses.
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pClk1;		/* Peripheral clock frequency */
	uint32_t sysClk;		/* System clock frequency */
	uint8_t clkSrc;		/* Clock source */
	uint8_t temp;
	uint16_t ahbp;		/* AHB prescalar */
	uint16_t apb1p;		/* APB1 prescalar */

	clkSrc = (RCC->CFGR >> RCC_CFGR_SWS) & 0x03;

	if (clkSrc == 0)
	{
		/* 00: HSI oscillator used as the system clock */
		sysClk = 16000000;	/* System clock = 16 MHz */
	}
	else if (clkSrc == 1)
	{
		/* 01: HSE oscillator used as the system clock */
		sysClk = 8000000;	/* In the case of STM32 Discovery board */
	}
	else if (clkSrc == 2)
	{
		/* 10: PLL used as the system clock */
		sysClk = RCC_GetPLLOutputClk();
	}

	/* Obtain AHB prescalar */
	temp = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xF);

	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_Prescalar[temp - 8];
	}

	/* Obtain APB1 prescalar */
	temp = ((RCC->CFGR >> RCC_CFGR_PPRE1) & 0x7);

	if (temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_Prescalar[temp - 4];
	}

	/* Compute I2C peripheral clock frequency */
	pClk1 = (sysClk / ahbp) / apb1p;

	return pClk1;
}

/**
 * I2C_Init()
 * Desc.	:
 * Param.	:
 * Return	:
 * Note		:
 */
void I2C_Init(I2C_Handle_TypeDef *pI2CHandle)
{
	uint32_t temp = 0;

	/* Configure I2C_CR1 (Enable ACK) */
	temp |= pI2CHandle->I2C_Config.I2C_ACKEnable << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = temp;

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

	pI2CHandle->pI2Cx->TRISE |= (temp & 0x3F);
}

/**
 * I2C_DeInit()
 * Desc.	:
 * Param.	:
 * Return	:
 * Note		:
 */
void I2C_DeInit(I2C_TypeDef *pI2Cx)
{
}

/**
 * I2C_MasterTx()
 * Desc.	:
 * Param.	: @pI2CHandle - pointer to I2C peripheral handle
 * 			  @pTxBuffer - address of the Tx buffer
 * 			  @len - length of the data to transmit
 * 			  @slaveAddr - slave address
 * Return	: None
 * Note		: N/A
 */
void I2C_MasterTx(I2C_Handle_TypeDef *pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t slaveAddr)
{
	/* 1. Generate the START condition */
	I2C_GenerateSTARTCondition(pI2CHandle->pI2Cx);	/* Helper function private to driver */

	/* 2. Confirm that START generation is completed by checking the SB flag in
	 * 	  the SR1
	 * Note: Until SB (Start Bit) is cleared, SCL will be stretched
	 * 		 (pulled to LOW)
	 * 		 cording to the reference manual, SB bit can be cleared by
	 * 		 reading the SR1 register followed by writing the DR register,
	 * 		 or by hardware when PE = 0.
	 */
	while ( !(pI2CHandle->pI2Cx->SR1 & (0x01 << I2C_SR1_SB)) );
		/* Reading SR1 register is done here. Writing the DR register will
		 * be done in Step 3.
		 */

	/* 3. Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	 */
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, slaveAddr);

	/* 4. Confirm that address phase is completed by checking the ADDR flag
	 *	  of SR1
	 */
	while ( !(pI2CHandle->pI2Cx->SR1 & (0x01 << I2C_SR1_ADDR)) );

	/* 5. Clear the ADDR flag according to its software sequence (ADDR bit is
	 * 	  cleared by software reading SR1 register followed by reading SR2, or
	 * 	  by hardware when PE = 0.)
	 * Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	 */
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	/* 6. Send the data until @len becomes 0 */
	while (len > 0)
	{
		/* Wait until the TxE is set */
		while ( !(pI2CHandle->pI2Cx->SR1 & (0x01 << I2C_SR1_TxE)) );

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
	while ( !(pI2CHandle->pI2Cx->SR1 & (0x01 << I2C_SR1_TxE)) );

	while ( !(pI2CHandle->pI2Cx->SR1 & (0x01 << I2C_SR1_BTF)) );


	/* 8. Generate STOP condition and master does not need to wait for the
	 * 	  completion of stop condition.
	 * Note: generating STOP, automatically clears the BTF.
	 */
	I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
}

/**
 * I2C_IRQInterruptConfig()
 * Desc.	:
 * Param.	:
 * Return	:
 * Note		:
 */
void I2C_IRQInterruptConfig(uint8_t irqNumber, uint8_t state)
{
}

/**
 * I2C_IRQPriorityConfig()
 * Desc.	:
 * Param.	:
 * Return	:
 * Note		:
 */
void I2C_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
}

/**
 * I2C_PeriControl()
 * Desc.	:
 * Param.	:
 * Return	:
 * Note		:
 */
void I2C_PeriControl(I2C_TypeDef *pI2Cx, uint8_t state)
{
}

/**
 * Application callback functions (Must be implemented by application)
 * Note: Since the driver does not know in which application this function will
 * 		 be implemented, it is good idea to give a weak function definition.
 */
/**
 * I2C_ApplicationEventCallback()
 * Desc.	:
 * Param.	:
 * Return	:
 * Note		:
 */
__WEAK void I2C_ApplicationEventCallback(I2C_Handle_TypeDef *pI2CHandle, uint8_t appEvent)
{
}


/*******************************************************************************
 * I2C driver private functions
 ******************************************************************************/

/**
 * I2C_GenerateSTARTCondition()
 * Desc.	:
 * Param.	:
 * Return	:
 * Note		:
 */
static void I2C_GenerateSTARTCondition(I2C_TypeDef *pI2Cx)
{
	pI2Cx->CR1 |= (0x1 << I2C_CR1_START);
}

/**
 * I2C_GenerateSTOPCondition()
 * Desc.	:
 * Param.	:
 * Return	:
 * Note		:
 */
static void I2C_GenerateSTOPCondition(I2C_TypeDef *pI2Cx)
{
	pI2Cx->CR1 |= (0x1 << I2C_CR1_STOP);
}

/**
 * I2C_ExecuteAddressPhase()
 * Desc.	:
 * Param.	:
 * Return	:
 * Note		:
 */
static void I2C_ExecuteAddressPhase(I2C_TypeDef *pI2Cx, uint8_t slaveAddr)
{
	/* Shift 7-bit slave address to left 1 bit position to make room for
	 * r/w bit. Slave address (1 byte) = 7-bit slave address + r/w bit.
	 */
	slaveAddr <<= 1;
	slaveAddr &= ~(1);	/* Clear bit[0] */
	pI2Cx->DR = slaveAddr;
}

/**
 * I2C_ClearADDRFlag()
 * Desc.	: Clears the ADDR bit of I2Cx SR1 register
 * Param.	: @pI2Cx - base address of I2Cx peripheral
 * Return	: None
 * Note		: ADDR bit is cleared by software reading SR1 register followed by
 * 			  reading SR2, or by hardware when PE = 0.
 * 			  This function is a private helper function.
 */
static void I2C_ClearADDRFlag(I2C_TypeDef *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR1;
	(void)dummyRead;	/* To suppress 'unused variable' warning */
}
