/*******************************************************************************
 * File		: stm32f407xx_spi_driver.c
 * Brief	: STM32F407xx MCU specific RCC driver source file
 * Author	; Kyungjae Lee
 * Date		: Jun 20, 2023
 * ****************************************************************************/

#include "stm32f407xx.h"

/* RCC_CFGR HPRE[7:4] */
uint16_t AHB_Prescalar[8] = {2, 4, 8, 16, 64, 128, 256, 512};

/* RCC_CFGR PPRE1[10:8] */
uint16_t APB1_Prescalar[4] = {2, 4, 8, 16};

/**
 * RCC_GetPCLK1Value()
 * Brief	: Computes and returns APB1 peripheral clock frequency
 * Param	: None
 * Retval	: APB1 peripheral clock frequency
 * Note		: N/A
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1;		/* APB1 peripheral clock frequency */
	uint32_t sysClk;	/* System clock frequency */
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

	/* Compute APB1 peripheral clock frequency */
	pclk1 = (sysClk / ahbp) / apb1p;

	return pclk1;
} /* End of RCC_GetPCLK1Value */

/**
 * RCC_GetPCLK2Value()
 * Brief	: Computes and returns APB2 peripheral clock frequency
 * Param	: None
 * Retval	: APB2 peripheral clock frequency
 * Note		: N/A
 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2;		/* APB2 peripheral clock frequency */
	uint32_t sysClk;	/* System clock frequency */
	uint8_t clkSrc;		/* Clock source */
	uint8_t temp;
	uint16_t ahbp;		/* AHB prescalar */
	uint16_t apb2p;		/* APB2 prescalar */

	clkSrc = (RCC->CFGR >> RCC_CFGR_SWS) & 0x03;

	if (clkSrc == 0)
	{
		/* 00: HSI oscillator used as the system clock */
		sysClk = 16000000;	/* System clock = 16 MHz */
	}
	else
	{
		sysClk = 8000000;	/* In the case of STM32 Discovery board */
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

	/* Obtain APB2 prescalar */
	temp = ((RCC->CFGR >> RCC_CFGR_PPRE2) & 0x7);

	if (temp < 4)
	{
		apb2p = 1;
	}
	else
	{
		apb2p = APB1_Prescalar[temp - 4];
	}

	/* Compute APB2 peripheral clock frequency */
	pclk2 = (sysClk / ahbp) / apb2p;

	return pclk2;
} /* End of RCC_GetPCLK2Value */

/**
 * RCC_GetPLLOutputClk()
 * Brief	: Controls PLL ouput clock
 * Param	: None
 * Retval	: None
 * Note		: To be implemented as per the project requirements. Not used
 * 			  at this point.
 */
uint32_t RCC_GetPLLOutputClk(void)
{
	return 0;
} /* End of RCC_GetPLLOutputClk */
