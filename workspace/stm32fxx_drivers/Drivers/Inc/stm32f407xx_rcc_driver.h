/*******************************************************************************
 * Filename		: stm32f407xx_rcc_driver.h
 * Description	: STM32F407xx MCU specific RCC driver header file
 * Author		: Kyungjae Lee
 * History		: Jun 20, 2023 - Created file
 ******************************************************************************/

#ifndef STM32F407XX_RCC_DRIVER_H
#define STM32F407XX_RCC_DRIVER_H

#include "stm32f407xx.h"

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutputClk(void);


#endif /* STM32F407XX_RCC_DRIVER_H */
