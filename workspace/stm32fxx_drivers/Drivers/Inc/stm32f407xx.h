/*
 * @ Filename		: stm32f407xx.h
 * @ Description	: Device header file for stm32f407xx MCU
 * @ Author			: Kyungjae Lee
 * @ Date created	: 05/18/2023
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __IO			volatile		/* I/O registers are highly volatile in nature */

/* Base addresses of memories */
#define FLASH_BASE		0x08000000U		/* Flash base address in the alias 				*/
#define SRAM1_BASE		0x20000000U		/* SRAM1(112 KB) base address in the alias 		*/
#define SRAM2_BASE		0x20001C00U		/* SRAM2(16 KB) base address in the alias 		*/
#define SRAM_BASE		SRAM1_BASE		/* SRAM	base address in the alias				*/
#define ROM_BASE 		0x1FFF0000U		/* ROM(system memory) base address in the alias	*/

/* Base addresses of APBx and AHB bus peripherals */
#define PERIPH_BASE		0x40000000U		/* Peripheral base address in the alias 		*/
#define APB1PERIPH_BASE	PERIPH_BASE
#define APB2PERIPH_BASE	(PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE	(PERIPH_BASE + 0x00020000U)
#define AHB2PERIPH_BASE	(PERIPH_BASE + 0x10000000U)

/* Base addresses of AHB1 peripherals */
#define GPIOA_BASE		(AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASE		(AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASE		(AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASE		(AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASE		(AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASE		(AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASE		(AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASE		(AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASE		(AHB1PERIPH_BASE + 0x2000U)

/* Base addresses of APB1 peripherals */
#define SPI2_BASE		(APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASE		(APB1PERIPH_BASE + 0x3C00U)
#define USART2_BASE		(APB1PERIPH_BASE + 0x4400U)
#define USART3_BASE		(APB1PERIPH_BASE + 0x4800U)
#define UART4_BASE		(APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASE		(APB1PERIPH_BASE + 0x5000U)
#define I2C1_BASE		(APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASE		(APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASE		(APB1PERIPH_BASE + 0x5C00U)

/* Base addresses of APB2 peripherals */
#define USART1_BASE		(APB2PERIPH_BASE + 0x1000U)
#define USART6_BASE		(APB2PERIPH_BASE + 0x1400U)
#define SPI1_BASE		(APB2PERIPH_BASE + 0x3000U)
#define SYSCFG_BASE		(APB2PERIPH_BASE + 0x3800U)
#define EXTI_BASE		(APB2PERIPH_BASE + 0x3C00U)


/*
 * Peripheral registers structures
 *
 * Note: Number of registers of each peripheral may defer from MCU family to MCU family.
 * 		 Check the reference manual!
 */

/* General Purpose I/O */
typedef struct
{
	__IO uint32_t MODER;	/* GPIO port mode register,					Address offset: 0x00 		*/
	__IO uint32_t OTYPER;	/* GPIO port output type register,			Address offset: 0x04 		*/
	__IO uint32_t OSPEEDR;	/* GPIO port output speed register,			Address offset: 0x08 		*/
	__IO uint32_t PUPDR;	/* GPIO port pull-up/pull-down register,	Address offset: 0x0C 		*/
	__IO uint32_t IDR;		/* GPIO port input data register,			Address offset: 0x10 		*/
	__IO uint32_t ODR;		/* GPIO port output data register,			Address offset: 0x14 		*/
	__IO uint32_t BSRR;		/* GPIO port bit set/reset register,		Address offset: 0x18 		*/
	__IO uint32_t LCKR;		/* GPIO port configuration lack register,	Address offset: 0x1C 		*/
	__IO uint32_t AFR[2];	/* GPIO port alternate function register,	Address offset: 0x20-0x24 	*/
} GPIO_TypeDef;

/* Reset and Clock Control */
typedef struct
{

} RCC_TypeDef;


/*
 * Peripheral declarations (Peripheral base addresses typecasted to (x_TypeDef *))
 */

/*
 * GPIOs
 */
#define GPIOA			((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB			((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC			((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD			((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE			((GPIO_TypeDef *)GPIOE_BASE)
#define GPIOF			((GPIO_TypeDef *)GPIOF_BASE)
#define GPIOG			((GPIO_TypeDef *)GPIOG_BASE)
#define GPIOH			((GPIO_TypeDef *)GPIOH_BASE)
#define GPIOI			((GPIO_TypeDef *)GPIOI_BASE)

#endif /* INC_STM32F407XX_H_ */
