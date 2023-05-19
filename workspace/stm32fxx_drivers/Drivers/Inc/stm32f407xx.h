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
#define RCC_BASE		(AHB1PERIPH_BASE + 0x3800U)

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
	__IO uint32_t CR;			/* RCC clock control register,									Address offset: 0x00 */
	__IO uint32_t PLLCFGR;		/* RCC PLL configuration register,								Address offset: 0x04 */
	__IO uint32_t CFGR;			/* RCC clock configuration register,							Address offset: 0x08 */
	__IO uint32_t CIR;			/* RCC clock interrupt register,								Address offset: 0x0C */
	__IO uint32_t AHB1RSTR;		/* RCC AHB1 peripheral reset register,							Address offset: 0x10 */
	__IO uint32_t AHB2RSTR;		/* RCC AHB2 peripheral reset register,							Address offset: 0x14 */
	__IO uint32_t AHB3RSTR;		/* RCC AHB3 peripheral reset register,							Address offset: 0x18 */
	uint32_t 	 RESERVED0;		/* Reserved, Address offset: 0x1C 													 */
	__IO uint32_t APB1RSTR;		/* RCC APB1 peripheral reset register,							Address offset: 0x20 */
	__IO uint32_t APB2RSTR;		/* RCC APB2 peripheral reset register,							Address offset: 0x24 */
	uint32_t 	 RESERVED1[2];	/* Reserved, Address offset: 0x28-0x2C												 */
	__IO uint32_t AHB1ENR;		/* RCC AHB1 peripheral clock enable register,					Address offset: 0x30 */
	__IO uint32_t AHB2ENR;		/* RCC AHB2 peripheral clock enable register,					Address offset: 0x34 */
	__IO uint32_t AHB3ENR;		/* RCC AHB3 peripheral clock enable register,					Address offset: 0x38 */
	uint32_t 	 RESERVED2;		/* Reserved, Address offset: 0x3C													 */
	__IO uint32_t APB1ENR;		/* RCC APB1 peripheral clock enable register,					Address offset: 0x40 */
	__IO uint32_t APB2ENR;		/* RCC APB2 peripheral clock enable register,					Address offset: 0x44 */
	uint32_t 	 RESERVED3[2];	/* Reserved, Address offset: 0x48-0x4C 												 */
	__IO uint32_t AHB1LPENR;		/* RCC AHB1 peripheral clock enable in low power mode register,	Address offset: 0x50 */
	__IO uint32_t AHB2LPENR;		/* RCC AHB2 peripheral clock enable in low power mode register,	Address offset: 0x54 */
	__IO uint32_t AHB3LPENR;		/* RCC AHB3 peripheral clock enable in low power mode register,	Address offset: 0x58 */
	uint32_t 	 RESERVED4;		/* Reserved, Address offset: 0x04													 */
	__IO uint32_t APB1LPENR;		/* RCC APB1 peripheral clock enable in low power mode register,	Address offset: 0x60 */
	__IO uint32_t APB2LPENR;		/* RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
	uint32_t 	 RESERVED5[2];	/* Reserved, Address offset: 0x68-0x6C												 */
	__IO uint32_t BDCR;			/* RCC Backup domain control register,							Address offset: 0x70 */
	__IO uint32_t CSR;			/* RCC clock control & status register,							Address offset: 0x74 */
	uint32_t 	 RESERVED6[2];	/* Reserved, Address offset: 0x78-7C												 */
	__IO uint32_t SSCGR;			/* RCC spread spectrum clock generation register,				Address offset: 0x80 */
	__IO uint32_t PLLI2SCFGR;	/* RCC PLLI2S configuration register,							Address offset: 0x84 */
} RCC_TypeDef;


/*
 * Peripheral declarations (Peripheral base addresses typecasted to (x_TypeDef *))
 */

/* GPIOs */
#define GPIOA			((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB			((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC			((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD			((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE			((GPIO_TypeDef *)GPIOE_BASE)
#define GPIOF			((GPIO_TypeDef *)GPIOF_BASE)
#define GPIOG			((GPIO_TypeDef *)GPIOG_BASE)
#define GPIOH			((GPIO_TypeDef *)GPIOH_BASE)
#define GPIOI			((GPIO_TypeDef *)GPIOI_BASE)

/* RCC */
#define RCC				((RCC_TypeDef *)RCC_BASE)


/*
 * Clock enable macros for peripherals
 */

/* GPIOx */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

/* I2Cx */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/* SPIx */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

/* USARTx */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/* SYSCFG */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))



/*
 * Clock enable macros for peripherals
 */

/* GPIOx */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

/* I2Cx */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/* SPIx */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))

/* USARTx */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/* SYSCFG */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))





#endif /* INC_STM32F407XX_H_ */
