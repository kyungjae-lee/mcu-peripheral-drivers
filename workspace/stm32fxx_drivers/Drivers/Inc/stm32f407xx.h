/**
 * Filename		: stm32f407xx.h
 * Description	: Device header file for stm32f407xx MCU
 * Author		: Kyungjae Lee
 * Created     	: May 18, 2023
 * Updates		: May 28, 2023 - Add SPI peripheral specific information
 */

#ifndef STM32F407XX_H
#define STM32F407XX_H

#include <stdint.h>

#define __IO			volatile		/* I/O registers are highly volatile in nature */

/************************** START: Processor Specific Details ***************************/

/**
 * ARM Cortex-Mx processor NVIC_ISERx register addresses
 * Interrupt Set-enable Registers
 * Note: Since ARM Cortex-Mx process implements only 82 interrupts, ISER3-7 will not
 * 		 be used.
 * 		 NVIC could be defined as a structure, but just the necessary registers are
 * 		 defined separately here since there is too much space in between each group of
 * 		 registers.
 */
#define NVIC_ISER0		((uint32_t volatile *)0xE000E100)
#define NVIC_ISER1		((uint32_t volatile *)0xE000E104)
#define NVIC_ISER2		((uint32_t volatile *)0xE000E108)
#define NVIC_ISER3		((uint32_t volatile *)0xE000E10C)
#define NVIC_ISER4		((uint32_t volatile *)0xE000E110)
#define NVIC_ISER5		((uint32_t volatile *)0xE000E114)
#define NVIC_ISER6		((uint32_t volatile *)0xE000E118)
#define NVIC_ISER7		((uint32_t volatile *)0xE000E11C)

/**
 * ARM Cortex-Mx processor NVIC_ICERx register addresses
 * Interrupt Clear-enable Registers
 * Note: Since ARM Cortex-Mx process implements only 82 interrupts, ICER3-7 will not
 * 		 be used.
 * 		 NVIC could be defined as a structure, but just the necessary registers are
 * 		 defined separately here since there is too much space in between each group of
 * 		 registers.
 */
#define NVIC_ICER0		((uint32_t volatile *)0XE000E180)
#define NVIC_ICER1		((uint32_t volatile *)0xE000E184)
#define NVIC_ICER2		((uint32_t volatile *)0xE000E188)
#define NVIC_ICER3		((uint32_t volatile *)0xE000E18C)
#define NVIC_ICER4		((uint32_t volatile *)0xE000E190)
#define NVIC_ICER5		((uint32_t volatile *)0xE000E194)
#define NVIC_ICER6		((uint32_t volatile *)0xE000E198)
#define NVIC_ICER7		((uint32_t volatile *)0xE000E19C)

/**
 * ARM Cortex-Mx processor NVIC_IPRx register addresses
 * Interrupt Priority Registers
 * Note: NVIC could be defined as a structure, but just the necessary registers are
 * 		 defined separately here since there is too much space in between each group of
 * 		 registers.
 */
#define NVIC_IPR_BASE	((uint32_t volatile *)0xE000E400)

#define NUM_PRI_BITS_USED	4

/*************************** END: Processor Specific Details ****************************/


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
#define SPI4_BASE		(APB2PERIPH_BASE + 0x3400U)
#define SYSCFG_BASE		(APB2PERIPH_BASE + 0x3800U)
#define EXTI_BASE		(APB2PERIPH_BASE + 0x3C00U)


/**
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
	__IO uint32_t BDCR;			/* RCC Backup domain control register,								Address offset: 0x70 */
	__IO uint32_t CSR;			/* RCC clock control & status register,								Address offset: 0x74 */
	uint32_t 	 RESERVED6[2];	/* Reserved, Address offset: 0x78-7C												 */
	__IO uint32_t SSCGR;			/* RCC spread spectrum clock generation register,				Address offset: 0x80 */
	__IO uint32_t PLLI2SCFGR;	/* RCC PLLI2S configuration register,								Address offset: 0x84 */
} RCC_TypeDef;

/* External interrupt/event controller (EXTI) */
typedef struct
{
	__IO uint32_t IMR;			/* Interrupt mask register,				Address offset: 0x00 */
	__IO uint32_t EMR;			/* Event mask register,					Address offset: 0x04 */
	__IO uint32_t RTSR;			/* Rising trigger selection register,	Address offset: 0x08 */
	__IO uint32_t FTSR;			/* Falling trigger selection register,	Address offset: 0x0C */
	__IO uint32_t SWIER;		/* Software interrupt event register,	Address offset: 0x10 */
	__IO uint32_t PR;			/* Pending register,					Address offset: 0x14 */
} EXTI_TypeDef;

/* System configuration controller (SYSCFG) */
typedef struct
{
	__IO uint32_t MEMRMP;		/* SYSCFG memory remap register,						Address offset: 0x00 */
	__IO uint32_t PMC;			/* SYSCFG peripheral mode configuration register,		Address offset: 0x04 */
	__IO uint32_t EXTICR[4];	/* SYSCFG external interrupt configuration register1-4, Address offset: 0x08 */
	uint32_t RESERVED[2];		/* Reserved, Address offset: 0x18-0x1C										 */
	__IO uint32_t CMPCR;		/* Compensation cell control register,					Address offset: 0x20 */
} SYSCFG_TypeDef;

/* Serial peripheral interface (SPI) */
typedef struct
{
	__IO uint32_t CR1;			/* SPI control register 1,				Address offset: 0x00 */
	__IO uint32_t CR2;			/* SPI control register 2,				Address offset: 0x04 */
	__IO uint32_t SR;			/* SPI status register,					Address offset: 0x08 */
	__IO uint32_t DR;			/* SPI data register,					Address offset: 0x0C */
	__IO uint32_t SRCPR;		/* SPI CRC polynomial register,			Address offset: 0x10 */
	__IO uint32_t RXCRCR;		/* SPI RX CRC register,					Address offset: 0x14 */
	__IO uint32_t TXCRCR;		/* SPI TX CRC register,					Address offset: 0x18 */
	__IO uint32_t I2SCFGR;		/* SPI_I2S configuration register,		Address offset: 0x1C */
	__IO uint32_t I2SPR;		/* SPI_I2S prescaler register,			Address offset: 0x20 */

} SPI_TypeDef;
/**
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

/* EXTI */
#define EXTI			((EXTI_TypeDef *)EXTI_BASE)

/* SYSCFG */
#define SYSCFG			((SYSCFG_TypeDef *)SYSCFG_BASE)

/* SPIx */
#define SPI1			((SPI_TypeDef *)SPI1_BASE)
#define SPI2			((SPI_TypeDef *)SPI2_BASE)
#define SPI3			((SPI_TypeDef *)SPI3_BASE)
#define SPI4			((SPI_TypeDef *)SPI4_BASE)

/**
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



/**
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
#define SPI4_PCLK_DI()		(RCC->APB2ENR |= ~(1 << 13))

/* USARTx */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/* SYSCFG */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/**
 * Reset peripherals (set the corresponding bit, and then clear)
 */

/* GPIO */
#define GPIOA_RESET()		do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while (0)
#define GPIOB_RESET()		do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while (0)
#define GPIOC_RESET()		do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while (0)
#define GPIOD_RESET()		do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while (0)
#define GPIOE_RESET()		do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while (0)
#define GPIOF_RESET()		do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while (0)
#define GPIOG_RESET()		do { (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while (0)
#define GPIOH_RESET()		do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while (0)
#define GPIOI_RESET()		do { (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while (0)

/* SPI */
#define SPI1_RESET()		do { (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); } while (0)
#define SPI2_RESET()		do { (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); } while (0)
#define SPI3_RESET()		do { (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); } while (0)
#define SPI4_RESET()		do { (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); } while (0)

/**
 * Macro function to return port code for the given GPIO base address
 */
#define GPIO_BASE_TO_PORT_CODE(x)	((x == GPIOA) ? 0 :	\
									(x == GPIOB) ? 1 :	\
									(x == GPIOC) ? 2 :	\
									(x == GPIOD) ? 3 :	\
									(x == GPIOE) ? 4 :	\
									(x == GPIOF) ? 5 :	\
									(x == GPIOG) ? 6 :	\
									(x == GPIOH) ? 7 :	\
									(x == GPIOI) ? 8 : 0)

/**
 * Other generic macros
 */
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET

/**
 * Interrupt Request (IRQ) numbers of STM32F407xx MCU
 * Note: This information is specific to MCU family
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/**
 * Possible NVIC IRQ priority levels
 */
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15


/*****************************************************************************************
 * SPI peripheral bit position definitions
 ****************************************************************************************/

/* SPI_CR1 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR  		3
#define SPI_CR1_SPE 		6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI 		8
#define SPI_CR1_SSM 		9
#define SPI_CR1_RXONLY 		10
#define SPI_CR1_DFF 		11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN  		13
#define SPI_CR1_BIDIOE 		14
#define SPI_CR1_BIDIMODE	15

/* SPI_CR2 */
#define SPI_CR2_RXDMAEN 	0
#define SPI_CR2_TXDMAEN 	1
#define SPI_CR2_SSOE    	2
#define SPI_CR2_FRF     	4
#define SPI_CR2_ERRIE   	5
#define SPI_CR2_RXNEIE  	6
#define SPI_CR2_XEIE    	7

/* SPI_SR */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE 			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR 			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR 			6
#define SPI_SR_BSY 			7
#define SPI_SR_FRE 			8

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* STM32F407XX_H */
