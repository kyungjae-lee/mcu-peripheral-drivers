/*****************************************************************************************
 * @ Filename		: stm32f407xx.h
 * @ Description	: Device header file for stm32f407xx MCU
 * @ Author			: Kyungjae Lee
 * @ Date created	: 05/18/2023
 ****************************************************************************************/

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 * Base addresses of memories
 */
#define FLASH_BASE		0x08000000U		/* Flash base address in the alias 				*/
#define SRAM1_BASE		0x20000000U		/* SRAM1(112 KB) base address in the alias 		*/
#define SRAM2_BASE		0x20001C00U		/* SRAM2(16 KB) base address in the alias 		*/
#define SRAM_BASE		SRAM1_BASE		/* SRAM	base address in the alias				*/
#define ROM_BASE 		0x1FFF0000U		/* ROM(system memory) base address in the alias	*/

/*
 * Base addresses of APBx and AHB bus peripherals
 */
#define PERIPH_BASE		0x40000000U		/* Peripheral base address in the alias 		*/
#define APB1PERIPH_BASE	PERIPH_BASE
#define APB2PERIPH_BASE	(PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE	(PERIPH_BASE + 0x00020000U)
#define AHB2PERIPH_BASE	(PERIPH_BASE + 0x10000000U)

/*
 * Base addresses of AHB1 peripherals
 */
#define GPIOA_BASE		(AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASE		(AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASE		(AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASE		(AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASE		(AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASE		(AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASE		(AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASE		(AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASE		(AHB1PERIPH_BASE + 0x2000U)
#define GPIOJ_BASE		(AHB1PERIPH_BASE + 0x2400U)
#define GPIOK_BASE		(AHB1PERIPH_BASE + 0x2800U)

/*
 * Base addresses of APB1 peripherals
 */
#define SPI2_BASE		(APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASE		(APB1PERIPH_BASE + 0x3C00U)
#define USART2_BASE		(APB1PERIPH_BASE + 0x4400U)
#define USART3_BASE		(APB1PERIPH_BASE + 0x4800U)
#define UART4_BASE		(APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASE		(APB1PERIPH_BASE + 0x5000U)
#define I2C1_BASE		(APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASE		(APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASE		(APB1PERIPH_BASE + 0x5C00U)

/*
 * Base addresses of APB2 peripherals
 */
#define USART1_BASE		(APB2PERIPH_BASE + 0x1000U)
#define USART6_BASE		(APB2PERIPH_BASE + 0x1400U)
#define SPI1_BASE		(APB2PERIPH_BASE + 0x3000U)
#define SYSCFG_BASE		(APB2PERIPH_BASE + 0x3800U)
#define EXTI_BASE		(APB2PERIPH_BASE + 0x3C00U)


#endif /* INC_STM32F407XX_H_ */
