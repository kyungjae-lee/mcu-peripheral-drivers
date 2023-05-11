/*****************************************************************************************
 * @ Filename		: main.c
 * @ Description	: Program to output HSI clock on a microcontroller pin. Measure it
 * 					  using oscilloscope or logic analyzer if possible.
 * @ Author			: Kyungjae Lee
 * @ Date created	: 05/14/2023
 ****************************************************************************************/

#include <stdint.h>

/* RCC */
#define RCC_BASE			0x40023800U
#define RCC_CFGR			(*(uint32_t volatile *)(RCC_BASE + 0x08U))
#define MCO1PRE_DIV_BY_4	(6 << 24)
#define RCC_AHB1ENR			(*(uint32_t volatile *)(RCC_BASE + 0x30U))
#define GPIOAEN				(1 << 0)

/* GPIO */
#define GPIOA_BASE			0x40020000U
#define GPIOA_MODER			(*(uint32_t volatile *)(GPIOA_BASE + 0x00U))
#define ALT_FCN_MODE		(2 << 16)
#define GPIOA_AFRH			(*(uint32_t volatile *)(GPIOA_BASE + 0x24U))

int main(void)
{
	/*************************************************************************************
	 * 1. Select the desired clock for the MCOx signal (Microcontroller Clock Output)
	 ************************************************************************************/

	RCC_CFGR &= ~(3 << 21);		/* Clear MCO1 bits[22:21]. 00: HSI clock selected */

	/*************************************************************************************
	 * 2. Output the MCOx signal on the MCU pin
	 ************************************************************************************/

	/* Enable peripheral clock for GPIOA */
	RCC_AHB1ENR |= GPIOAEN;

	/* Configure GPIOA pin 8 as alternate function mode */
	GPIOA_MODER &= ~(3 << 16);		/* Clear */
	GPIOA_MODER |= ALT_FCN_MODE;	/* Set */

	/* Configure alternate function register to set the mode AF0 for PA8 */
	GPIOA_AFRH &= ~(0xF << 0);

	/*
	 * To test the output with logic analyzer:
	 *
	 * 1. Connect CH0 of the logic analyzer with pin PA8 of the board.
	 * 2. Connect GND of the logic analyzer with pin GND of the board.
	 *
	 * [!] Note: If you are getting unexpected frequency due to the limitation of the
	 * 			 Logic analyzer hardware, adjust the HSI frequency by using the
	 * 			 prescalar value.
	 */

	/* Configure MCO1 prescalar */
	RCC_CFGR &= ~(8 << 24);			/* Clear */
	RCC_CFGR |= MCO1PRE_DIV_BY_4;	/* Set */

	/* Now, you should be getting 4 MHz in CH0 of logic analyzer */

    /* Loop forever */
	for(;;);
}
