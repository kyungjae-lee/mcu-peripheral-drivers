/*****************************************************************************************
 * @ Filename		: main.c
 * @ Description	: Program to demonstrate how to enable peripheral clock before using
 * 					  any peripherals. (ADC1 is used for example in thie program.)
 * @ Author			: Kyungjae Lee
 * @ Date created	: 05/14/2023
 ****************************************************************************************/

#include <stdint.h>

/* RCC */
#define RCC_BASE		0x40023800U
#define	RCC_APB2ENR		(*(uint32_t volatile *)(RCC_BASE + 0x44U))
#define ADC1EN			(1 << 8)

/* ADC1 */
#define ADC_BASE		0x40012000U
#define ADC_CR1			(*(uint32_t volatile *)(ADC_BASE + 0x04U))
#define SCAN			(1 << 8)

int main(void)
{
	/* Enable peripheral clock for ADC1 */
	RCC_APB2ENR |= ADC1EN;

	/* Enable scan mode */
	ADC_CR1 |= SCAN;	/* No effect unless corresponding peripheral clock is enabled. */

    /* Loop forever */
	for(;;);
}
