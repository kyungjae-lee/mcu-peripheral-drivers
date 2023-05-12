/*****************************************************************************************
 * @ Filename		: main.c
 * @ Description	: Program to demonstrate issuing the user button interrupt to the
 * 					  processor
 * @ Author			: Kyungjae Lee
 * @ Date created	: 05/14/2023
 ****************************************************************************************/

#include <stdint.h>
#include <stdio.h>

/* RCC */
#define RCC_BASE			0x40023800UL
#define RCC_AHB1ENR			(*(uint32_t volatile *)(RCC_BASE + 0x30U))
#define GPIOAEN				(1 << 0)
#define RCC_APB2ENR			(*(uint32_t volatile *)(RCC_BASE + 0x44U))
#define SYSCFGEN			(1 << 0)

/* EXTI */
#define EXTI_BASE			0x40013C00UL
#define EXTI_IMR			(*(uint32_t volatile *)(EXTI_BASE + 0x00U))
#define MR0_UNMASK			(1 << 0)
#define EXTI_RTSR			(*(uint32_t volatile *)(EXTI_BASE + 0x08U))
#define TR0_EN				(1 << 0)
#define EXTI_PR				(*(uint32_t volatile *)(EXTI_BASE + 0x14U))

/* GPIOA */
#define GPIOA_BASE			0x40020000UL
#define GPIOA_MODER			(*(uint32_t volatile *)(GPIOA_BASE + 0x00U))

/* NVIC (Internal to the processor) */
#define NVIC_ISER0			(*(uint32_t volatile *)0xE000E100U)
#define EXTI0_EN			(1 << 6)

/* Global variables */
uint8_t volatile g_button_pressed = 0;	/* Shared between main and ISR */
uint32_t g_button_press_count = 0;

/* Function prototypes */
void button_init(void);

int main(void)
{
	button_init();

	/* Disable interrupt */
	EXTI_IMR &= ~(1 << 0);	/* Mask line 0 */

	if (g_button_pressed)
	{
		/* Button debouncing time - Spinlock delay */
		for (uint32_t volatile i = 0; i < 5000000 / 2; i++);

		g_button_press_count++;
		printf("Button is pressed: %lu\n", g_button_press_count);
		g_button_pressed = 0;
	}

	/* Enable interrupt */
	EXTI_IMR |= MR0_UNMASK;	/* Unmask line 0 */

	/* Loop forever */
	for (;;);
}

void button_init(void)
{
	/* Enable peripheral clock for GPIOA */
	RCC_AHB1ENR |= GPIOAEN;

	/* Enable system configuration controller clock */
	RCC_APB2ENR |= SYSCFGEN;

	/* Configure Edge detection; enable rising trigger (for Event and Interrupt) for input line 0 */
	EXTI_RTSR |= TR0_EN;

	/* Unmask (enable) interrupt request from line 0 */
	EXTI_IMR |= MR0_UNMASK;

	/* Enable NVIC IRQ for EXTI0 (NVIC number 6) */
	NVIC_ISER0 |= EXTI0_EN;

	/*
	 * [!] Note: ARM Cortex-M4 supports 15 system exceptions (internal to the proc.),
	 * 			 and 240 interrupts (external to the proc.) by default.
	 * 			 To support enabling/disabling 240 interrupts, 8 32-bit registers are
	 * 			 used. (NVIC_ISER0 - NVIC_ISER7) Each bit in these registers is
	 * 			 sequentially assigned to the interrupts.
	 *
	 * 			 e.g., EXTI0's NVIC number is 6, so its counterpart is bit[6] of
	 * 			 NVIC_ISER0.
	 * 			 e.g., I2C1_ER's NVIC number is 32, so its counterpart is bit[0] of
	 * 			 NVIC_ISER1.
	 */
}

/* Button interrupt handler */
void EXTI0_IRQHandler(void)
{
	/* Set this flag when button is pressed */
	g_button_pressed = 1;

	EXTI_PR |= (1 << 0);	/* Clear PR0 */
		/* Check if this clears all the pending bits in the EXTI_PR. There were people
		   saying that EXTI_PR = 1; is the right way to prevent from other pending bits
		   from being cleared. */

	/*
	 * [!] Note: Clearing pending bits in EXTI_PR register is a bit unusual.
	 * 			 A pending bit is set when the selected edge event arrives on the
	 * 			 external interrupt line.
	 * 			 A pending bit is cleared by programming it to '1'.
	 * 			 This is design choice of ST Micro.
	 */
}
