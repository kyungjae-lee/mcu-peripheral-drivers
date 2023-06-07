/**
 * Filename		: gpio_04_led_toggle_with_ext_button_interrupt.c
 * Description	: Program to toggle LED whenever an interrupt is triggered by
 * 				  the external button press
 * Author		: Kyungjae Lee
 * History		: May 24, 2023 - Created file
 * 				  Jun 02, 2023 - Removed redundant 'GPIO_PeriClockControl()'
 */

#include <string.h>
#include "stm32f407xx.h"

#define HIGH			1
#define LOW 			0
#define BTN_PRESSED 	LOW

/**
 * delay()
 * Desc.	: Spinlock delays the program execution
 * Param.	: None
 * Returns	: None
 * Note		: N/A
 */
void delay(void)
{
	/* Appoximately ~200ms delay when the system clock freq is 16 MHz */
	for (uint32_t i = 0; i < 500000 / 2; i++);
}

int main(int argc, char *argv[])
{
	GPIO_Handle_TypeDef GPIOLed, GPIOBtn;

	/* Zero-out all the fields in the structures (Very important! GPIOLed and GPIOBtn
	 * are local variables whose members may be filled with garbage values before
	 * initialization. These garbage values may set (corrupt) the bit fields that
	 * you did not touch assuming that they will be 0 by default. Do NOT make this
	 * mistake!
	 */
	memset(&GPIOLed, 0, sizeof(GPIOLed));
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	/* GPIOLed configuration */
	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_OUT_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOutType = GPIO_PIN_OUT_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIO_Init(&GPIOLed);

	/* GPIOBtn configuration */
	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_MODE_IT_FT;	/* Interrupt falling-edge */
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_OUT_SPEED_FAST; /* Doesn't matter */
	//GPIOBtn.GPIO_PinConfig.GPIO_PinOutType = GPIO_PIN_OUT_TYPE_PP;	/* N/A */
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_Init(&GPIOBtn);

	/* IRQ configurations */
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);	/* Optional in this case */
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);	/* EXTI 9 to 5 */

	while (1);
}

void EXTI9_5_IRQHandler(void)
{
	delay(); /* Debounding time */
	GPIO_IRQHandling(GPIO_PIN_6);	/* Clear the pending event from EXTI line */
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
}

