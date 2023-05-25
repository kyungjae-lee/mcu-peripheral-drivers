/**
 * Filename		: 03_button_led_external.c
 * Description	: Program to toggle the external LED whenever the external LED is pressed
 * Author		: Kyungjae Lee
 * Created on	: May 24, 2023
 */

#include "stm32f407xx.h"

#define HIGH			1
#define LOW 			0
#define BTN_PRESSED 	LOW

/* Spinlock delay */
void delay(void)
{
	for (uint32_t i = 0; i < 500000 / 2; i++);
}

int main(int argc, char *argv[])
{
	GPIO_Handle_TypeDef GPIOLed, GPIOBtn;

	/* GPIOLed configuration */
	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_OUT_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOutType = GPIO_PIN_OUT_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	GPIO_PeriClockControl(GPIOLed.pGPIOx, ENABLE);
	GPIO_Init(&GPIOLed);

	/* GPIOBtn configuration */
	GPIOBtn.pGPIOx = GPIOB;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_OUT_SPEED_FAST; /* Doesn't matter */
	//GPIOBtn.GPIO_PinConfig.GPIO_PinOutType = GPIO_PIN_OUT_TYPE_PP;	/* N/A */
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
		/* External pull-down resistor is already present (see the schematic) */
	GPIO_PeriClockControl(GPIOBtn.pGPIOx, ENABLE);
	GPIO_Init(&GPIOBtn);


	while (1)
	{
		if (GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, GPIOBtn.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED)
		{
			delay();	/* Introduce debouncing time */
			GPIO_ToggleOutputPin(GPIOLed.pGPIOx, GPIOLed.GPIO_PinConfig.GPIO_PinNumber);
		}
	}

	return 0;
}
