/**
 * Filename		: stm32f407xx_gpio_driver.c
 * Description	: STM32F407xx MCU specific GPIO driver source file
 * Author		: Kyungjae Lee
 * Created on	: May 18, 2023
 */

#include "stm32f407xx_gpio_driver.h"

/*****************************************************************************************
 * APIs supported by the GPIO driver (See function definitions for more information)
 ****************************************************************************************/

/**
 * Peripheral clock setup
 */

/**
 * @fn			: GPIO_PeriClockControl
 * @brief		: enables or disables peripheral clock for the given GPIO port
 * @param[in]	: pGPIOx - base address of GPIO peripheral
 * @param[in]	: state - ENABLE or DISABLE macros
 * @return		: none
 * @note		: n/a
 */
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t state)
{
	if (state == ENABLE)
	{
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if (pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if (pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if (pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
		else if (pGPIOx == GPIOI)
			GPIOI_PCLK_EN();
	}
	else
	{
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if (pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		else if (pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		else if (pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		else if (pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		else if (pGPIOx == GPIOF)
			GPIOF_PCLK_DI();
		else if (pGPIOx == GPIOG)
			GPIOG_PCLK_DI();
		else if (pGPIOx == GPIOH)
			GPIOH_PCLK_DI();
		else if (pGPIOx == GPIOI)
			GPIOI_PCLK_DI();

	}
}

/**
 * Init and de-init
 */

/**
 * @fn			: GPIO_Init
 * @brief		: configures GPIO pins
 * @param[in]	: pGPIOHandle - pointer to the GPIO handle structure
 * @return		: none
 * @note		: n/a
 */
void GPIO_Init(GPIO_Handle_TypeDef *pGPIOHandle)
{
	uint32_t temp = 0;

	/* Configure GPIO pin mode */
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_PIN_MODE_ANALOG)
	{
		/**
		 *  Non-interrupt mode
		 */

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* Clear */
		pGPIOHandle->pGPIOx->MODER |= temp;	/* Set */
	}
	else
	{
		/**
		 * Interrupt mode
		 */

	}

	/* Configure GPIO pin speed */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* Clear */
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; /* Set */

	/* Configure GPIO pin pull-up / pull-down setting */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* Clear */
	pGPIOHandle->pGPIOx->PUPDR |= temp; /* Set */

	/* Configure GPIO pin output type */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOutType << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* Clear */
	pGPIOHandle->pGPIOx->OTYPER |= temp; /* Set */

	/* Configure GPIO pin alternate functionality (Applicable only when mode is alternate function mode) */
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_PIN_MODE_ALTFCN)
	{
		/* Configure alternate function registers */
		uint8_t index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t offset = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8) * 4;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFcnMode << offset);
		pGPIOHandle->pGPIOx->AFR[index] &= ~(0xF << offset);	/* Clear */
		pGPIOHandle->pGPIOx->AFR[index] |= temp;				/* Set */
	}
}

/**
 * @fn			: GPIO_DeInit
 * @brief		: configures GPIO pins
 * @param[in]	: pGPIOx - base address of GPIO peripheral
 * @return		: none
 * @note		: n/a
 */
void GPIO_DeInit(GPIO_TypeDef *pGPIOx)	/* Utilize RCC_AHB1RSTR (AHB1 peripheral reset register to reset in one queue) */
{
	/* Set and clear the corresponding bit (0-8) of RCC_AHB1RSTR to reset all the pins of the GPIOx port */
	if (pGPIOx == GPIOA)
		GPIOA_RESET();
	else if (pGPIOx == GPIOB)
		GPIOB_RESET();
	else if (pGPIOx == GPIOC)
		GPIOC_RESET();
	else if (pGPIOx == GPIOD)
		GPIOD_RESET();
	else if (pGPIOx == GPIOE)
		GPIOE_RESET();
	else if (pGPIOx == GPIOF)
		GPIOF_RESET();
	else if (pGPIOx == GPIOG)
		GPIOG_RESET();
	else if (pGPIOx == GPIOH)
		GPIOH_RESET();
	else if (pGPIOx == GPIOI)
		GPIOI_RESET();
}

/**
 * Data read and write
 */

/**
 * @fn			: GPIO_ReadFromInputPin
 * @brief		: reads the input pin (@pGPIOx, @pinNumber) and returns the read value
 * @param[in]	: pGPIOx - base address of GPIO peripheral
 * @param[in]	: pinNumber - pin number
 * @return		: 0 or 1
 * @note		: n/a
 */
uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber)
{
	return (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x1);
}

/**
 * @fn			: GPIO_ReadFromInputPort
 * @brief		: returns the contents of @pGPIOx IDR
 * @param[in]	: pGPIOx - base address of GPIO peripheral
 * @return		: contents of IDR (least significant 16 bits)
 * @note		: n/a
 */
uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
}

/**
 * @fn			: GPIO_WriteToOutputPin
 * @brief		: writes @value to the output pin @pGPIOx @pinNumber
 * @param[in]	: pGPIOx - base address of GPIO peripheral
 * @param[in]	: pinNumber - output pin number
 * @param[in]	: value - value to write to output pin
 * @return		: none
 * @note		: be aware of the possibility that a user may pass 0-255 value to @value
 */
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	if (value == GPIO_PIN_RESET)
		pGPIOx->ODR &= ~(0x1 << pinNumber);
	else
		pGPIOx->ODR |= (0x1 << pinNumber);
}

void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= (0x1 << pinNumber);
}

/**
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t irqNumber, uint8_t irqPriority, uint8_t state)
{
}

void GPIO_IRQHandling(uint8_t pinNumber)
{
}
