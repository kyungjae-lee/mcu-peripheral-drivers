/*******************************************************************************
 * File		: stm32f407xx_gpio_driver.c
 * Brief	: STM32F407xx MCU specific GPIO driver source file
 * Author	; Kyungjae Lee
 * Date		: May 21, 2023
 * ****************************************************************************/

#include "stm32f407xx_gpio_driver.h"

/*******************************************************************************
 * APIs supported by the GPIO driver
 * (See function definitions for more information)
 ******************************************************************************/

/**
 * GPIO_PeriClockControl()
 * Brief	: Enables or disables peripheral clock for the given GPIO port
 * Param	: @pGPIOx - base address of GPIO peripheral
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: N/A
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
} /* End of GPIO_PeriClockControl */

/**
 * Init and de-init
 */

/**
 * GPIO_Init()
 * Brief	: Configures GPIO pin
 * Param	: @pGPIOHandle - pointer to the GPIO handle structure
 * Retval	: None
 * Note		: N/A
 */
void GPIO_Init(GPIO_Handle_TypeDef *pGPIOHandle)
{
	uint32_t temp = 0;

	/* Enable peripheral clock */
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

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

		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_PIN_MODE_IT_FT)
		{
			/**
			 * Configure the Falling Trigger Selection Register (FTSR)
			 */

			/* Set the corresponding bit of FTSR */
			EXTI->FTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			/* Clear the corresponding bit of RTSR in case it is set */
			EXTI->RTSR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_PIN_MODE_IT_FT)
		{
			/**
			 * Configure the Rising Trigger Selection Register (RTSR)
			 */

			/* Set the corresponding bit of RTSR */
			EXTI->RTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			/* Clear the corresponding bit of FTSR in case it is set */
			EXTI->FTSR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_PIN_MODE_IT_RFT)
		{
			/**
			 * Configure both the FTSR and RTSR
			 */

			/* Set the corresponding bit of FTSR and RTSR */
			EXTI->FTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		/* Configure the GPIO port selection in SYSCFG_EXTICR */
		uint8_t index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t bitOffset = (pGPIOHandle-> GPIO_PinConfig.GPIO_PinNumber % 4) * 4;
		uint8_t portCode = GPIO_BASE_TO_PORT_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();	/* Clock for SYSCFG must be enabled for the
							   following bit manipulations to take effect */
		SYSCFG->EXTICR[index] &= ~(0xF << bitOffset);
		SYSCFG->EXTICR[index] |= (portCode << bitOffset);

		/* Enable the EXTI interrupt delivery by using the Interrupt Mask Register (IMR) */
		EXTI->IMR |= (0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
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
		uint8_t bitOffset = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8) * 4;
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFcnMode << bitOffset);
		pGPIOHandle->pGPIOx->AFR[index] &= ~(0xF << bitOffset);	/* Clear */
		pGPIOHandle->pGPIOx->AFR[index] |= temp;				/* Set */
	}
} /* End of GPIO_Init */

/**
 * GPIO_DeInit()
 * Brief	: Deinitializes GPIO pin
 * Params.	: @pGPIOx - base address of GPIO peripheral
 * Retval	: None
 * Note		: N/A
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
} /* End of GPIO_DeInit */

/**
 * GPIO_ReadFromInputPin()
 * Brief	: Reads the input pin (@pGPIOx, @pinNumber) and returns the read value
 * Param	: @pGPIOx - base address of GPIO peripheral
 * 			  @pinNumber - pin number
 * Retval	: 0 or 1
 * Note 	: N/A
 */
uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber)
{
	return (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x1);
} /* End of GPIO_ReadFromInputPin */

/**
 * GPIO_ReadFromInputPort()
 * Brief	: Returns the contents of @pGPIOx IDR
 * Param	: @pGPIOx - base address of GPIO peripheral
 * Retval	: Contents of IDR (least significant 16 bits)
 * @note	: N/A
 */
uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
} /* End of GPIO_ReadFromInputPort */

/**
 * GPIO_WriteToOutputPin()
 * Brief	: Writes @value to the output pin @pGPIOx @pinNumber
 * Param	: @pGPIOx - base address of GPIO peripheral
 * 			  @pinNumber - output pin number
 * 			  @value - value to write to output pin
 * Retval	: None
 * Note		: Be aware of the possibility that a user may pass 0-255 value to @value
 */
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	if (value == GPIO_PIN_RESET)
		pGPIOx->ODR &= ~(0x1 << pinNumber);
	else
		pGPIOx->ODR |= (0x1 << pinNumber);
} /* End of GPIO_WriteToOutputPin */

/**
 * GPIO_WriteToOutputPort()
 * Brief	: Writes @value to @pGPIOx ODR
 * Param	: @pGPIOx - base address of GPIO peripheral
 * 			  @pinNumber - output pin number
 * 			  @value - value to write to output pin
 * Retval	: None
 * Note		: Be aware of the possibility that a user may pass 0-255 value to @value
 */
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
} /* End of GPIO_WriteToOutputPort */

/**
 * GPIO_ToggleOutputPin()
 * Brief	: Toggles output pin @pGPIOx @pinNumber
 * Param	: @pGPIOx - base address of GPIO peripheral
 * 			  @pinNumber - output pin number
 * Retval	: None
 * Note		: N/A
 */
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= (0x1 << pinNumber);
} /* End of GPIO_ToggleOutputPin */

/**
 * GPIO_IRQInterruptConfig()
 * Brief	: Configures IRQ interrupts (processor; NVIC_ISERx, NVIC_ICERx)
 * Param	: @irqNumber - IRQ number
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: Reference - Cortex-M4 Devices Generic User Guide
 * 			  Clearing ISERx bit won't disable the interrupt.
 * 			  To disable interrupt ICERx bit has to be set!
 */
void GPIO_IRQInterruptConfig(uint8_t irqNumber, uint8_t state)
{
	if (state == ENABLE)
	{
		/* Configure NVIC_ISERx register */
		if (irqNumber <= 31)
			*NVIC_ISER0 |= (0x1 << irqNumber);
		else if (32 <= irqNumber && irqNumber <= 63)
			*NVIC_ISER1 |= (0x1 << irqNumber % 32);
		else if (64 <= irqNumber && irqNumber <= 95)
			*NVIC_ISER2 |= (0x1 << irqNumber % 32);
	}
	else
	{
		/* Configure NVIC_ICERx register */
		if (irqNumber <= 31)
			*NVIC_ICER0 |= (0x1 << irqNumber);
		else if (32 <= irqNumber && irqNumber <= 63)
			*NVIC_ICER1 |= (0x1 << irqNumber % 32);
		else if (64 <= irqNumber && irqNumber <= 95)
			*NVIC_ICER2 |= (0x1 << irqNumber % 32);
	}
} /* End of GPIO_IRQInterruptConfig */

/**
 * GPIO_IRQPriorityConfig()
 * Brief	: Configures IRQ priority (processor; NVIC_IPRx)
 * Param	: @irqNumber - IRQ number
 * 			  @irqPriotity - IRQ priority (Make sure this parameter is of
 * 			  				 type uint32_t. Due to the number of bits it
 * 			  				 needs to be shifted during the calculation,
 * 							 declaring it as uint8_t did not do its job.)
 * 			  @state - ENABLE or DISABLE macro
 * Retval	: None
 * Note		: Reference - Cortex-M4 Devices Generic User Guide
 * 			  STM32F407xx MCUs use only 4 most significant bits within
 * 			  each 8-bit section of IPRx. Make sure to account for
 * 			  this offset when calculating the place to write the
 * 			  priority.
 */
void GPIO_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority)
{
	/* Find out the IPR register */
	uint8_t iprNumber = irqNumber / 4;
	uint8_t iprSection = irqNumber % 4;
	uint8_t bitOffset = (iprSection * 8) + (8 - NUM_PRI_BITS_USED);
	*(NVIC_IPR_BASE + iprNumber) |= (irqPriority << bitOffset);
} /* GPIO_IRQPriorityConfig */

/**
 * GPIO_IRQHandling()
 * Brief	: Clears the pending state of the triggered IRQ
 * Param	: @pinNumber - Pin number
 * Retval	: None
 * Note		: Clearing the pending bit in EXTI_PR is a bit unusual! The set bit
 * 			  can be cleared by programming the bit to 1. Don't be confused!
 * 			  This is by design!
 */
void GPIO_IRQHandling(uint8_t pinNumber)
{
	/* Clear the corresponding bit of the EXTI_PR (Pending Register) */
	if (EXTI->PR & (0x1 << pinNumber))
	{
		/* Clear the bit by programming it to 1 */
		EXTI->PR |= (0x1 << pinNumber);
	}
} /* End of GPIO_IRQHandling */
