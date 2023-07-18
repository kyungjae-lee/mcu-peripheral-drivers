/*******************************************************************************
 * File		: stm32f407xx_gpio_driver.h
 * Brief	: STM32F407xx MCU specific GPIO driver header file
 * Author	; Kyungjae Lee
 * Date		: May 21, 2023
 * ****************************************************************************/

#ifndef STM32F407XX_GPIO_DRIVER_H
#define STM32F407XX_GPIO_DRIVER_H

#include "stm32f407xx.h"

/*******************************************************************************
 * GPIOx peripheral structures
 ******************************************************************************/

/* GPIO pin configuration structure */
typedef struct
{
	uint8_t GPIO_PinNumber;			/* Available values @GPIO_PIN_NUMBERS 		*/
	uint8_t GPIO_PinMode;			/* Available values @GPIO_PIN_MODES 		*/
	uint8_t GPIO_PinSpeed;			/* Available values @GPIO_PIN_SPEED 		*/
	uint8_t GPIO_PinPuPdControl;	/* Available values @GPIO_PIN_PUPD 			*/
	uint8_t GPIO_PinOutType;		/* Available values @GPIO_PIN_OUT_TYPES 	*/
	uint8_t GPIO_PinAltFcnMode;		/* Only applicable when mode is set to alternate function mode */
} GPIO_PinConfig_TypeDef;

/* GPIO pin handle structure */
typedef struct
{
	GPIO_TypeDef *pGPIOx;					/* Holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_TypeDef GPIO_PinConfig;	/* Holds the GPIO pin configuration settings */
} GPIO_Handle_TypeDef;

/**
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN_4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10				10
#define GPIO_PIN_11				11
#define GPIO_PIN_12				12
#define GPIO_PIN_13				13
#define GPIO_PIN_14				14
#define GPIO_PIN_15				15

/**
 * @GPIO_PIN_MODES
 * GPIO pin modes
 */
#define GPIO_PIN_MODE_IN			0	/* Non-interrupt mode */
#define GPIO_PIN_MODE_OUT			1	/* Non-interrupt mode */
#define GPIO_PIN_MODE_ALTFCN		2	/* Non-interrupt mode */
#define GPIO_PIN_MODE_ANALOG		3	/* Non-interrupt mode */
#define GPIO_PIN_MODE_IT_FT			4	/* Falling edge trigger (for interrupt) */
#define GPIO_PIN_MODE_IT_RT			5	/* Rising edge trigger (for interrupt) */
#define GPIO_PIN_MODE_IT_RFT		6	/* Rising falling edge trigger (for interrupt) */

/**
 * @GPIO_PIN_OUT_TYPES
 * GPIO pin output types
 */
#define GPIO_PIN_OUT_TYPE_PP		0 	/* Push-pull type */
#define GPIO_PIN_OUT_TYPE_OD		1	/* Open-drain type */

/**
 * @GPIO_PIN_SPEED
 * GPIO pin output speeds
 */
#define GPIO_PIN_OUT_SPEED_LOW			0
#define GPIO_PIN_OUT_SPEED_MEDIUM		1
#define GPIO_PIN_OUT_SPEED_HIGH			2
#define GPIO_PIN_OUT_SPEED_VERY_HIGH	3

/**
 * @GPIO_PIN_PUPD
 * GPIO pin pull-up / pull-down configurations
 */
#define GPIO_PIN_NO_PUPD			0
#define GPIO_PIN_PU					1
#define GPIO_PIN_PD					2


/*******************************************************************************
 * APIs supported by the GPIO driver
 * (See function definitions for more information)
 ******************************************************************************/

/**
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t state);

/**
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_TypeDef *pGPIOHandle);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);	/* Utilize RCC_AHB1RSTR (AHB1 peripheral reset register) */

/**
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx);					/* Returns the contents of IDR */
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber);

/**
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t irqNumber, uint8_t state);
void GPIO_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void GPIO_IRQHandling(uint8_t pinNumber);


#endif /* STM32F407XX_GPIO_DRIVER_H */
