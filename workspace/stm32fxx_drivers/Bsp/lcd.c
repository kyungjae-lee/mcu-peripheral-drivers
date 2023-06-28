/*******************************************************************************
 * Filename		: lcd.c
 * Description	: Implementation of APIs for 16x2 Character LCD module
 * 				  (4-bit interface; only DB4-DB7 pins of the LCD module will be
 * 				  used)
 * Author		: Kyungjae Lee
 * History 		: Jun 27, 2023 - Created file
 ******************************************************************************/

#include "stm32f407xx.h"
#include "lcd.h"

/* Function prototypes */
static void Write4Bits(uint8_t nibble);
static void LCD_Enable(void);
static void DelayMs(uint32_t delayInMs);

/**
 * LCD_Init()
 * Desc.	: Initializes GPIO pins to be used for LCD connections
 * Param.	: None
 * Return	: None
 * Note		: N/A
 */
void LCD_Init(void)
{
	GPIO_Handle_TypeDef gpioPin;

	/* 1. Configure the GPIO pins to be used for LCD connections **************/

	/* Configure the GPIO pin to be connected to LCD RS pin */
	gpioPin.pGPIOx = LCD_GPIO_PORT;
	gpioPin.GPIO_PinConfig.GPIO_PinMode = LCD_PIN_RS;
	gpioPin.GPIO_PinConfig.GPIO_PinOutType = GPIO_PIN_MODE_OUT;
	gpioPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	gpioPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_OUT_SPEED_HIGH;
	GPIO_Init(&gpioPin);

	/* Configure the GPIO pin to be connected to LCD RW pin */
	gpioPin.GPIO_PinConfig.GPIO_PinNumber = LCD_PIN_RW;
	GPIO_Init(&gpioPin);

	/* Configure the GPIO pin to be connected to LCD EN pin */
	gpioPin.GPIO_PinConfig.GPIO_PinNumber = LCD_PIN_EN;
	GPIO_Init(&gpioPin);

	/* Configure the GPIO pin to be connected to LCD D4 pin */
	GPIO_Init(&gpioPin);

	/* Configure the GPIO pin to be connected to LCD D5 pin */
	gpioPin.GPIO_PinConfig.GPIO_PinNumber = LCD_PIN_D5;
	GPIO_Init(&gpioPin);

	/* Configure the GPIO pin to be connected to LCD D6 pin */
	gpioPin.GPIO_PinConfig.GPIO_PinNumber = LCD_PIN_D6;
	GPIO_Init(&gpioPin);

	/* Configure the GPIO pin to be connected to LCD D7 pin */
	gpioPin.GPIO_PinConfig.GPIO_PinNumber = LCD_PIN_D7;
	GPIO_Init(&gpioPin);

	/* Set the configured GPIO pins' output values to 0 */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_RS, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_RW, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_EN, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_D4, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_D5, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_D6, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_D7, RESET);

	/* 2. Initialize LCD module ***********************************************
	 * (See 'Figure 24. 4-Bit Interface' in the LCD reference manual)
	 */

	/* Wait for more than 40 ms after Vcc rises to 2.7V */
	DelayMs(40);

	/* Set RS pin to 0 for LCD command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_RS, RESET);

	/* Set RW pin to 0 for writing to LCD (If you are always writing to LCD, you
	 * can keep this pin connected to GND)
	 */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_RW, RESET);

	Write4Bits(0x3);	/* 0 0 1 1 in D7 D6 D5 D4 order */

	DelayMs(5);

	Write4Bits(0x3);

	DelayMs(150);

	Write4Bits(0x3);

	Write4Bits(0x2);

	/* Now, ready to send the command */
} /* End of LCD_Init */

/**
 * LCD_TxCmd()
 * Desc.	:
 * Param.	: @cmd -
 * Return	: None
 * Note		: N/A
 */
void LCD_TxCmd(uint8_t cmd)
{
	/* 1. Set RS=0 for LCD command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_RS, RESET);

	/* 2. Set RW=0 for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_RW, RESET);

	Write4Bits(cmd >> 4);	/* Send the higher nibble */
	Write4Bits(cmd & 0xF);	/* Send the lower nibble */
} /* End of LCD_TxCmd */

/**
 * LCD_TxChar()
 * Desc.	: Sends a character to the LCD
 * Param.	: @ch -
 * Return	: None
 * Note		: This function assumes 4-bit parallel data transmission.
 * 			  First, the higher nibble of the data will be sent through the data
 * 			  lines D4, D5, d6, d7,
 * 			  then, the lower nibble of the data will be sent through the data
 * 			  lines D4, D5, d6, d7.
 */
void LCD_TxChar(uint8_t ch)
{
	/* 1. Set RS=1 for LCD user data */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_RS, SET);

	/* 2. Set RW=0 for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_RW, RESET);

	Write4Bits(ch >> 4);	/* Send the higher nibble */
	Write4Bits(ch & 0xF);	/* Send the lower nibble */
} /* End of LCD_TxChar */


/*******************************************************************************
 * Private functions
 ******************************************************************************/

/**
 * DelayMs()
 * Desc.	: Spinlock delays for @delayInMs milliseconds
 * Param.	: @delay - time to delay in milliseconds
 * Returns	: None
 * Note		: N/A
 */
static void DelayMs(uint32_t delayInMs)
{
	for (uint32_t i = 0; i < delayInMs * 1000; i++);
} /* End of DelayMs */

/**
 * Write4Bits()
 * Desc.	: Writes 4 bits of data/command to D4, D5, D6, D7 lines
 * 			  (msb to D7, lsb to D4)
 * Param.	: @value -
 * Return	: None
 * Note		: N/A
 */
static void Write4Bits(uint8_t nibble)
{
	/* Write each bit of @nibble to D4-D7 lines */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_D4, (nibble >> 0) & 0x1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_D5, (nibble >> 1) & 0x1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_D6, (nibble >> 2) & 0x1);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_D7, (nibble >> 3) & 0x1);

	/* After writing every nibble to the data lines, give the LCD time to latch
	 * that data inside the LCD
	 */
	/* Make high-to-low transition on EN pin (See the timing diagram) */
	LCD_Enable();

} /* End of Write4Bits */

/**
 * LCD_Enable()
 * Desc.	: Makes high-to-low transition on the EN line
 * Param.	: None
 * Return	: None
 * Note		: For the instruction execution time, give any value that is greater
 * 			  than 32 micro-seconds.
 */
static void LCD_Enable(void)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_EN, SET);
	DelayMs(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_EN, RESET);
	DelayMs(10);

} /* End of LCD_Enable */
