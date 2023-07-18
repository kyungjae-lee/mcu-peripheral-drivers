/*******************************************************************************
 * File		: lcd_hd44780u.c
 * Brief	: Implementation of APIs for the HD44780U 16x2 Character LCD module
 * 			  (4-bit interface; only DB4-7 pins of the LCD module will be used)
 * Author	: Kyungjae Lee
 * Date		: Jun 27, 2023
 *
 * Note		: This code includes only the features that are necessary for my
 * 			  personal projects.
 ******************************************************************************/

#include "lcd_hd44780u.h"
#include "stm32f407xx.h"

/* Function prototypes */
static void Write4Bits(uint8_t nibble);
static void LCD_Enable(void);
static void DelayMs(uint32_t delayInMs);
static void DelayUs(uint32_t delayInUs);

/**
 * LCD_Init()
 * Brief	: Initializes GPIO pins to be used for LCD connections
 * Param	: None
 * Retval	: None
 * Note		: N/A
 */
void LCD_Init(void)
{
	GPIO_Handle_TypeDef gpioPin;

	/* 1. Configure the GPIO pins to be used for LCD connections -------------*/

	/* Configure the GPIO pin to be connected to LCD RS pin */
	gpioPin.pGPIOx = LCD_GPIO_PORT;
	gpioPin.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_MODE_OUT;
	gpioPin.GPIO_PinConfig.GPIO_PinNumber = LCD_PIN_RS;
	gpioPin.GPIO_PinConfig.GPIO_PinOutType = GPIO_PIN_OUT_TYPE_PP;
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
	gpioPin.GPIO_PinConfig.GPIO_PinNumber = LCD_PIN_D4;
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

	/* 2. Initialize LCD module ------------------------------------------------
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

	DelayUs(150);

	Write4Bits(0x3);
	Write4Bits(0x2);

	/* Now, ready to send the command */

	/* Set to use 4 data lines, 2 lines, 5x8 char font size */
	LCD_TxInstruction(LCD_INST_4DL_2N_5X8F);

	/* Set display on, cursor on */
	LCD_TxInstruction(LCD_INST_DON_CURON);

	/* Clear display */
	LCD_ClearDisplay();

	/* Entry mode set */
	LCD_TxInstruction(LCD_INST_INCADD);

} /* End of LCD_Init */

/**
 * LCD_TxInstruction()
 * Brief	: Sends passed instruction (@instruction) to LCD
 * Param	: @instruction - LCD instruction to send
 * Retval	: None
 * Note		: N/A
 */
void LCD_TxInstruction(uint8_t instruction)
{
	/* 1. Set RS=0 for LCD command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_RS, RESET);

	/* 2. Set RW=0 for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_RW, RESET);

	Write4Bits(instruction >> 4);	/* Send the higher nibble */
	Write4Bits(instruction & 0xF);	/* Send the lower nibble */
} /* End of LCD_TxInstruction */

/**
 * LCD_PrintChar()
 * Brief	: Sends a character to be printed to the LCD
 * Param	: @ch - character to print
 * Retval	: None
 * Note		: This function assumes 4-bit parallel data transmission.
 * 			  First, the higher nibble of the data will be sent through the data
 * 			  lines D4, D5, d6, d7,
 * 			  then, the lower nibble of the data will be sent through the data
 * 			  lines D4, D5, d6, d7.
 */
void LCD_PrintChar(uint8_t ch)
{
	/* 1. Set RS=1 for LCD user data */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_RS, SET);

	/* 2. Set RW=0 for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_RW, RESET);

	Write4Bits(ch >> 4);	/* Send the higher nibble */
	Write4Bits(ch & 0xF);	/* Send the lower nibble */
} /* End of LCD_TxChar */

/**
 * LCD_ClearDisplay()
 * Brief	: Performs display clear routine
 * Param	: None
 * Retval	: None
 * Note		: After sending the clear display instruction, give it 2 ms as per
 * 			  the datasheet of LCD (See p.24 of LCD datasheet).
 */
void LCD_ClearDisplay(void)
{
	/* Clear display */
	LCD_TxInstruction(LCD_INST_CLEAR_DISPLAY);

	DelayMs(2);	/* Wait 2 ms as per the datasheet */
} /* End of LCD_ClearDisplay */

/**
 * LCD_ReturnHome()
 * Brief	: Returns the cursor to the home position
 * Param	: None
 * Retval	: None
 * Note		: After sending the clear display instruction, give it 2 ms as per
 * 			  the datasheet of LCD (See p.24 of LCD datasheet).
 */
void LCD_ReturnHome(void)
{
	/* Return home */
	LCD_TxInstruction(LCD_INST_RETURN_HOME);

	DelayMs(2);	/* Wait 2 ms as per the datasheet */
} /* End of LCD_ReturnHome */

/**
 * LCD_PrintString()
 * Brief	: Prints the passed string @msg to the LCD screen
 * Param	: @msg - string to print
 * Retval	: None
 * Note		: N/A
 */
void LCD_PrintString(char *msg)
{
	do
	{
		LCD_PrintChar((uint8_t)*msg++);
	}
	while (*msg != '\0');
} /* End of LCD_PrintString */

/**
 * LCD_SetCursor()
 * Brief	: Sets the cursor position (@row, @column)
 * Param	: @row - the row in which the cursor should be placed
 * 			  @column - the column in which the cursor should be placed
 * Retval	: None
 * Note		: Row number: 1 ~ 2
 * 			  Column number: 1 ~ 16 assuming a 2x16 character display
 */
void LCD_SetCursor(uint8_t row, uint8_t column)
{
	column--;

	switch (row)
	{
	case 1:
		/* Set cursor to 1st row address and add index */
		LCD_TxInstruction((column |= 0x80));
		break;
	case 2:
		/* Set cursor to 2nd row address and add index */
		LCD_TxInstruction((column |= 0xC0));
		break;
	default:
		break;
	}
} /* End of LCD_SetCursor */


/*******************************************************************************
 * Private functions
 ******************************************************************************/

/**
 * DelayMs()
 * Brief	: Spinlock delays for @delayInMs milliseconds
 * Param	: @delayInMs - time to delay in milliseconds
 * Retval	: None
 * Note		: N/A
 */
static void DelayMs(uint32_t delayInMs)
{
	for (uint32_t i = 0; i < delayInMs * 1000; i++);
} /* End of DelayMs */

/**
 * DelayUs()
 * Brief	: Spinlock delays for @delayInUs microseconds
 * Param	: @delayInUs - time to delay in microseconds
 * Retval	: None
 * Note		: N/A
 */
static void DelayUs(uint32_t delayInUs)
{
	for (uint32_t i = 0; i < delayInUs * 1; i++);
} /* End of DelayMs */

/**
 * Write4Bits()
 * Brief	: Writes 4 bits of data/command to D4, D5, D6, D7 lines
 * 			  (msb to D7, lsb to D4)
 * Param	: @value -
 * Retval	: None
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
 * Brief	: Makes high-to-low transition on the EN line
 * Param	: None
 * Retval	: None
 * Note		: For the instruction execution time, give any value that is greater
 * 			  than 32 micro-seconds.
 */
static void LCD_Enable(void)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_EN, SET);
	DelayUs(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_PIN_EN, RESET);
	DelayUs(100);
} /* End of LCD_Enable */
