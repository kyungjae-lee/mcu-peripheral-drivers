/*******************************************************************************
 * File		: lcd_hd44780u.h
 * Brief	: APIs for the HD44780U 16x2 Character LCD module
 * Author	: Kyungjae Lee
 * Date		: Jun 27, 2023
 *
 * Note		: This code includes only the features that are necessary for my
 * 			  personal projects.
 ******************************************************************************/

#ifndef LCD_HD44780U_H
#define LCD_HD44780U_H

#include "stm32f407xx.h"

/* Application configurable items */
#define LCD_GPIO_PORT	GPIOD
#define LCD_PIN_RS		GPIO_PIN_0
#define LCD_PIN_RW		GPIO_PIN_1
#define LCD_PIN_EN		GPIO_PIN_2
#define LCD_PIN_D4		GPIO_PIN_3
#define LCD_PIN_D5		GPIO_PIN_4
#define LCD_PIN_D6		GPIO_PIN_5
#define LCD_PIN_D7		GPIO_PIN_6

/* LCD instructions */
#define LCD_INST_4DL_2N_5X8F	0x28 /* 4 data lines, 2 lines, 5x8 font size */
#define LCD_INST_DON_CURON		0x0E /* Display on, cursor on */
#define LCD_INST_INCADD			0x06 /* Entry mode set */
#define LCD_INST_CLEAR_DISPLAY	0x01 /* Clear display */
#define LCD_INST_RETURN_HOME	0x02 /* Return home */


/*******************************************************************************
 * APIs supported by the HD44780U 16x2 Character LCD module
 * (See the function definitions for more information)
 ******************************************************************************/

void LCD_Init(void);
void LCD_TxInstruction(uint8_t instruction);
void LCD_ClearDisplay(void);
void LCD_ReturnHome(void);
void LCD_PrintChar(uint8_t ch);
void LCD_PrintString(char *msg);
void LCD_SetCursor(uint8_t row, uint8_t column);


#endif /* LCD_HD44780U_H */
