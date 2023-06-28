/*******************************************************************************
 * Filename		: lcd.h
 * Description	: APIs for 16x2 Character LCD module
 * Author		: Kyungjae Lee
 * History 		: Jun 27, 2023 - Created file
 ******************************************************************************/

#ifndef LCD_H
#define LCD_H

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
 * APIs (See the function definitions for more information)
 ******************************************************************************/

void LCD_Init(void);
void LCD_TxInstruction(uint8_t instruction);
void LCD_TxChar(uint8_t ch);
void LCD_ClearDisplay(void);




#endif /* LCD_H */
