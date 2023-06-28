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

/*******************************************************************************
 * APIs (See the function definitions for more information)
 ******************************************************************************/

void LCD_Init(void);
void LCD_TxCmd(uint8_t cmd);



#endif /* LCD_H */
