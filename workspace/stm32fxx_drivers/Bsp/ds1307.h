/*******************************************************************************
 * Filename		: ds1307.h
 * Description	: APIs for DS1307 RTC module
 * Author		: Kyungjae Lee
 * History 		: Jun 25, 2023 - Created file
 ******************************************************************************/

#ifndef DS1307_H
#define DS1307_H

#include "stm32f407xx.h"

/* Application configurable items */
#define DS1307_I2C				I2C1
#define DS1307_I2C_GPIO_PORT	GPIOB
#define DS1307_I2C_PIN_SCL		GPIO_PIN_6
#define DS1307_I2C_PIN_SDA		GPIO_PIN_7
#define DS1307_I2C_SPEED		I2C_SCL_SPEED_SM /* Doesn't support fast mode */
#define DS1307_I2C_PUPD			GPIO_PIN_PU		 /* Using internal pull-up R */


/* Register addresses */
#define DS1307_SEC				0x00
#define DS1307_MIN				0x01
#define DS1307_HR				0x02
#define DS1307_DAY				0x03
#define DS1307_DATE  			0x04
#define DS1307_MONTH			0x05
#define DS1307_YEAR				0x06

/* DS1307 I2C address */
#define DS1307_I2C_ADDR			0x68 /* 1101000(2) */

/* Time formats */
#define TIME_FORMAT_12HRS_AM 	0
#define TIME_FORMAT_12HRS_PM	1
#define TIME_FORMAT_24HRS	 	2

/* Days */
#define SUNDAY					0
#define MONDAY					1
#define TUESDAY					2
#define WEDNESDAY				3
#define THURSDAY				4
#define FRIDAY					5
#define SATURDAY				6


typedef struct
{
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day;
} RTC_Date_TypeDef;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t	timeFormat;
} RTC_Time_TypeDef;

/*******************************************************************************
 * APIs (See the function definitions for more information)
 ******************************************************************************/

uint8_t DS1307_Init(void);
void DS1307_SetCurrentTime(RTC_Time_TypeDef *);
void DS1307_GetCurrentTime(RTC_Time_TypeDef *);
void DS1307_SetCurrentDate(RTC_Date_TypeDef *);
void DS1307_GetCurrentDate(RTC_Date_TypeDef *);

#endif /* DS1307_H */
