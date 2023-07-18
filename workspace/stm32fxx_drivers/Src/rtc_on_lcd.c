/*******************************************************************************
 * File		: rtc_on_lcd.c
 * Brief	: Application to read time and date information from DS1207 RTC chip
 *			  and display it on the 16x2 Character LCD
 * Author	: Kyungjae Lee
 * Date		: Jun 25, 2023
 ******************************************************************************/

#include <lcd_hd44780u.h>
#include "rtc_ds1307.h"
#include <string.h> 		/* strlen() */
#include <stdio.h> 			/* printf() */
#include "stm32f407xx.h"

#define SYSTICK_TIM_CLK		16000000UL

/* SysTick timer registers */
#define SYST_CSR			(*(uint32_t volatile *)0xE000E010)
#define SYST_RVR			(*(uint32_t volatile *)0xE000E014)

/* SysTick Control and Status Register (SYST_CSR) bit fields */
#define SYST_CSR_ENABLE		(1 << 0U) /* Counter enabled */
#define SYST_CSR_TICKINT	(1 << 1U)
#define SYST_CSR_CLKSOURCE	(1 << 2U) /* Processor clock */

#define PRINT_ON_LCD	/* Comment this out if LCD is not installed */

/* Function prototypes */
void DisplayPrologue(void);
char* TimeToString(RTC_Time_TypeDef *rtcTime);
char* DateToString(RTC_Date_TypeDef *rtcDate);
char* GetDay(uint8_t dayCode);
void NumToString(uint8_t num, char *buf);
void SysTickTimer_Init(uint32_t tickHz);
void DelayMs(uint32_t delayInMs);


int main(int argc, char *argv[])
{
	RTC_Time_TypeDef currTime;
	RTC_Date_TypeDef currDate;
	char *amPm;

	printf("Application running with LCD...\n");

#ifndef PRINT_ON_LCD
    printf("Application running without LCD...\n");
#else
    /* Initialize LCD module */
	LCD_Init();

	/* Display the project title and the name of the author */
	DisplayPrologue();
#endif

	/* Initialize Tiny RTC module */
	if (DS1307_Init())
	{
		/* RTC initialization was unsuccessful */
		printf("RTC init failed\n");

		while (1);
	}

	/* Initialize the SysTick Timer so it generates 1 interrupt per second */
	SysTickTimer_Init(1);

	/* Configure initial date and time to set */
	currDate.day = FRIDAY;
	currDate.date = 14;
	currDate.month = 7;
	currDate.year = 23;		/* Only the last two digits of the year */

	currTime.hours = 11;
	currTime.minutes = 59;
	currTime.seconds = 50;
	currTime.timeFormat = TIME_FORMAT_12HRS_PM;

	/* Set initial time and date */
	DS1307_SetCurrentDate(&currDate);
	DS1307_SetCurrentTime(&currTime);

	/* Get initial time and date */
	DS1307_GetCurrentDate(&currDate);
	DS1307_GetCurrentTime(&currTime);

	/* Print current time ----------------------------------------------------*/

	if (currTime.timeFormat != TIME_FORMAT_24HRS)
	{
		/* 12HRS format (e.g., 08:33:45 PM) */
		amPm = (currTime.timeFormat) ? "PM" : "AM";

#ifndef PRINT_ON_LCD
		printf("Current time = %s %s\n", TimeToString(&currTime), amPm);
#else
		LCD_SetCursor(2, 1);
		LCD_PrintString(TimeToString(&currTime));
		LCD_PrintString(" ");
		LCD_PrintString(amPm);
#endif
	}
	else
	{
		/* 24HRS format (e.g., 20:33:45) */

#ifndef PRINT_ON_LCD
		printf("Current time = %s\n", TimeToString(&currTime));
#else
		LCD_PrintString(TimeToString(&currTime));
#endif
	}

	/* Print current date in 'MM/DD/YY Day' format ---------------------------*/

#ifndef PRINT_ON_LCD
	printf("Current date = %s %s\n", DateToString(&currDate), GetDay(currDate.day));
#else
	LCD_SetCursor(1, 1);
	LCD_PrintString(DateToString(&currDate));
	LCD_PrintString(" ");
	LCD_PrintString(GetDay(currDate.day));
#endif

	while (1);

	return 0;
} /* End of main */

/**
 * DisplayPrologue()
 * Desc.	: Displays the project title and the author's name
 * Param.	: None
 * Returns	: None
 * Note		: N/A
 */
void DisplayPrologue(void)
{
	/* Display the project title */
    LCD_PrintString("Real-time clock");
	LCD_SetCursor(2, 1);
    LCD_PrintString("on 16x2 LCD");

    /* Introduce inter-frame delay */
    DelayMs(3000);

    /* Clear display */
    LCD_ClearDisplay();
    LCD_ReturnHome();

    /* Display the author's name */
    LCD_PrintString("by");
	LCD_SetCursor(2, 1);
    LCD_PrintString("Kyungjae Lee");

    DelayMs(3000);

    /* Clear display */
    LCD_ClearDisplay();
    LCD_ReturnHome();
} /* End of DisplayPrologue */

/**
 * TimeToString()
 * Desc.	: Converts the time represented by @rtcTime to a string
 * Param.	: @rtcTime - pointer to RTC Time structure
 * Return	: Time in string format (e.g., HH:MM:SS PM)
 * Note		: N/A
 */
char* TimeToString(RTC_Time_TypeDef *rtcTime)
{
	static char buf[9]; /* Make it static not to return a dangling ptr */

	buf[2] = ':';
	buf[5] = ':';

	NumToString(rtcTime->hours, buf);
	NumToString(rtcTime->minutes, &buf[3]);
	NumToString(rtcTime->seconds, &buf[6]);

	buf[8] = '\0';

	return buf;
} /* End of TimeToString */

/**
 * DateToString()
 * Desc.	: Converts the date represented by @rtcDate to a string
 * Param.	: @rtcDate - pointer to RTC Date structure
 * Return	: Date in string format (e.g., MM/DD/YY <Day>)
 * Note		: N/A
 */
char* DateToString(RTC_Date_TypeDef *rtcDate)
{
	static char buf[9]; /* Make it static not to return a dangling ptr */

	buf[2] = '/';
	buf[5] = '/';

	NumToString(rtcDate->month, buf);
	NumToString(rtcDate->date, &buf[3]);
	NumToString(rtcDate->year, &buf[6]);

	buf[8] = '\0';

	return buf;
} /* End of DateToString */

/**
 * GetDay()
 * Desc.	: Gets the day represented by @dayCode in string format
 * Param.	: @dayCode - day macro
 * Return	: Day in string format (e.g., Sunday)
 * Note		: N/A
 */
char* GetDay(uint8_t dayCode)
{
	char *days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

	return days[dayCode];
} /* End of GetDay */

/**
 * NumToString()
 * Desc.	: Converts the passed uint8_t number @num and store it into @buf
 * Param.	: @num - a number to be converted into a string
 * 			  @buf - pointer to a string buffer
 * Return	: None
 * Note		: N/A
 */
void NumToString(uint8_t num, char *buf)
{
	if (num < 10)
	{
		buf[0] = '0';
		buf[1] = num + '0';
		//buf[1] = num + 48;
	}
	else if (10 <= num && num < 99)
	{
		buf[0] = num / 10 + '0';
		buf[1] = num % 10 + '0';
		//buf[0] = num / 10 + 48;
		//buf[1] = num % 10 + 48;
	}
} /* End of NumToString */

/**
 * SysTickTimer_Init()
 * Desc.	: Initializes the SysTick Timer
 * Param.	: @tickHz - number of interrupts to be triggered per second
 * Return	: None
 * Note		: Counting down to zero asserts the SysTick exception request.
 */
void SysTickTimer_Init(uint32_t tickHz)
{
	/* Calculate reload value */
	uint32_t reloadVal = (SYSTICK_TIM_CLK / tickHz) - 1;

	/* Clear the least significant 24 bits in the SYST_RVR */
	SYST_RVR &= ~0x00FFFFFF;

	/* Load the counter start value into SYST_RVR */
	SYST_RVR |= reloadVal;

	/* Configure SYST_CSR */
	SYST_CSR |= (SYST_CSR_TICKINT | SYST_CSR_CLKSOURCE | SYST_CSR_ENABLE);
		/* TICKINT	: Enable SysTick exception request */
		/* CLKSOURCE: Specify clock source; processor clock source */
		/* ENABLE	: Enable counter */
} /* End of SysTickTimer_Init */

/**
 * SysTick_Handler()
 * Desc.	: Handles the SysTick exception
 * Param.	: None
 * Return	: None
 * Note		: N/A
 */
void SysTick_Handler(void)
{
	RTC_Time_TypeDef currTime;
	RTC_Date_TypeDef currDate;
	char *amPm;

	/* Print current time ----------------------------------------------------*/

	DS1307_GetCurrentTime(&currTime);

	if (currTime.timeFormat != TIME_FORMAT_24HRS)
	{
		/* 12HRS format (e.g., 08:33:45 PM) */
		amPm = (currTime.timeFormat) ? "PM" : "AM";

#ifndef PRINT_ON_LCD
		printf("Current time = %s %s\n", TimeToString(&currTime), amPm);
#else
		LCD_SetCursor(2, 1);
		LCD_PrintString(TimeToString(&currTime));
		LCD_PrintString(" ");
		LCD_PrintString(amPm);
#endif
	}
	else
	{
		/* 24HRS format (e.g., 20:33:45) */
#ifndef PRINT_ON_LCD
		printf("Current time = %s\n", TimeToString(&currTime));
#else
		LCD_SetCursor(2, 1);
		LCD_PrintString(TimeToString(&currTime));
#endif
	}

	/* Print current date in 'MM/DD/YY Day' format ---------------------------*/

	DS1307_GetCurrentDate(&currDate);

#ifndef PRINT_ON_LCD
	printf("Current date = %s <%s>\n", DateToString(&currDate), GetDay(currDate.day));
#else
	LCD_SetCursor(1, 1);
	LCD_PrintString(DateToString(&currDate));
	LCD_PrintString(" ");
	//LCD_PrintChar('<');
	LCD_PrintString(GetDay(currDate.day));
	//LCD_PrintChar('>');
#endif
} /* End of SysTick_Handler */

/**
 * DelayMs()
 * Desc.	: Spinlock delays for @delayInMs milliseconds
 * Param.	: @delayInMs - time to delay in milliseconds
 * Returns	: None
 * Note		: N/A
 */
void DelayMs(uint32_t delayInMs)
{
	for (uint32_t i = 0; i < delayInMs * 1000; i++);
} /* End of DelayMs */
