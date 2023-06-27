/*******************************************************************************
 * Filename		: rtc_lcd.c
 * Description	: Application to read time and date information from DS1207 RTC
 *				  chip and display it on the 16x2 Character LCD
 * Author		: Kyungjae Lee
 * History 		: Jun 25, 2023 - Created file
 ******************************************************************************/

#include <string.h> 		/* strlen() */
#include <stdio.h> 			/* printf() */
#include "stm32f407xx.h"
#include "ds1307.h"

/* Function prototypes */
char* TimeToString(RTC_Time_TypeDef *rtcTime);
char* DateToString(RTC_Date_TypeDef *rtcDate);
char* GetDay(uint8_t dayCode);
void NumToString(uint8_t num, char *buf);

int main(int argc, char *argv[])
{
	RTC_Time_TypeDef currTime;
	RTC_Date_TypeDef currDate;
	char *amPm;

	printf("RTC test application running...\n");

	if (DS1307_Init())
	{
		/* RTC initialization was unsuccessful */
		printf("RTC init failed\n");

		while (1);
	}

	currDate.day = FRIDAY;
	currDate.date = 25;
	currDate.month = 6;
	currDate.year = 23;		/* Only the last two digits of the year */

	currTime.hours = 14;
	currTime.minutes = 25;
	currTime.seconds = 41;
	currTime.timeFormat = TIME_FORMAT_24HRS;

	DS1307_SetCurrentDate(&currDate);
	DS1307_SetCurrentTime(&currTime);

	DS1307_GetCurrentDate(&currDate);
	DS1307_GetCurrentTime(&currTime);

	/* Print current time *****************************************************/

	if (currTime.timeFormat != TIME_FORMAT_24HRS)
	{
		/* 12HRS format (e.g., 08:33:45 PM) */
		amPm = (currTime.timeFormat) ? "PM" : "AM";
		printf("Current time = %s %s\n", TimeToString(&currTime), amPm);
	}
	else
	{
		/* 24HRS format (e.g., 20:33:45) */
		printf("Current time = %s\n", TimeToString(&currTime));
	}

	/* Print current date in 'MM/DD/YY <Day>' format **************************/

	printf("Current date = %s <%s>\n", DateToString(&currDate), GetDay(currDate.day));

	return 0;
} /* End of main */

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
	char *days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

	return days[dayCode];
} /* End of GetDay */

void NumToString(uint8_t num, char *buf)
{
	if (num < 10)
	{
		buf[0] = '0';
		buf[1] = num + '0';
	}
	else if (10 <= num && num < 99)
	{
		buf[0] = num / 10 + '0';
		buf[1] = num % 10 + '0';
	}
}
