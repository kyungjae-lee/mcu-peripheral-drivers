/*******************************************************************************
 * Filename		: ds1307.c
 * Description	:
 * Author		: Kyungjae Lee
 * History 		: Jun 25, 2023 - Created file
 ******************************************************************************/

#include <string.h> 	/* memset() */
#include <stdint.h>
#include "ds1307.h"

/* Private function prototypes */
static void DS1307_I2CPinConfig(void);
static void DS1307_I2CConfig(void);
static void DS1307_Write(uint8_t value, uint8_t regAddr);
static uint8_t DS1307_Read(uint8_t regAddr);

/* Global variables */
I2C_Handle_TypeDef gDS1307I2CHandle;

/**
 * DS1307_Init()
 * Desc.	: Initializes DS1307 RTC module
 * Param.	: None
 * Return	: 0 if setting DS1307_SEC CH bit to 0 was successful (init success),
 * 			  1 otherwise (init fail)
 * Note		: N/A
 */
uint8_t DS1307_Init(void)
{
	/* 1. Initialize the I2C pins */
	DS1307_I2CPinConfig();

	/* 2. Initialize the I2C peripheral */
	DS1307_I2CConfig();

	/* 3. Enable the I2C peripheral */
	I2C_PeriControl(DS1307_I2C, ENABLE);

	/* 4. Set Clock Halt (CH) bit to 0 to initiate time keeping
	 * Note: At power on, the time keeping will not function until the CH bit
	 * 		 becomes 0. So, to initiate the time keeping functionality the
  	 *		 master needs to write 0 to the slave's CH bit.
  	 *		 For data write logic, see the "I2C Data Bus" section of DS1307
  	 *		 reference manual.
  	 */
	DS1307_Write(0x00, DS1307_SEC);

	/* 5. Read back Clock Halt (CH) bit
	 * Note: For data read logic, see the "I2C Data Bus" section of DS1307
	 * 		 reference manual.
	 */
	uint8_t clockState = DS1307_Read(DS1307_SEC);

	return ((clockState >> 7) & 0x1);
}

/**
 * DS1307_SetCurrentTime()
 * Desc.	:
 * Param.	:
 * Return	: None
 * Note		: N/A
 */
void DS1307_SetCurrentTime(RTC_Time_TypeDef *rtcTime)
{

} /* End of DS1307_SetCurrentTime */

/**
 * DS1307_GetCurrentTime()
 * Desc.	:
 * Param.	:
 * Return	: None
 * Note		: N/A
 */
void DS1307_GetCurrentTime(RTC_Time_TypeDef *rtcTime)
{

} /* End of DS1307_GetCurrentTime */

/**
 * DS1307_SetCurrentDate()
 * Desc.	:
 * Param.	:
 * Return	: None
 * Note		: N/A
 */
void DS1307_SetCurrentDate(RTC_Time_TypeDef *rtcTime)
{

} /* End of DS1307_SetCurrentDate */

/**
 * DS1307_GetCurrentDate()
 * Desc.	:
 * Param.	:
 * Return	: None
 * Note		: N/A
 */
void DS1307_GetCurrentDate(RTC_Time_TypeDef *rtcTime)
{

} /* End of DS1307_GetCurrentDate */


/*******************************************************************************
 * Private functions
 ******************************************************************************/

/**
 * DS1307_I2CPinConfig()
 * Desc.	:
 * Param.	:
 * Return	: None
 * Note		: N/A
 */
static void DS1307_I2CPinConfig(void)
{
	GPIO_Handle_TypeDef i2cSda, i2cScl;

	/* Zero-out all the fields in the structures (Very important! i2cSda, i2cScl
	 * are local variables whose members may be filled with garbage values before
	 * initialization. These garbage values may set (corrupt) the bit fields that
	 * you did not touch assuming that they will be 0 by default. Do NOT make this
	 * mistake!
	 */
	memset(&i2cSda, 0, sizeof(i2cSda));
	memset(&i2cScl, 0, sizeof(i2cScl));

	/*
	 * I2C1_SCL: PB6
	 * I2C1_SDA: PB7
	 */

	i2cSda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2cSda.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_MODE_ALTFCN;
	i2cSda.GPIO_PinConfig.GPIO_PinAltFcnMode = 4;
	i2cSda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_PIN_SDA;
	i2cSda.GPIO_PinConfig.GPIO_PinOutType= GPIO_PIN_OUT_TYPE_OD;
	i2cSda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2cSda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_OUT_SPEED_FAST;

	GPIO_Init(&i2cSda);

	i2cScl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2cScl.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_MODE_ALTFCN;
	i2cScl.GPIO_PinConfig.GPIO_PinAltFcnMode = 4;
	i2cScl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_PIN_SCL;
	i2cScl.GPIO_PinConfig.GPIO_PinOutType= GPIO_PIN_OUT_TYPE_OD;
	i2cScl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2cScl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_OUT_SPEED_FAST;

	GPIO_Init(&i2cScl);
} /* End of DS1307_I2CPinConfig */

/**
 * DS1307_I2CConfig()
 * Desc.	: Configures and initializes DS1307 I2C peripheral
 * Param.	: None
 * Return	: None
 * Note		: N/A
 */
static void DS1307_I2CConfig(void)
{
	gDS1307I2CHandle.pI2Cx = DS1307_I2C;
	gDS1307I2CHandle.I2C_Config.I2C_ACKEnable = I2C_ACK_ENABLE;
	gDS1307I2CHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;
	I2C_Init(&gDS1307I2CHandle);
} /* End of DS1307_I2CConfig */

/**
 * DS1307_Write()
 * Desc.	: Writes @value to @regAddr
 * Param.	: @value - value to write to @regAddr
 *            @regAddr - DS1307 register address to write @value to
 * Return	: None
 * Note		: N/A
 */
static void DS1307_Write(uint8_t value, uint8_t regAddr)
{
	uint8_t tx[2];
	tx[0] = regAddr;
	tx[1] = value;
	I2C_MasterTxBlocking(&gDS1307I2CHandle, tx, 2, DS1307_I2C_ADDR, 0);

} /* End of DS1307_Write */

/**
 * DS1307_Read()
 * Desc.	: Writes @value to @regAddr
 * Param.   : @regAddr - DS1307 register address to read from
 * Return	: 1-byte data received from the slave (DS1307 RTC module)
 * Note		: N/A
 */
static uint8_t DS1307_Read(uint8_t regAddr)
{
	uint8_t rxData;
	I2C_MasterTxBlocking(&gDS1307I2CHandle, &regAddr, 1, DS1307_I2C_ADDR, 0);
	I2C_MasterRxBlocking(&gDS1307I2CHandle, &rxData, 1, DS1307_I2C_ADDR, 0);

	return rxData;
} /* End of DS1307_Read */
