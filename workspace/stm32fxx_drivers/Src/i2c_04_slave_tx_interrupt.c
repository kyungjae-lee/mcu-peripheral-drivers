/**
 * Filename		: i2c_04_slave_tx_interrupt.c
 * Description	: Program to test I2C slave's Tx (interrupt) functionality
 * Author		: Kyungjae Lee
 * History 		: Jun 18, 2023 - Created file
 */

#include <string.h> 		/* strlen() */
#include <stdio.h> 			/* printf() */
#include "stm32f407xx.h"

#define SLAVE_ADDR			0x69		/* STM32 Discovery board's address */
#define MY_ADDR				SLAVE_ADDR 	/* In this app, STM32 board is slave */

/* Global variables */
I2C_Handle_TypeDef I2C1Handle;
uint8_t txBuff[32] = "Msg from STM32 slave.";
	/* Arduino wire library limits the length of I2C data to be <= 32 bytes */

/**
 * Pin selection for I2C communication
 *
 * I2C1_SCL  - PB6 (AF4)
 * I2C1_SDA  - PB7 (AF4)
 */

/**
 * delay()
 * Desc.	: Spinlock delays the program execution
 * Param.	: None
 * Returns	: None
 * Note		: N/A
 */
void delay(void)
{
	/* Appoximately ~200ms delay when the system clock freq is 16 MHz */
	for (uint32_t i = 0; i < 500000 / 2; i++);
} /* End of delay */

/**
 * I2C1_PinsInit()
 * Desc.	: Initializes and configures GPIO pins to be used as I2C1 pins
 * Param.	: None
 * Returns	: None
 * Note		: N/A
 */
void I2C1_PinsInit(void)
{
	GPIO_Handle_TypeDef I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_MODE_ALTFCN;
	I2CPins.GPIO_PinConfig.GPIO_PinOutType = GPIO_PIN_OUT_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFcnMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_OUT_SPEED_FAST;

	/* SCL */
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);

	/* SDA */
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPins);
} /* End of I2C1_PinsInit */

/**
 * I2C1_Init()
 * Desc.	: Creates an SPI2Handle initializes SPI2 peripheral parameters
 * Param.	: None
 * Returns	: None
 * Note		: N/A
 */
void I2C1_Init(void)
{

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKEnable = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
		/* Since STM32 board is master, I2C_DeviceAddress field does not have
		 * to be configured. However, you can assign some dummy value to it if
		 * you wanted to. When selecting the dummy address value, make sure to
		 * avoid using the reserved addresses defined in the I2C specification.
		 */
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
} /* End of I2C1_Init */


int main(int argc, char *argv[])
{
	printf("Application Running\n");

	/* Initialize I2C pins */
	I2C1_PinsInit();

	/* Configure I2C peripheral */
	I2C1_Init();

	/* I2C IRQ configurations */
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	/* Enable I2C peripheral (PE bit gets set here) */
	I2C_PeriControl(I2C1, ENABLE);

	/* Enable ACK
	 * ACK bit is set and cleared by SW, and cleared by HW when PE=0.
	 * Since PE bit has just been set in the 'I2C_PeriControl()' function,
	 * now you can set the ACK bit. */
	I2C_ManageACK(I2C1, ENABLE);

	while (1);
} /* End of main */

/**
 * I2C1_ER_IRQHandler()
 * Desc.	: Handles I2C error IRQ
 * Param.	: None
 * Return	: None
 * Note		: This function calls 'I2C_ER_IRQHandling()' function which
 * 			  implements the actual error IRQ handling functionality.
 */
void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
} /* End of I2C1_ER_IRQHandler */

/**
 * I2C1_EV_IRQHandler()
 * Desc.	: Handles I2C event IRQ
 * Param.	: None
 * Return	: None
 * Note		: This function calls 'I2C_EV_IRQHandling()' function which
 * 			  implements the actual event IRQ handling functionality.
 */
void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
} /* End of I2C1_EV_IRQHandler */

/**
 * I2C_ApplicationEventCallback()
 * Desc.	: Notifies the application of the event occurred
 * Param.	: @pSPIHandle - pointer to SPI handle structure
 * 			  @appEvent - SPI event occurred
 * Returns	: None
 * Note		: Contents of this function depends on the I2C transactions used
 * 			  in the application. (e.g., Master's write operation results in
 * 			  slave's read.)
 */
void I2C_ApplicationEventCallback(I2C_Handle_TypeDef *pI2CHandle, uint8_t appEvent)
{
	/* Static variables are not allocated in the stack. These variables will
	 * last throughout the duration of the program.
	 */
	static uint8_t cmdCode = 0;
	static uint8_t cnt = 0;

	if (appEvent == I2C_EV_TX)
	{
		/* Master wants to read data. Slave has to send data. */
		if (cmdCode == 0x51)
		{
			/* Send the length information to the master */
			I2C_SlaveTx(pI2CHandle->pI2Cx, strlen((char *)txBuff));
		}
		else if (cmdCode == 0x52)
		{
			/* Send the contents of Tx buffer to the master */
			I2C_SlaveTx(pI2CHandle->pI2Cx, txBuff[cnt++]);
		}
	}
	else if (appEvent == I2C_EV_RX)
	{
		/* Data waits to be read by the slave. Let the slave read it */
		cmdCode = I2C_SlaveRx(pI2CHandle->pI2Cx);
	}
	else if (appEvent == I2C_ERROR_AF)
	{
		/* This happens only during the slave Tx.
		 * Master has sent NACK, meaning that master has no more data to send.
		 */
		cmdCode = 0xFF;	/* Invalidate cmdCode */
		cnt = 0;		/* Reset cnt */
	}
	else if (appEvent == I2C_EV_STOP)
	{
		/* This happens only during slave Rx.
		 * Master has ended the I2C communication with the slave.
		 */
	}
} /* End of I2C_ApplicationEventCallback */
