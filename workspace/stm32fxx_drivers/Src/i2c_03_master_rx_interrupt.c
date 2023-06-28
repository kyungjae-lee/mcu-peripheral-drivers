/*******************************************************************************
 * Filename		: i2c_03_master_rx_interrupt.c
 * Description	: Program to test I2C master's Rx (interrupt) functionality
 * Author		: Kyungjae Lee
 * History 		: Jun 15, 2023 - Created file
 ******************************************************************************/

/**
 * Pin selection for I2C communication
 *
 * I2C1_SCL  - PB6 (AF4)
 * I2C1_SDA  - PB7 (AF4)
 */

#include <string.h> 		/* strlen() */
#include <stdio.h> 			/* printf() */
#include "stm32f407xx.h"

#define MASTER_ADDR			0x61
#define SLAVE_ADDR			0x68		/* Check Arduino IDE serial monitor */
#define MY_ADDR				MASTER_ADDR /* STM32 Discovery board is master */

/* Global variables */
I2C_Handle_TypeDef I2C1Handle;
uint8_t rxCmplt = RESET;

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
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_OUT_SPEED_HIGH;

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

/**
 * GPIO_ButtonInit()
 * Desc.	: Initializes a GPIO pin for button
 * Param.	: None
 * Returns	: None
 * Note		: N/A
 */
void GPIO_ButtonInit(void)
{
	GPIO_Handle_TypeDef GPIOBtn;

	/* Zero-out all the fields in the structures (Very important! GPIOLed and GPIOBtn
	 * are local variables whose members may be filled with garbage values before
	 * initialization. These garbage values may set (corrupt) the bit fields that
	 * you did not touch assuming that they will be 0 by default. Do NOT make this
	 * mistake!
	 */
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	/* GPIOBtn configuration */
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_OUT_SPEED_HIGH; /* Doesn't matter */
	//GPIOBtn.GPIO_PinConfig.GPIO_PinOutType = GPIO_PIN_OUT_TYPE_PP;	/* N/A */
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
		/* External pull-down resistor is already present (see the schematic) */
	GPIO_Init(&GPIOBtn);
} /* End of GPIO_ButtonInit */


int main(int argc, char *argv[])
{
	printf("Application Running\n");

	uint8_t cmdCode;
	uint8_t len;

	/* Create an Rx buffer
	 * (The Arduino sketch is written using the Arduino Wire library. The wire
	 * library has limitation on how many bytes can be transferred or received
	 * in single I2C transaction and the limit is 32 bytes. So, don't
	 * send/receive more than 32 bytes in a single I2C transaction. You may want
	 * to split it into multiple I2C transactions in such cases.)
	 */
	uint8_t rxBuff[32];

	/* Initialize GPIO pin for button */
	GPIO_ButtonInit();

	/* Initialize I2C pins */
	I2C1_PinsInit();

	/* Configure I2C peripheral */
	I2C1_Init();

	/* I2C IRQ configurations */
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	/* Enable I2C peripheral (PE bit gets set here) */
	I2C_PeriControl(I2C1, ENABLE);

	/* Enable ACK
	 * ACK bit is set and cleared by SW, and cleared by HW when PE=0.
	 * Since PE bit has just been set in the 'I2C_PeriControl()' function,
	 * now you can set the ACK bit. */
	I2C_ManageACK(I2C1, ENABLE);

	/* Wait for button press */
	while (1)
	{
		/* Wait until button is pressed */
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		/* Introduce debouncing time for button press */
		delay();

		/* Send the command to fetch 1 byte length info */
		cmdCode = 0x51;	/* Command code for asking 1 byte length info */
		while (I2C_MasterTxInterrupt(&I2C1Handle, &cmdCode, 1, SLAVE_ADDR, I2C_REPEATED_START_EN) != I2C_READY);

		/* Fetch and store the length info received from the slave in @len */
		while (I2C_MasterRxInterrupt(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_REPEATED_START_EN) != I2C_READY);

		/* Send the command to receive the complete data from slave */
		cmdCode = 0x52;	/* Command code for reading complete data from slave */
		while (I2C_MasterTxInterrupt(&I2C1Handle, &cmdCode, 1, SLAVE_ADDR, I2C_REPEATED_START_EN) != I2C_READY);

		/* Receive the complete data from slave */
		while (I2C_MasterRxInterrupt(&I2C1Handle, rxBuff, len, SLAVE_ADDR, I2C_REPEATED_START_DI) != I2C_READY);
			/* Since this is the last transaction, disable the repeated start.
			 * After this transaction, you should be able to see the whole
			 * data received in the Rx buffer.
			 */

		/* Reset rxCmplt set by the previous 'I2C_MasterRxInterrupt()' function */
		rxCmplt = RESET;

		/* Wait till Rx operation is finished */
		while (rxCmplt != SET);

		/* Print the data received to the console
		 *
		 * Note: To use printf() function to print the data to console,
		 * 		 a couple of settings have to be done.
		 * 		 See, https://kyungjae.dev/arm-cortex-m3-m4-processor-architecture/using-printf-on-arm-cortex-m3-m4-m7-based-mcus
		 *
		 * 		 You can also use semihosting feature instead.
		 */
		rxBuff[len + 1] = '\0';
			/* Due to the I2C_MasterRxBlocking() mechanism, rxBuff does not contain
			 * the terminating null ('\n') character. Add it here!
			 */
		printf("Data received: %s\n", rxBuff);

		/* Reset rxCmplt for the next operation */
		rxCmplt = RESET;
	}
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
 * Note		: N/A
 */
void I2C_ApplicationEventCallback(I2C_Handle_TypeDef *pI2CHandle, uint8_t appEvent)
{
	/*
	#define I2C_EV_TX_CMPLT			0
	#define I2C_EV_RX_CMPLT			1
	#define I2C_EV_STOP				2
	#define I2C_ERROR_BERR			3
	#define I2C_ERROR_ARLO			4
	#define I2C_ERROR_AF  			5
	#define I2C_ERROR_OVR 			6
	#define I2C_ERROR_TIMEOUT		7
	#define I2C_EV_DATA_REQ			8
	#define I2C_EV_DATA_RCV			9
	*/

	if (appEvent == I2C_EV_TX_CMPLT)
	{
		printf("Tx completed\n");
	}
	else if (appEvent == I2C_EV_RX_CMPLT)
	{
		printf("Rx completed\n");
		rxCmplt = SET;
	}
	else if (appEvent == I2C_ERROR_AF)
	{
		printf("Error: ACK failure\n");

		/* Master ACK failure occurs when slave fails to send ACK for the byte
		 * sent from the master.
		 * If this is the case, there's no reason to keep the communication
		 * going. So close the communication.
		 */
		I2C_CloseTx(pI2CHandle);

		/* Generate STOP condition to release the bus */
		I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);

		/* Hang in infinite loop */
		while (1);
	}
} /* End of I2C_ApplicationEventCallback */
