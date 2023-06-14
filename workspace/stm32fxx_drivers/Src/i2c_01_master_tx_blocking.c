/**
 * Filename		: i2c_01_master_tx_blocking.c
 * Description	: Program to test I2C master's Tx functionality
 * Author		: Kyungjae Lee
 * History 		: Jun 12, 2023 - Created file
 */

#include <string.h> 		/* strlen() */
#include "stm32f407xx.h"

#define DUMMY_ADDR			0x61
#define SLAVE_ADDR			0x68	/* Check Arduino IDE serial monitor */

I2C_Handle_TypeDef I2C1Handle;

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
}

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
}

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
	I2C1Handle.I2C_Config.I2C_DeviceAddress = DUMMY_ADDR;
		/* Since STM32 board is master, I2C_DeviceAddress field does not have
		 * to be configured. However, you can assign some dummy value to it if
		 * you wanted to. When selecting the dummy address value, make sure to
		 * avoid using the reserved addresses defined in the I2C specification.
		 */
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

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
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_OUT_SPEED_FAST; /* Doesn't matter */
	//GPIOBtn.GPIO_PinConfig.GPIO_PinOutType = GPIO_PIN_OUT_TYPE_PP;	/* N/A */
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
		/* External pull-down resistor is already present (see the schematic) */
	GPIO_Init(&GPIOBtn);
}

int main(int argc, char *argv[])
{
	/* Create data to send
	 * (The Arduino sketch is written using the Arduino Wire library. The wire
	 * library has limitation on how many bytes can be transferred or received
	 * in single I2C transaction and the limit is 32 bytes. So, don't
	 * send/receive more than 32 bytes in a single I2C transaction. You may want
	 * to split it into multiple I2C transactions in such cases.)
	 */
	uint8_t data[] = "I2C master Tx testing\n";

	/* Initialize GPIO pin for button */
	GPIO_ButtonInit();

	/* Initialize I2C pins */
	I2C1_PinsInit();

	/* Configure I2C peripheral */
	I2C1_Init();

	/* Enable I2C peripheral */
	I2C_PeriControl(I2C1, ENABLE);

	/* Wait for button press */
	while (1)
	{
		/* Wait until button is pressed */
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		/* Introduce debouncing time for button press */
		delay();

		/* Send data to slave */
		I2C_MasterTx(&I2C1Handle, data, strlen((char *)data), SLAVE_ADDR);
	}
}
