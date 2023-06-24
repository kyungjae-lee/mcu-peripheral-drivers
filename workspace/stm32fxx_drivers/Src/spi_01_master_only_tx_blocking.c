/*******************************************************************************
 * Filename		: spi_01_master_only_tx_blocking.c
 * Description	: Program to test SPI master's Tx (blocking) functionality
 * 				  (without slave)
 * Author		: Kyungjae Lee
 * History   	: May 27, 2023 - Created file
 * 				  Jun 23, 2023 - Refactored code for consistency
 ******************************************************************************/

/**
 * Pin selection for SPI communication
 *
 * SPI2_SCK  - PB13 (AF5)
 * SPI2_MOSI - PB15 (AF5)
 * SPI2_MISO - PB14 (AF5)
 * SPI2_NSS  - PB12 (AF5)
 */

#include <string.h> 		/* strlen() */
#include "stm32f407xx.h"

/**
 * SPI2_PinsInit()
 * Desc.	: Initializes and configures GPIO pins to be used as SPI2 pins
 * Param.	: None
 * Returns	: None
 * Note		: N/A
 */
void SPI2_PinsInit(void)
{
	GPIO_Handle_TypeDef SPI2Pins;

	/* Zero-out all the fields in the structures (Very important! SPI2Pins
	 * is a local variables whose members may be filled with garbage values before
	 * initialization. These garbage values may set (corrupt) the bit fields that
	 * you did not touch assuming that they will be 0 by default. Do NOT make this
	 * mistake!
	 */
	memset(&SPI2Pins, 0, sizeof(SPI2Pins));

	SPI2Pins.pGPIOx = GPIOB;
	SPI2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_PIN_MODE_ALTFCN;
	SPI2Pins.GPIO_PinConfig.GPIO_PinAltFcnMode = 5;
	SPI2Pins.GPIO_PinConfig.GPIO_PinOutType = GPIO_PIN_OUT_TYPE_PP;
		/* I2C - Open-drain only!, SPI - Push-pull okay! */
	SPI2Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;	/* Optional */
	SPI2Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_PIN_OUT_SPEED_FAST; /* Medium or slow ok as well */

	/* SCLK */
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPI2Pins);

	/* MOSI */
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPI2Pins);

	/* MISO (Not required for this application, save it for other use) */
	//SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	//GPIO_Init(&SPI2Pins);

	/* NSS (Not required for this application, save it for other use) */
	//SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	//GPIO_Init(&SPI2Pins);
} /* End of SPI2_PinsInit */

/**
 * SPI2_Init()
 * Desc.	: Creates an SPI2Handle and initializes SPI2 peripheral parameters
 * Param.	: None
 * Returns	: None
 * Note		: N/A
 */
void SPI2_Init(void)
{
	SPI_Handle_TypeDef SPI2Handle;

	/* Zero-out all the fields in the structures (Very important! SPI2Handle
	 * is a local variables whose members may be filled with garbage values before
	 * initialization. These garbage values may set (corrupt) the bit fields that
	 * you did not touch assuming that they will be 0 by default. Do NOT make this
	 * mistake!
	 */
	memset(&SPI2Handle, 0, sizeof(SPI2Handle));

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SCLKSpeed = SPI_SCLK_SPEED_PRESCALAR_2;	/* Generates 8MHz SCLK */
		/* Min prescalar -> maximum clk speed */
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN; /* SW slave mgmt enabled for NSS pin since NSS is not used */

	SPI_Init(&SPI2Handle);
} /* End of SPI2_Init */

int main(int argc, char *argv[])
{
	char msg[] = "Hello world";		/* Message to transnmit */

	/* Initialize and configure GPIO pins to be used as SPI2 pins */
	SPI2_PinsInit();

	/* Initialize SPI2 peripheral parameters */
	SPI2_Init();
		/* At this point, all the required parameters are loaded into SPIx control registers.
		 * But, this does not mean that SPI2 peripheral is enabled.
		 *
		 * SPI configuration must be completed before it is enabled. When SPI is enabled, it
		 * will be busy communicating with other device(s) and will not allow modifying its
		 * control registers.
		 */

	/* To make NSS signal pulled to high internally and avoid MODF error */
	SPI_SSIConfig(SPI2, ENABLE);

	/* Enable SPI2 peripheral (Set SPI_CR1 bit[6] SPE - Peripheral enabled) */
	SPI_PeriControl(SPI2, ENABLE);

	/* Send data */
	SPI_TxBlocking(SPI2, (uint8_t *)msg, strlen(msg));

	/* Wait until SPI no longer busy */
	while (SPI2->SR & (0x1 << SPI_SR_BSY));
		/* SPI_SR bit[7] - BSY (Busy flag)
		 * 0: SPI (or I2S) not busy
		 * 1: SPI (or I2S) is busy in communication or Tx buffer is not empty
		 * This flag is set and cleared by hardware.
		 */

	/* Disable SPI2 peripheral (Terminate communication) */
	SPI_PeriControl(SPI2, DISABLE);

	while (1);

	return 0;
} /* End of main */
