# Microcontroller Peripheral Drivers


## Introduction

* Developing MCU peripheral (GPIO, I2C, SPI, USART) drivers from scratch on the STM32F407-Discovery board
* Developing applications to test correct functionalities of each peripheral driver



## Development Environment

* Operating system: Ubuntu 22.04 LTS
* Integrated Development Environment (IDE): STM32 CubeIDE Version 1.10.1



## Architecture



<img src="img/mcu-peripheral-driver-development-project-architecture.png" alt="mcu-peripheral-driver-development-project-architecture" width="650">





## List of Files

* Driver Layer
  * Device header: [stm32f407xx.h](./workspace/stm32fxx_drivers/Drivers/Inc/stm32f407xx.h)
  * GPIO driver: [stm32f407xx_gpio.h](./workspace/stm32fxx_drivers/Drivers/Inc/stm32f407xx_gpio.h),  [stm32f407xx_gpio.c](./workspace/Src/stm32fxx_drivers/Drivers/stm32f407xx_gpio.h)
