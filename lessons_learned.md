# Lessons Learned



## STMCubeIDE Features

* Data Watch Point
  * Helpful for keeping track of where the selected variable gets updated
* Special Function Registers (SFRs) window (Window $\to$ Show View $\to$ SFRs)
  * Helpful for inspecting or controlling the content of the microcontroller's peripheral registers (e.g., GPIO registers, ADC registers, etc.)
* Code auto completion: `ctrl` + `space`
* View list of functions & global variables: `ctrl` + `o`
  * Quickly find the function and jump to it



## Memory Map of the MCU (STM32F4xx)

* Uses ARM Cortex-M4 processor

* Width of the system bus: 32 bits

  * The processor can product 2^32^ different addresses (4 GB).
  * 0x0000_0000 ~ 0xFFFF_FFFF

* Each MCU peripheral is mapped to a subset of these addressable memory addresses.

  * e.g., When the processor produces 0x4002_0000 on the system bus, it is referring to the GPIO registers.

* Different types of MCUs have different memory maps but the fundamentals are the same. Consult the MCU reference manual for the MCU specific information.

* See also, [https://kyungjae.dev/embedded-systems-programming-arm/memory-map](https://kyungjae.dev/embedded-systems-programming-arm/memory-map).

* Some important base addresses of peripheral registers:

  ```c
  /* Base adddress of AHB1 peripheral registers */
  #define AHB1_BASE	0x40020000U
  
  /* Base adddress of GPIOA peripheral registers */
  #define GPIOA_BASE	0x40020000U
  
  /* Base adddress of RCC engine registers of the MCU */
  #define RCC_BASE	0x40023800U
  
  /* Base adddress of APB1 peripheral registers */
  #define APB1_BASE	0x40000000U
  
  /* Base adddress of FLASH memory */
  #define FLASH_BASE	0x40020000U
  
  /* Base adddress of SRAM1, SRAM2 */
  #define SRAM1_BASE	0x20000000U
  #define SRAM1_SIZE	(112 * 1024)	/* SRAM1: 112 KB, SRAM2: 16 KB */
  #define SRAM2_BASE	((SRAM1_BASE) + (SRAM1_SIZE))
  
  /* Base adddress of ADC1 peripheral registers */
  #define ADC1_BASE	0x40012000U
  ```

  > Make sure to find this information from the MCU reference manual.

  
