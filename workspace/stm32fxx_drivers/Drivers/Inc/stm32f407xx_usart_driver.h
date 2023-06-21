/*******************************************************************************
 * Filename		: stm32f407xx_usart_driver.h
 * Description	: STM32F407xx MCU specific USART driver header file
 * Author		: Kyungjae Lee
 * History		: Jun 20, 2023 - Created file
 ******************************************************************************/

#ifndef STM32F407XX_USART_DRIVER_H
#define STM32F407XX_USART_DRIVER_H

#include "stm32f407xx.h"

/*******************************************************************************
 * USART peripheral structures
 ******************************************************************************/

/* USART peripheral configuration structure */
typedef struct
{
	uint8_t USART_Mode;				/* Available values @USART_Mode 		 */
	uint32_t USART_Baud;			/* Available values @USART_Baud 		 */
	uint8_t USART_NumOfStopBits;	/* Available values @USART_NumOfStopBits */
	uint8_t USART_WordLength;		/* Available values @USART_WordLength 	 */
	uint8_t USART_ParityControl;	/* Available values @USART_ParityControl */
	uint8_t USART_HWFlowControl;	/* Available values @USART_HWFlowControl */
} USART_Config_TypeDef;

/* USART peripheral handle structure */
typedef struct
{
	USART_TypeDef 			*pUSARTx;	/* Base address of USARTx(x:1,2,3) */
	USART_Config_TypeDef 	USART_Config;
} USART_Handle_TypeDef;

/**
 * @USART_Mode
 * Note: Check TE and RE bits in USART_CR1 register.
 */
#define USART_MODE_TX			0	/* (CR1) TE=1, RE=0 */
#define USART_MODE_RX			1	/* (CR1) TE=0, RE=1 */
#define USART_MODE_TXRX			2	/* (CR1) TE=1, RE=1 */

/**
 * @USART_Baud
 * Note: Baudrates are in 'bits per second (bps)' and can be found in the
 * 		 MCU reference manual.
 * 		 Check USART_BRR register.
 */
#define USART_STD_BAUD_1200		1200
#define USART_STD_BAUD_2400		2400
#define USART_STD_BAUD_9600		9600
#define USART_STD_BAUD_19200	19200
#define USART_STD_BAUD_38400	38400
#define USART_STD_BAUD_56700	56700
#define USART_STD_BAUD_115200	115200
#define USART_STD_BAUD_230400	230400
#define USART_STD_BAUD_460800	460800
#define USART_STD_BAUD_921600	921600
#define USART_STD_BAUD_2M		2000000
#define USART_STD_BAUD_3M		3000000

/**
 * @USART_NumOfStopBits
 * Note: Check STOP bit field in USART_CR2 register.
 */
#define USART_STOPBITS_1  		0
#define USART_STOPBITS_0_5		1
#define USART_STOPBITS_2		2
#define USART_STOPBITS_1_5		3

/**
 * @USART_WordLength
 * Note: Check M bit in USART_CR1 register.
 */
#define USART_WORDLEN_8BITS		0
#define USART_WORDLEN_9BITS		1

/**
 * @USART_ParityControl
 * Note: Check PCE and PS bits in USART_CR1 register.
 */
#define USART_PARITY_DISABLE	0
#define USART_PARITY_ODD		1
#define USART_PARITY_EVEN		2

/**
 * @USART_HWFlowControl
 * Note: Check CTSE and RTSE bits in USART_CR3 register.
 */
#define USART_HW_FLOW_CTRL_NONE		0
#define USART_HW_FLOW_CTRL_CTS		1
#define USART_HW_FLOW_CTRL_RTS		2
#define USART_HW_FLOW_CTRL_CTS_RTS	3



/*******************************************************************************
 * APIs supported by the USART driver
 * (See function definitions for more information)
 ******************************************************************************/

/* Peripheral clock setup */
void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t state);

/**
 * Init and De-init
 */
void USART_Init(USART_Handle_TypeDef *pUSARTHandle);
void USART_DeInit(USART_TypeDef *pUSARTx);


/**
 * Data send and receive
 */
void USART_TxBlocking(USART_Handle_TypeDef *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
void USART_RxBlocking(USART_Handle_TypeDef *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len);
uint8_t USART_TxInterrupt(USART_Handle_TypeDef *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t USART_RxInterrupt(USART_Handle_TypeDef *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len);


/**
 * IRQ configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t irqNumber, uint8_t state);
void USART_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void USART_IRQHandling(USART_Handle_TypeDef *pUSARTHandle);


/**
 * Other peripheral control APIs
 */
void USART_PeriControl(USART_TypeDef *pUSARTx, uint8_t state);
void USART_SetBaudRate(USART_TypeDef *pUSARTx, uint32_t baudrate);


#endif /* STM32F407XX_USART_DRIVER_H */
