/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H08R7.h
 Description   : Header file for module H08R7.
 IR Time-if-Flight (ToF) Sensor (ST VL53L1CX)
 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef H08R7_H
#define H08R7_H

/* Includes ****************************************************************/
#include "BOS.h"
#include "H08R7_MemoryMap.h"
#include "H08R7_uart.h"
#include "H08R7_gpio.h"
#include "H08R7_i2c.h"
#include "H08R7_dma.h"
#include "H08R7_inputs.h"
#include "H08R7_eeprom.h"
#include "Application_VL53L1.h"

/* Exported Macros *********************************************************/
#define	MODULE_PN		_H08R7

/* Port-related definitions */
#define	NUM_OF_PORTS	6
#define P_PROG        	P2            /* ST factory bootloader UART */

/* Define available ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5
#define _P6

/* Define Available USARTs */
#define _USART1
#define _USART2
#define _USART3
#define _USART4
#define _USART5
#define _USART6

/* Port-UART mapping */
#define UART_P1 &huart4
#define UART_P2 &huart2
#define UART_P3 &huart3
#define UART_P4 &huart1
#define UART_P5 &huart5
#define UART_P6 &huart6

/* Module-specific Hardware Definitions ************************************/
/* Port Definitions */
#define USART1_TX_PIN   GPIO_PIN_9
#define USART1_RX_PIN   GPIO_PIN_10
#define USART1_TX_PORT  GPIOA
#define USART1_RX_PORT  GPIOA
#define USART1_PORT		GPIOA
#define USART1_AF       GPIO_AF1_USART1

#define USART2_TX_PIN   GPIO_PIN_2
#define USART2_RX_PIN   GPIO_PIN_3
#define USART2_TX_PORT  GPIOA
#define USART2_RX_PORT  GPIOA
#define USART2_PORT		GPIOA
#define USART2_AF       GPIO_AF1_USART2

#define USART3_TX_PIN   GPIO_PIN_10
#define USART3_RX_PIN   GPIO_PIN_11
#define USART3_TX_PORT  GPIOB
#define USART3_RX_PORT  GPIOB
#define USART3_PORT		GPIOB
#define USART3_AF       GPIO_AF4_USART3

#define USART4_TX_PIN   GPIO_PIN_0
#define USART4_RX_PIN   GPIO_PIN_1
#define USART4_TX_PORT  GPIOA
#define USART4_RX_PORT  GPIOA
#define USART4_PORT		GPIOA
#define USART4_AF       GPIO_AF4_USART4

#define USART5_TX_PIN   GPIO_PIN_3
#define USART5_RX_PIN   GPIO_PIN_2
#define USART5_TX_PORT  GPIOD
#define USART5_RX_PORT  GPIOD
#define USART5_PORT		GPIOD
#define USART5_AF       GPIO_AF3_USART5

#define USART6_TX_PIN   GPIO_PIN_8
#define USART6_RX_PIN   GPIO_PIN_9
#define USART6_TX_PORT  GPIOB
#define USART6_RX_PORT  GPIOB
#define USART6_PORT		GPIOB
#define USART6_AF       GPIO_AF8_USART6

/* I2C Pin Definition */
#define I2C2_SCL_PIN	GPIO_PIN_13
#define I2C2_SCL_PORT	GPIOB
#define I2C2_SDA_PIN	GPIO_PIN_14
#define I2C2_SDA_PORT	GPIOB
#define I2C2_PORT		GPIOB
#define I2C2_AF			GPIO_AF6_I2C2

#define HANDLER_I2C		hi2c2
#define Instance_I2C	I2C2

/* GPIO Pin Definition */
#define TOF_XSHUT_Pin        GPIO_PIN_5
#define TOF_XSHUT_GPIO_Port  GPIOA
#define TOF_INT_Pin          GPIO_PIN_1
#define TOF_INT_GPIO_Port    GPIOB

/* Indicator LED */
#define _IND_LED_PORT        GPIOB
#define _IND_LED_PIN         GPIO_PIN_7

/* Module-specific Macro Definitions ***************************************/
#define ToF_SENSOR_I2C_ADDRESS 	    0x52

#define NUM_MODULE_PARAMS			1
#define MIN_MEMS_PERIOD_MS			100
#define MAX_MEMS_TIMEOUT_MS			0xFFFFFFFF

/* Macros define for measurement ranging */
#define REQ_IDLE                	0
#define REQ_MEASUREMENT_READY      	1
#define SAMPLE_TOF					2

/* Macros definitions */
#define STREAM_MODE_TO_PORT      	1
#define STREAM_MODE_TO_TERMINAL  	2

#define _WAITFORINT()				__WFI()

/* Module-specific Type Definition *****************************************/
/* Module-status Type Definition */
typedef enum {
	H08R7_OK = 0,
	H08R7_ERR_UNKNOWNMESSAGE,
	H08R7_ERR_WRONGPARAMS,
	H0BR7_ERR_TERMINATED,
	H08R7_ERROR = 255
} Module_Status;

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status SampleTOF(uint16_t *Distance);

Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort);
Module_Status StreamToPort(uint8_t dstModule,uint8_t dstPort,uint32_t numOfSamples,uint32_t streamTimeout);
Module_Status StreamToTerminal(uint8_t dstPort,uint32_t numOfSamples,uint32_t streamTimeout);

#endif /* H08R7_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
