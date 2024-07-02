/*
 BitzOS (BOS) V0.3.5 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H08R7.h
 Description   : Header file for module H08R7.
 IR Time-if-Flight (ToF) Sensor (ST VL53L1CX)
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H08R7_H
#define H08R7_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H08R7_MemoryMap.h"
#include "H08R7_uart.h"
#include "H08R7_gpio.h"
#include "H08R7_i2c.h"
#include "H08R7_dma.h"
#include "H08R7_inputs.h"
#include "H08R7_eeprom.h"
#include "Application_VL53L1.h"

/* Exported definitions -------------------------------------------------------*/
#define modulePN    _H08R7

/* Port-related definitions */
#define NumOfPorts    6
#define P_PROG        P2            /* ST factory bootloader UART */
/* Define available ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5
#define _P6

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart4 1
#define _Usart5 1
#define _Usart6 1

/* Port-UART mapping */
#define P1uart &huart4
#define P2uart &huart2
#define P3uart &huart3
#define P4uart &huart1
#define P5uart &huart5
#define P6uart &huart6

/* Port Definitions */
#define USART1_TX_PIN   GPIO_PIN_9
#define USART1_RX_PIN   GPIO_PIN_10
#define USART1_TX_PORT  GPIOA
#define USART1_RX_PORT  GPIOA
#define USART1_AF       GPIO_AF1_USART1

#define USART2_TX_PIN   GPIO_PIN_2
#define USART2_RX_PIN   GPIO_PIN_3
#define USART2_TX_PORT  GPIOA
#define USART2_RX_PORT  GPIOA
#define USART2_AF       GPIO_AF1_USART2

#define USART3_TX_PIN   GPIO_PIN_10
#define USART3_RX_PIN   GPIO_PIN_11
#define USART3_TX_PORT  GPIOB
#define USART3_RX_PORT  GPIOB
#define USART3_AF       GPIO_AF4_USART3

#define USART4_TX_PIN   GPIO_PIN_0
#define USART4_RX_PIN   GPIO_PIN_1
#define USART4_TX_PORT  GPIOA
#define USART4_RX_PORT  GPIOA
#define USART4_AF       GPIO_AF4_USART4

#define USART5_TX_PIN   GPIO_PIN_3
#define USART5_RX_PIN   GPIO_PIN_2
#define USART5_TX_PORT  GPIOD
#define USART5_RX_PORT  GPIOD
#define USART5_AF       GPIO_AF3_USART5

#define USART6_TX_PIN   GPIO_PIN_8
#define USART6_RX_PIN   GPIO_PIN_9
#define USART6_TX_PORT  GPIOB
#define USART6_RX_PORT  GPIOB
#define USART6_AF       GPIO_AF8_USART6

/* Module-specific Definitions */
#define _TOF_I2C2_SDA_PORT            GPIOB
#define _TOF_I2C2_SDA_PIN             GPIO_PIN_14
#define _TOF_I2C2_SDA_GPIO_CLK()      __GPIOB_CLK_ENABLE();
#define _TOF_I2C2_SCL_PORT            GPIOB
#define _TOF_I2C2_SCL_PIN             GPIO_PIN_13
#define _TOF_I2C2_SCL_GPIO_CLK()      __GPIOB_CLK_ENABLE();
#define _TOF_INT_PORT                 GPIOB
#define _TOF_INT_PIN                  GPIO_PIN_1
#define _TOF_INT_GPIO_CLK()           __GPIOB_CLK_ENABLE();
#define _TOF_XSHUT_PORT               GPIOA
#define _TOF_XSHUT_PIN                GPIO_PIN_5
#define _TOF_XSHUT_GPIO_CLK()         __GPIOB_CLK_ENABLE();
#define NUM_MODULE_PARAMS		1
/* VL53L1X definition */
#define MIN_MEMS_PERIOD_MS				100
#define MAX_MEMS_TIMEOUT_MS				0xFFFFFFFF
/* Macros define for measurement ranging */
#define REQ_IDLE                			0
#define REQ_MEASUREMENT_READY         		1
#define SAMPLE_TOF					     	2


#define STREAM_TO_PORT          1
#define STREAM_TO_Terminal      3
#define DEFAULT                 4

/* Module_Status Type Definition */
typedef enum {
	H08R7_OK = 0,
	H08R7_ERR_UnknownMessage,
	H08R7_ERR_WrongColor,
	H08R7_ERR_WrongIntensity,
	H08R7_ERR_Timeout,
	H08R7_ERR_WrongParams,
	H08R7_ERR_BUSY,
	H0BR7_ERR_TERMINATED,
	H08R7_ERROR = 255
} Module_Status;

/* Indicator LED */
#define _IND_LED_PORT   GPIOB
#define _IND_LED_PIN    GPIO_PIN_7

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

/* -----------------------------------------------------------------------
 |                               APIs                                    |
 -----------------------------------------------------------------------
 */
Module_Status Vl53l1xInit(void);
Module_Status Sample_ToF(uint16_t *Distance);
Module_Status StreamDistanceToPort(uint8_t module,uint8_t port,uint32_t Numofsamples,uint32_t timeout);
Module_Status StreamDistanceToTerminal(uint8_t Port ,uint32_t Numofsamples, uint32_t timeout);
Module_Status StreamDistanceToBuffer(uint16_t *buffer, uint32_t Numofsamples,uint32_t timeout);
Module_Status SampletoPort(uint8_t module, uint8_t port);

/* -----------------------------------------------------------------------
 |                             Commands                                  |
 -----------------------------------------------------------------------
 */

#endif /* H08R7_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
