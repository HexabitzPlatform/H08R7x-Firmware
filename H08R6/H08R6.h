/*
    BitzOS (BOS) V0.1.4 - Copyright (C) 2018 Hexabitz
    All rights reserved

    File Name     : H08R6.h
    Description   : Header file for module H08R6.
										IR Time-if-Flight (ToF) Sensor (ST VL53L0CX)
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H08R6_H
#define H08R6_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H08R6_uart.h"
#include "H08R6_gpio.h"
#include "H08R6_i2c.h"
#include "H08R6_dma.h"


/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H08R6

/* Port-related definitions */
#define	NumOfPorts		6
#define P_PROG 				P2						/* ST factory bootloader UART */

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
#define _Usart6	1

/* Port-UART mapping */
#define P1uart &huart4
#define P2uart &huart2
#define P3uart &huart6
#define P4uart &huart3
#define P5uart &huart1
#define P6uart &huart5

/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT	GPIOA
#define	USART1_RX_PORT	GPIOA
#define	USART1_AF				GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT	GPIOA
#define	USART2_RX_PORT	GPIOA
#define	USART2_AF				GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT	GPIOB
#define	USART3_RX_PORT	GPIOB
#define	USART3_AF				GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT	GPIOA
#define	USART4_RX_PORT	GPIOA
#define	USART4_AF				GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_4
#define	USART5_TX_PORT	GPIOB
#define	USART5_RX_PORT	GPIOB
#define	USART5_AF				GPIO_AF4_USART5

#define	USART6_TX_PIN		GPIO_PIN_4
#define	USART6_RX_PIN		GPIO_PIN_5
#define	USART6_TX_PORT	GPIOA
#define	USART6_RX_PORT	GPIOA
#define	USART6_AF				GPIO_AF5_USART6

/* Module-specific Definitions */
#define _TOF_I2C2_SDA_PORT						GPIOB
#define _TOF_I2C2_SDA_PIN							GPIO_PIN_14
#define _TOF_I2C2_SDA_GPIO_CLK()			__GPIOB_CLK_ENABLE();
#define _TOF_I2C2_SCL_PORT						GPIOB
#define _TOF_I2C2_SCL_PIN							GPIO_PIN_13
#define _TOF_I2C2_SCL_GPIO_CLK()			__GPIOB_CLK_ENABLE();
#define _TOF_INT_PORT									GPIOB
#define _TOF_INT_PIN									GPIO_PIN_2
#define _TOF_INT_GPIO_CLK()						__GPIOB_CLK_ENABLE();
#define _TOF_XSHUT_PORT								GPIOB
#define _TOF_XSHUT_PIN								GPIO_PIN_12
#define _TOF_XSHUT_GPIO_CLK()					__GPIOB_CLK_ENABLE();

/* VL53L0X definition */
#define VL53L0X_ADDR                  0x52

#define vl53l0x_set_xshut_pin()       HAL_GPIO_WritePin(_TOF_XSHUT_PORT, _TOF_XSHUT_PIN, GPIO_PIN_SET) 
#define vl53l0x_reset_xshut_pin()     HAL_GPIO_WritePin(_TOF_XSHUT_PORT, _TOF_XSHUT_PIN, GPIO_PIN_RESET) 

/* Macros define for measurement ranging */
#define EVENT_READY_MEASUREMENT_DATA  ( 1 << 0 ) /* New ready data measurement ranging */
#define STOP_MEASUREMENT_RANGING      0
#define START_MEASUREMENT_RANGING     1
#define UNIT_MEASUREMENT_MM           0
#define UNIT_MEASUREMENT_CM           1
#define UNIT_MEASUREMENT_INCH         2



/* Module_Status Type Definition */
typedef enum
{
  H08R6_OK = 0,
	H08R6_ERR_UnknownMessage,
  H08R6_ERR_WrongColor,
	H08R6_ERR_WrongIntensity,
	H08R6_ERR_Timeout,
	H08R6_ERR_WrongParams,
	H08R6_ERROR = 255
} Module_Status;

/* Indicator LED */
#define _IND_LED_PORT		GPIOA
#define _IND_LED_PIN		GPIO_PIN_11


/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

extern uint8_t vl53l0x_UnitMeasurement;
extern VL53L0X_Dev_t vl53l0x_HandleDevice;
extern EventGroupHandle_t handleNewReadyData;
extern VL53L0X_RangingMeasurementData_t measurementResult;
extern uint8_t startMeasurementRaning;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);


/* -----------------------------------------------------------------------
	|														Message Codes	 														 	|
   -----------------------------------------------------------------------
*/



/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   -----------------------------------------------------------------------
*/

float SampleToF(void);
float ReadToF(uint32_t period);
Module_Status StopToF(void);
Module_Status SetRangeUnit(uint8_t input);
uint8_t GetRangeUnit(void);

/* -----------------------------------------------------------------------
	|															Commands																 	|
   -----------------------------------------------------------------------
*/



#endif /* H08R6_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
