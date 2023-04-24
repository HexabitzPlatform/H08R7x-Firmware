/*
    BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
    All rights reserved

    File Name     : H08R6.h
    Description   : Header file for module P08R6 / H08R6.
                    IR Time-if-Flight (ToF) Sensor (ST VL53L0CX)
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H08R6_H
#define H08R6_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H08R6_MemoryMap.h"
#include "H08R6_uart.h"
#include "H08R6_gpio.h"
#include "H08R6_i2c.h"
#include "H08R6_dma.h"
#include "H08R6_inputs.h"
#include "H08R6_eeprom.h"
#include "vl53l0x_api.h"


/* Exported definitions -------------------------------------------------------*/
#ifdef P08R6
	#define modulePN    _P08R6
#endif
#ifdef H08R6
	#define modulePN    _H08R6
#endif

/* Port-related definitions */
#ifdef P08R6
	#define NumOfPorts    5
#endif
#ifdef H08R6
	#define NumOfPorts    6
#endif
#define P_PROG        P2            /* ST factory bootloader UART */

/* Define available ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5
#ifdef H08R6
	#define _P6
#endif

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#ifdef H08R6
	#define _Usart4 1
#endif
#define _Usart5 1
#define _Usart6 1

/* Port-UART mapping */
#ifdef P08R6
	#define P1uart &huart5
	#define P2uart &huart2
	#define P3uart &huart6
	#define P4uart &huart3
	#define P5uart &huart1
#endif
#ifdef H08R6
	#define P1uart &huart4
	#define P2uart &huart2
	#define P3uart &huart3
	#define P4uart &huart1
	#define P5uart &huart5
	#define P6uart &huart6
#endif

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
#define USART5_AF       GPIO_AF8_USART5

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
#ifdef P08R6
	#define _TOF_XSHUT_PORT               GPIOB
	#define _TOF_XSHUT_PIN                GPIO_PIN_0
	#define _TOF_XSHUT_GPIO_CLK()         __GPIOB_CLK_ENABLE();
#endif
#ifdef H08R6
	#define _TOF_XSHUT_PORT               GPIOA
	#define _TOF_XSHUT_PIN                GPIO_PIN_5
	#define _TOF_XSHUT_GPIO_CLK()         __GPIOB_CLK_ENABLE();
#endif

#define NUM_MODULE_PARAMS		1

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

#define VL53L0X_DEFAULT_MAX_LOOP      2000

#define REQ_IDLE                			0
#define REQ_SAMPLE		                	1
#define REQ_SAMPLE_CLI                		2
#define REQ_SAMPLE_VERBOSE_CLI				3
#define REQ_SAMPLE_ARR                		4
#define REQ_STREAM_PORT_CLI           		5
#define REQ_STREAM_VERBOSE_PORT_CLI   		6
#define REQ_STREAM_PORT_ARR           		7
#define REQ_STREAM_MEMORY         			8
#define REQ_OUT_RANGE_CLI            	 	9
#define REQ_OUT_RANGE_ARR             		10
#define REQ_TIMEOUT             			11
#define REQ_MEASUREMENT_READY         		12
#define REQ_TIMEOUT_CLI						13
#define REQ_TIMEOUT_VERBOSE_CLI				14
#define REQ_TIMEOUT_MEMORY					15
#define REQ_TIMEOUT_ARR						16

#define TIMERID_TIMEOUT_MEASUREMENT   0xFF

/* declare VL53L0x state */
#define VL53L0x_STATE_FREE            0x00
#define VL53L0x_STATE_RUNNING         0x01
#define VL53L0x_STATE_OUT_RANGE_MAX   0x02
#define VL53L0x_STATE_OUT_RANGE_MIN   0x03

/* Macros define VL53L0CX mode running */
#define VL53L0x_MODE_SINGLE           0x00
#define VL53L0x_MODE_CONTINUOUS       0x01
#define VL53L0x_MODE_CONTINUOUS_TIMED 0x02

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
#define _IND_LED_PORT   GPIOB
#define _IND_LED_PIN    GPIO_PIN_7



/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
#ifdef H08R6
	extern UART_HandleTypeDef huart4;
#endif
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

extern uint8_t h08r6UnitMeasurement;
extern VL53L0X_Dev_t vl53l0x_HandleDevice;
extern EventGroupHandle_t handleNewReadyData;
extern uint8_t startMeasurementRanging;
extern float h08r6MaxRange;
extern uint8_t tofState;


/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
#ifdef H08R6
	extern void MX_USART4_UART_Init(void);
#endif
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);



/* -----------------------------------------------------------------------
  |                               APIs                                    |
   -----------------------------------------------------------------------
*/

float Sample_ToF(void);
void Stream_ToF_Port(uint32_t period, uint32_t timeout, uint8_t port, uint8_t module, bool verbose);
void Stream_ToF_Memory(uint32_t period, uint32_t timeout, float* buffer);
Module_Status Stop_ToF(void);
Module_Status SetRangeUnit(uint8_t input);
uint8_t GetRangeUnit(void);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
/* -----------------------------------------------------------------------
  |                             Commands                                  |
   -----------------------------------------------------------------------
*/



#endif /* H08R6_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
