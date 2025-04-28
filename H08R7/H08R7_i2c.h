/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name          : H08R7_i2c.h
 Description        : This file contains all the functions prototypes for
 the i2c

 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ****************************************************************/
#include "stm32g0xx_hal.h"

/* Exported Variables ******************************************************/
extern I2C_HandleTypeDef hi2c2;

#define HANDLER_I2C				hi2c2
#define Instance_I2C			I2C2
#define I2C2_SCL_PIN			GPIO_PIN_13
#define I2C2_SCL_PORT			GPIOB
#define I2C2_SDA_PIN			GPIO_PIN_14
#define I2C2_SDA_PORT			GPIOB
#define I2C2_PORT				GPIOB
#define I2C2_AF					GPIO_AF6_I2C2
#define ToF_SENSOR_I2C_ADDRESS 	0x52

/* Exported Functions ******************************************************/
extern  void MX_I2C_Init(void);
extern void MX_I2C2_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__i2c_H */

 /***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
