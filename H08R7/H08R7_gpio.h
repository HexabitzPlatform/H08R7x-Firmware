/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H08R7_gpio.h
 Description   : Header file contains all the functions prototypes for
 the GPIO .

 */


/* Define to prevent recursive inclusion ***********************************/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ****************************************************************/
#include "stm32g0xx_hal.h"


#define TOF_XSHUT_Pin          GPIO_PIN_5
#define TOF_XSHUT_GPIO_Port    GPIOA
#define TOF_INT_Pin            GPIO_PIN_1
#define TOF_INT_GPIO_Port      GPIOB

extern void GPIO_Init(void);
extern void IND_LED_Init(void);
extern void TOF_GPIO_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__gpio_H */

 /***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
