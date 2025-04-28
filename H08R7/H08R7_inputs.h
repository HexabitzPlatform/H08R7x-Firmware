/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H08R7_inputs.h
 Description   : header file for Bitz digital and analog inputs.
 
*/

/* Includes ****************************************************************/
#include "BOS.h"
#include "stm32g0xx_hal_adc.h"
#include "stm32g0xx_hal_adc_ex.h"
#include "string.h"

/* Port-ADC Definitions */
#define ADC_CH1_PIN   		GPIO_PIN_2
#define ADC_CH2_PIN   		GPIO_PIN_3
#define ADC_CH3_PIN   		GPIO_PIN_4
#define ADC_CH4_PIN   		GPIO_PIN_5
#define ADC12_PORT  		P2
#define ADC34_PORT			P3
#define ADC12_GPIO_PORT  	GPIOA
#define ADC34_GPIO_PORT		GPIOA
#define ADC_CH1_USART   	USART2
#define ADC_CH2_USART   	USART2
#define ADC_CH3_USART   	USART3
#define ADC_CH4_USART   	USART3
#define ADC_CH1_CHANNEL   	ADC_CHANNEL_2
#define ADC_CH2_CHANNEL   	ADC_CHANNEL_3
#define ADC_CH3_CHANNEL   	ADC_CHANNEL_11
#define ADC_CH4_CHANNEL   	ADC_CHANNEL_15

/***************************************************************************/
/* Exported Functions Prototypes *******************************************/
/***************************************************************************/
extern void ReadTempAndVref(float *temp,float *Vref);
extern void ReadADCChannel(uint8_t Port,char *side,float *ADC_Value);
extern void ADCSelectPort(uint8_t ADC_port);
extern void GetReadPercentage(uint8_t port, char *side, float *precentageValue);
extern void ADCDeinitChannel(uint8_t port);

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
