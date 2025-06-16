/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H08R7_i2c.c
 Description: Configures I2C2 instance for module H08R7.
 I2C: Initializes I2C2 with 100 kHz, 7-bit addressing, and analog filter enabled.
 GPIO: Sets up SCL and SDA pins (PB13, PB14) in open-drain alternate function mode.
*/

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include <string.h>
#include <stdio.h>

/* Exported Variables ******************************************************/
I2C_HandleTypeDef hi2c2;

/* Exported Functions ******************************************************/
void MX_I2C_Init(void);
void MX_I2C2_Init(void);

/***************************************************************************/
/* Configure I2C ***********************************************************/
/***************************************************************************/

/** I2C Configuration
 */
void MX_I2C_Init(void)
{
  MX_I2C2_Init();
}

/***************************************************************************/
/* I2C2 init function */
void MX_I2C2_Init(void) {

	/* Initialize I2C2 peripheral */
	HANDLER_I2C.Instance = Instance_I2C;
	HANDLER_I2C.Init.Timing = 0x10B17DB5; // Normal mode (100 kHz)
	HANDLER_I2C.Init.OwnAddress1 = 0; // No specific address required for master mode
	HANDLER_I2C.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; // 7-bit addressing mode
	HANDLER_I2C.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // Disable dual address mode
	HANDLER_I2C.Init.OwnAddress2 = 0; // Not used, set to 0
	HANDLER_I2C.Init.OwnAddress2Masks = I2C_OA2_NOMASK; // No mask for second address
	HANDLER_I2C.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // Disable general call
	HANDLER_I2C.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; // Disable clock stretching
	HAL_I2C_Init(&HANDLER_I2C);

	/** Configure Analogue filter */
	HAL_I2CEx_ConfigAnalogFilter(&HANDLER_I2C, I2C_ANALOGFILTER_ENABLE); // Enable analog filter

	/** Configure Digital filter */
	HAL_I2CEx_ConfigDigitalFilter(&HANDLER_I2C, 0); // Digital filter set to 0 (disabled)
}

/***************************************************************************/
/* I2C2 MspInit function */
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(i2cHandle->Instance==Instance_I2C)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);


    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = I2C2_SCL_PIN|I2C2_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = I2C2_AF;
    HAL_GPIO_Init(I2C2_PORT, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

/***************************************************************************/
/* I2C2 MspDeInit function */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==Instance_I2C)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB13     ------> I2C2_SCL
    PB14     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(I2C2_SCL_PORT, I2C2_SCL_PIN);

    HAL_GPIO_DeInit(I2C2_SDA_PORT, I2C2_SDA_PIN);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
