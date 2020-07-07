/**
  ******************************************************************************
  * File Name          : H08R6_i2c.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/*
    MODIFIED by Hexabitz for BitzOS (BOS) V0.2.2 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
*/

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


I2C_HandleTypeDef hi2c2;

/*----------------------------------------------------------------------------*/
/* Configure I2C                                                             */
/*----------------------------------------------------------------------------*/

/** I2C Configuration
*/
void MX_I2C_Init(void)
{
  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();   // for HSE and Boot0

  MX_I2C2_Init();
}

//-- Configure indicator LED
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  /* hi2c2.Init.Timing = 0x2010091A; */ /* fast mode: 400 KHz */
  hi2c2.Init.Timing = 0x20303E5D; /* Standard mode: 100 KHz */
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);

    /**Configure Analogue filter
    */
  HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);

    /**Configure Digital filter
    */
  HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);
}

/*-----------------------------------------------------------*/

/**
* @brief Writes the supplied byte buffer to the device
*/
int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count)
{
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t *buff;

  buff = (uint8_t *)malloc(sizeof(uint8_t)*(count + 1));
  buff[0] = index;
  memcpy(&buff[1],pdata, sizeof(uint8_t)*count);

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, buff, (count + 1), HAL_MAX_DELAY);

  free(buff);

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Reads the requested number of bytes from the device
*/
int32_t VL53L0X_read_multi(uint8_t address,  uint8_t index, uint8_t  *pdata, int32_t count)
{
  HAL_StatusTypeDef result = HAL_ERROR;

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, &index, 1, HAL_MAX_DELAY);

  address |= 0x01;
  result |= HAL_I2C_Master_Receive(&hi2c2, address, pdata, count, HAL_MAX_DELAY);

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Writes a single byte to the device
*/
int32_t VL53L0X_write_byte(uint8_t address,  uint8_t index, uint8_t data)
{
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t buff[2];

  buff[0] = index;
  buff[1] = data;

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, buff, 2, HAL_MAX_DELAY);

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Writes a single word (16-bit unsigned) to the device
*/
int32_t VL53L0X_write_word(uint8_t address,  uint8_t index, uint16_t  data)
{
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t buff[3];

  buff[0] = index;
  buff[1] = (data >> 8);
  buff[2] = data & 0xFF;

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, buff, 3, HAL_MAX_DELAY);

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Writes a single dword (32-bit unsigned) to the device
*/
int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t  data)
{
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t buff[5];

  buff[0] = index;
  buff[1] = (data >> 24);
  buff[2] = (data >> 16);
  buff[3] = (data >> 8);
  buff[4] = data & 0xFF;

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, buff, 5, HAL_MAX_DELAY);

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Reads a single byte from the device
*/
int32_t VL53L0X_read_byte(uint8_t address,  uint8_t index, uint8_t  *pdata)
{
  HAL_StatusTypeDef result = HAL_ERROR;

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, &index, 1, HAL_MAX_DELAY);

  address |= 0x01;
  result |= HAL_I2C_Master_Receive(&hi2c2, address, pdata, 1, HAL_MAX_DELAY);

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Reads a single word (16-bit unsigned) from the device
*/
int32_t VL53L0X_read_word(uint8_t address,  uint8_t index, uint16_t *pdata)
{
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t buff[2];

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, &index, 1, HAL_MAX_DELAY);

  address |= 0x01;
  result |= HAL_I2C_Master_Receive(&hi2c2, address, buff, 2, HAL_MAX_DELAY);

  *pdata = buff[0];
  *pdata <<= 8;
  *pdata |= buff[1];

  return (uint32_t)result;
}

/*-----------------------------------------------------------*/

/**
* @brief  Reads a single dword (32-bit unsigned) from the device
*/
int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t buff[4];

  address &= 0xFE;
  result = HAL_I2C_Master_Transmit(&hi2c2, address, &index, 1, HAL_MAX_DELAY);

  address |= 0x01;
  result |= HAL_I2C_Master_Receive(&hi2c2, address, buff, 4, HAL_MAX_DELAY);

  *pdata = buff[0];
  *pdata <<= 8;
  *pdata |= buff[1];
  *pdata <<= 8;
  *pdata |= buff[2];
  *pdata <<= 8;
  *pdata |= buff[3];

  return (uint32_t)result;
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
