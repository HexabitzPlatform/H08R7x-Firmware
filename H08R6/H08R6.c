/*
    BitzOS (BOS) V0.1.4 - Copyright (C) 2018 Hexabitz
    All rights reserved

    File Name     : H08R6.c
    Description   : Source code for module H08R6.
                    IR Time-if-Flight (ToF) Sensor (ST VL53L0CX)

    Required MCU resources :

      >> USARTs 1,2,3,4,5,6 for module ports.
      >> I2C2 for the ToF sensor.
      >> GPIOB 2 for ToF interrupt (INT).
      >> GPIOB 12 for ToF shutdown (XSHUT).

*/

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include <stdlib.h>

/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

VL53L0X_Dev_t vl53l0x_HandleDevice;

/* Unit of measurement ranging
 * 0 = mm
 * 1 = cm
 * 2 = inch
 */
uint8_t vl53l0x_UnitMeasurement = UNIT_MEASUREMENT_MM;
EventGroupHandle_t handleNewReadyData = NULL;
VL53L0X_RangingMeasurementData_t measurementResult;
uint8_t startMeasurementRaning = STOP_MEASUREMENT_RANGING;

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void Vl53l0xInit(void);
Module_Status waitEventFinishRanging(void);
float getValueCurrentUnit(uint32_t input);

/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE Vl53l0xTestBoardCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE Vl53l0xSampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE Vl53l0xReadCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE Vl53l0xStopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE Vl53l0xUnitsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure : vl53l0x-test */
const CLI_Command_Definition_t Vl53l0xTestBoardCommandDefinition =
{
  ( const int8_t * ) "vl53l0x-test", /* The command string to type. */
  ( const int8_t * ) "vl53l0x-test:\r\n Command to test PCB board design\r\n\r\n",
  Vl53l0xTestBoardCommand, /* The function to run. */
  0 /* No parameters are expected. */
};

/* CLI command structure : sample */
const CLI_Command_Definition_t Vl53l0xSampleCommandDefinition =
{
  ( const int8_t * ) "sample", /* The command string to type. */
  ( const int8_t * ) "sample:\r\nTake one sample measurement\r\n\r\n",
  Vl53l0xSampleCommand, /* The function to run. */
  0 /* No parameters are expected. */
};

/* CLI command structure : read period */
const CLI_Command_Definition_t Vl53l0xReadCommandDefinition =
{
  ( const int8_t * ) "read", /* The command string to type. */
  ( const int8_t * ) "read:\r\nSet period streaming in msec between measurements\r\n\r\n",
  Vl53l0xReadCommand, /* The function to run. */
  -1 /* one parameter is expected. */
};

/* CLI command structure : stop */
const CLI_Command_Definition_t Vl53l0xStopCommandDefinition =
{
  ( const int8_t * ) "stop", /* The command string to type. */
  ( const int8_t * ) "stop:\r\nStop continuous or timed ranging\r\n\r\n",
  Vl53l0xStopCommand, /* The function to run. */
  0 /* No parameters are expected. */
};

/* CLI command structure : units */
const CLI_Command_Definition_t Vl53l0xUnitsCommandDefinition =
{
  ( const int8_t * ) "units", /* The command string to type. */
  ( const int8_t * ) "units:\r\nSetup the range output unit: mm, cm, inch\r\n\r\n",
  Vl53l0xUnitsCommand, /* The function to run. */
  1 /* one parameter is expected. */
};
/* -----------------------------------------------------------------------
  |                        Private Functions                              |
   -----------------------------------------------------------------------
*/

/* --- H08R6 module initialization.
*/
void Module_Init(void)
{
  /* Peripheral clock enable */


  /* Array ports */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_USART5_UART_Init();
  MX_USART6_UART_Init();

  /* create a event group for measurement ranging */
  handleNewReadyData = xEventGroupCreate();

  /* I2C initialization */
  MX_I2C_Init();

  /* VL53L0X initialization */
  Vl53l0xInit();

  /* LED status */
  IND_ON();
  Delay_us(100);
  IND_OFF();
  Delay_us(100);

}

/*-----------------------------------------------------------*/

/* --- H08R6 message processing task.
*/
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
  Module_Status result = H08R6_OK;

  switch (code)
  {



    default:
      result = H08R6_ERR_UnknownMessage;
      break;
  }

  return result;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
*/
void RegisterModuleCLICommands(void)
{
  FreeRTOS_CLIRegisterCommand( &Vl53l0xTestBoardCommandDefinition);
  FreeRTOS_CLIRegisterCommand( &Vl53l0xSampleCommandDefinition);
  FreeRTOS_CLIRegisterCommand( &Vl53l0xReadCommandDefinition);
  FreeRTOS_CLIRegisterCommand( &Vl53l0xStopCommandDefinition);
  FreeRTOS_CLIRegisterCommand( &Vl53l0xUnitsCommandDefinition);
}

/*-----------------------------------------------------------*/

/* --- Get the port for a given UART.
*/
uint8_t GetPort(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART4)
    return P1;
  else if (huart->Instance == USART2)
    return P2;
  else if (huart->Instance == USART6)
    return P3;
  else if (huart->Instance == USART3)
    return P4;
  else if (huart->Instance == USART1)
    return P5;
  else if (huart->Instance == USART5)
    return P6;

  return 0;
}

/*-----------------------------------------------------------*/

void Vl53l0xInit(void)
{
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t VhvSettings;
  uint8_t PhaseCal;

  vl53l0x_HandleDevice.I2cDevAddr = 0x52;
  vl53l0x_HandleDevice.comms_type = 1; /* Using I2C communication */
  vl53l0x_HandleDevice.comms_speed_khz = 100; /* 100kHz for I2C */

  vl53l0x_set_xshut_pin();
  Delay_us(100);

  if (VL53L0X_ERROR_NONE == status)
  {
    status = VL53L0X_DataInit(&vl53l0x_HandleDevice);
  }

  if (status == VL53L0X_ERROR_NONE)
  {
    /* Device Initialization */
    status = VL53L0X_StaticInit(&vl53l0x_HandleDevice);
  }

  if(status == VL53L0X_ERROR_NONE)
  {
    /* Device Initialization */
    status = VL53L0X_PerformRefSpadManagement(&vl53l0x_HandleDevice,
                                              &refSpadCount,
                                              &isApertureSpads);
  }

  if (status == VL53L0X_ERROR_NONE)
  {
    /* Device Initialization */
    status = VL53L0X_PerformRefCalibration(&vl53l0x_HandleDevice,
                                           &VhvSettings,
                                           &PhaseCal);
  }

  // Enable/Disable Sigma and Signal check
  if (status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetLimitCheckEnable(&vl53l0x_HandleDevice,
                                         VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                         1);
  }

  if (status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetLimitCheckEnable(&vl53l0x_HandleDevice,
                                         VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                         1);
  }

  if (status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetLimitCheckValue(&vl53l0x_HandleDevice,
                                        VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                        (FixPoint1616_t)(0.25*65536));
  }

  if (status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetLimitCheckValue(&vl53l0x_HandleDevice,
                                        VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                        (FixPoint1616_t)(32*65536));
  }

  if (status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&vl53l0x_HandleDevice, 35000);
  }

  /* Setting interrupt on INT pin of VL53L0X */
  if (VL53L0X_ERROR_NONE == status)
  {
    status = VL53L0X_SetGpioConfig(&vl53l0x_HandleDevice,
                                   0,
                                   0,
                                   VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
                                   VL53L0X_INTERRUPTPOLARITY_LOW);
  }

  if (VL53L0X_ERROR_NONE == status)
  {
    status = VL53L0X_SetInterruptThresholds(&vl53l0x_HandleDevice, 0, 60, 200);
  }

  if(VL53L0X_ERROR_NONE == status)
  {
    status = VL53L0X_ClearInterruptMask(&vl53l0x_HandleDevice, 0);
  }
}

/*-----------------------------------------------------------*/

/* --- Wait event finish measurement ranging
*/
Module_Status waitEventFinishRanging(void)
{
  Module_Status result = H08R6_ERR_Timeout;
  EventBits_t tEvBits;

  tEvBits = xEventGroupWaitBits(handleNewReadyData, EVENT_READY_MEASUREMENT_DATA, pdTRUE, pdFALSE, cmd50ms);
  if ((tEvBits & EVENT_READY_MEASUREMENT_DATA) == EVENT_READY_MEASUREMENT_DATA)
  {
    result = H08R6_OK;
  }
  return result;
}

/* --- Get value correct with current unit measurement
*/
float getValueCurrentUnit(uint32_t input)
{
  float distance = 0.0;
  if (UNIT_MEASUREMENT_MM == vl53l0x_UnitMeasurement)
  {
    distance = input;
  }
  else if (UNIT_MEASUREMENT_CM == vl53l0x_UnitMeasurement)
  {
    distance = input / 10;
  }
  else if (UNIT_MEASUREMENT_INCH == vl53l0x_UnitMeasurement)
  {
    distance = input / 25.4; /* 1mm = (1/25.4)″ = 0.03937007874″ */
  }
  else
  {
    /* nothing to do here */
  }
  return distance;
}
/* -----------------------------------------------------------------------
  |                               APIs                                    |
   -----------------------------------------------------------------------
*/

/* --- Takes one sample measurement (triggers ST API Single Ranging)
*/
float SampleToF(void)
{
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  float distance = 0.0;


  if(status == VL53L0X_ERROR_NONE)
  {
    /* no need to do this when we use VL53L0X_PerformSingleRangingMeasurement */
    /* Setup in single ranging mode */
    status = VL53L0X_SetDeviceMode(&vl53l0x_HandleDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
  }

  if (status == VL53L0X_ERROR_NONE)
  {
    VL53L0X_StartMeasurement(&vl53l0x_HandleDevice);
  }

  startMeasurementRaning = START_MEASUREMENT_RANGING;
  if (H08R6_OK == waitEventFinishRanging())
  {
    distance = getValueCurrentUnit(measurementResult.RangeMilliMeter);
  }

  startMeasurementRaning = STOP_MEASUREMENT_RANGING;

  return distance;
}

/* --- steam measurement
*/
float ReadToF(uint32_t period)
{
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  float distance = 0.0;
  int8_t *pcOutputString;
  static const int8_t *pcMessageOK = ( int8_t * ) "Distance (%s): %.2f\r\n";

  if (0 == period) /* Continuous Ranging */
  {
    if(status == VL53L0X_ERROR_NONE)
    {
      /* no need to do this when we use VL53L0X_PerformSingleRangingMeasurement */
      /* Setup in single ranging mode */
      status = VL53L0X_SetDeviceMode(&vl53l0x_HandleDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    }
  }
  else /* Timed Ranging */
  {
    if(status == VL53L0X_ERROR_NONE)
    {
      /* no need to do this when we use VL53L0X_PerformSingleRangingMeasurement */
      /* Setup in single ranging mode */
      status = VL53L0X_SetDeviceMode(&vl53l0x_HandleDevice, VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING);
    }
    if(status == VL53L0X_ERROR_NONE)
    {
      status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(&vl53l0x_HandleDevice, period);
    }
  }

  if (status == VL53L0X_ERROR_NONE)
  {
    VL53L0X_StartMeasurement(&vl53l0x_HandleDevice);
  }

  startMeasurementRaning = START_MEASUREMENT_RANGING;
  while(START_MEASUREMENT_RANGING == startMeasurementRaning)
  {
    if (H08R6_OK == waitEventFinishRanging())
    {
      pcOutputString = FreeRTOS_CLIGetOutputBuffer();
      distance = getValueCurrentUnit(measurementResult.RangeMilliMeter);
      if (UNIT_MEASUREMENT_MM == vl53l0x_UnitMeasurement)
      {
        sprintf( ( char * ) pcOutputString, ( char * ) pcMessageOK, "mm", distance);
      }
      else if (UNIT_MEASUREMENT_CM == vl53l0x_UnitMeasurement)
      {
        sprintf( ( char * ) pcOutputString, ( char * ) pcMessageOK, "cm", distance);
      }
      else if (UNIT_MEASUREMENT_INCH == vl53l0x_UnitMeasurement)
      {
        sprintf( ( char * ) pcOutputString, ( char * ) pcMessageOK, "inch", distance);
      }
      else
      {
        /* nothing to do here */
      }
      writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd50ms, HAL_MAX_DELAY);
    }
  }

  return distance;
}

/* --- Stop measurement
*/
Module_Status StopToF(void)
{
  uint32_t status = 0;
  Module_Status result = H08R6_OK;

  VL53L0X_StopMeasurement(&vl53l0x_HandleDevice);
  VL53L0X_GetStopCompletedStatus(&vl53l0x_HandleDevice, &status);
  if (0 != status)
  {
    result = H08R6_ERROR;
  }
  else
  {
    startMeasurementRaning = STOP_MEASUREMENT_RANGING;
  }

  return result;
}

/* --- Set measurement unit
*/
Module_Status SetRangeUnit(uint8_t input)
{
  Module_Status result = H08R6_OK;

  if (UNIT_MEASUREMENT_MM == input)
  {
    vl53l0x_UnitMeasurement = UNIT_MEASUREMENT_MM;
  }
  else if (UNIT_MEASUREMENT_CM == input)
  {
    vl53l0x_UnitMeasurement = UNIT_MEASUREMENT_CM;
  }
  else if (UNIT_MEASUREMENT_INCH == input)
  {
    vl53l0x_UnitMeasurement = UNIT_MEASUREMENT_INCH;
  }
  else
  {
    result = H08R6_ERROR;
  }

  return result;
}

/* --- Get measurement unit
*/
uint8_t GetRangeUnit(void)
{
  return vl53l0x_UnitMeasurement;
}

/* -----------------------------------------------------------------------
  |                             Commands                                  |
   -----------------------------------------------------------------------
*/
static portBASE_TYPE Vl53l0xTestBoardCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) pcCommandString;
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  static const int8_t *pcPrintTestValue = ( int8_t * ) "Value at address 0x%x = 0x%x\r\n";

  uint8_t t_a_C0 = 0xc0;
  uint8_t t_a_C1 = 0xc1;
  uint8_t t_a_C2 = 0xc2;
  uint8_t t_v_C0;
  uint8_t t_v_C1;
  uint8_t t_v_C2;

  vl53l0x_set_xshut_pin();

  sprintf( ( char * ) pcWriteBuffer, "Reference section 3.2 of VL53L0X datasheet\r\n");
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);

  /* print all datas in output buffer of Terminal */
  VL53L0X_read_byte((uint8_t)VL53L0X_ADDR,  t_a_C0, &t_v_C0);
  sprintf((char *)pcWriteBuffer, (char *)pcPrintTestValue, t_a_C0, t_v_C0);
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);

  /* print all datas in output buffer of Terminal */
  VL53L0X_read_byte((uint8_t)VL53L0X_ADDR,  t_a_C1, &t_v_C1);
  sprintf((char *)pcWriteBuffer, (char *)pcPrintTestValue, t_a_C1, t_v_C1);
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);

  /* print all datas in output buffer of Terminal */
  VL53L0X_read_byte((uint8_t)VL53L0X_ADDR,  t_a_C2, &t_v_C2);
  sprintf((char *)pcWriteBuffer, (char *)pcPrintTestValue, t_a_C2, t_v_C2);
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);

  /* vl53l0x_reset_xshut_pin(); */

  sprintf((char *)pcWriteBuffer, "\r\n");

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

static portBASE_TYPE Vl53l0xSampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
  float distance = 0.0;
  static const int8_t *pcMessageOK = ( int8_t * ) "Distance (%s): %.2f\r\n";

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) pcCommandString;
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  sprintf( ( char * ) pcWriteBuffer, "Command takes one sample measurement (Single Ranging)\r\n");
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);

  distance = SampleToF();
  if (UNIT_MEASUREMENT_MM == vl53l0x_UnitMeasurement)
  {
    sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "mm", distance);
  }
  else if (UNIT_MEASUREMENT_CM == vl53l0x_UnitMeasurement)
  {
    sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "cm", distance);
  }
  else if (UNIT_MEASUREMENT_INCH == vl53l0x_UnitMeasurement)
  {
    sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageOK, "inch", distance);
  }
  else
  {
    /* nothing to do here */
  }
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);

  /* clean terminal output */
  memset((char *) pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
  sprintf((char *)pcWriteBuffer, "\r\n");

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

static portBASE_TYPE Vl53l0xReadCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
  uint32_t period = 0;

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

	/* Obtain the 1st parameter string: The set parameter */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);

  if (NULL == pcParameterString1) /* Continuous Ranging with period = 0 */
  {
    sprintf( ( char * ) pcWriteBuffer, "Ranging operating mode: Continuous Ranging\r\n");
    period = 0;
  }
  else /* Timed Ranging */
  {
    sprintf( ( char * ) pcWriteBuffer, "Ranging operating mode: Timed Ranging\r\n");
    period = atoi( (char *)pcParameterString1);
  }
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
  sprintf( ( char * ) pcWriteBuffer, "\r\nPress enter to stop streaming data on terminal\r\n\r\n");
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
  ReadToF(period);

  /* clean terminal output */
  memset((char *) pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
  sprintf((char *)pcWriteBuffer, "\r\n");

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

static portBASE_TYPE Vl53l0xStopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) pcCommandString;
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  sprintf( ( char * ) pcWriteBuffer, "Stop measurement\r\n");
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);

  StopToF();

  sprintf((char *)pcWriteBuffer, "\r\n");

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

static portBASE_TYPE Vl53l0xUnitsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
  Module_Status result = H08R6_OK;
  int8_t *pcParameterString1;
  portBASE_TYPE xParameterStringLength1 = 0;
  static const int8_t *pcMessageWrongParam = ( int8_t * ) "Wrong parameter!\r\n";

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  /* 1st parameter for naming of uart port: P1 to P6 */
  pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
  if (!strncmp((const char *)pcParameterString1, "mm", 2))
  {
    vl53l0x_UnitMeasurement = UNIT_MEASUREMENT_MM;
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used measurement unit: mm\r\n" );
  }
  else if (!strncmp((const char *)pcParameterString1, "cm", 2))
  {
    vl53l0x_UnitMeasurement = UNIT_MEASUREMENT_CM;
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used measurement unit: cm\r\n" );
  }
  else if (!strncmp((const char *)pcParameterString1, "inch", 4))
  {
    vl53l0x_UnitMeasurement = UNIT_MEASUREMENT_INCH;
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used measurement unit: inch\r\n" );
  }
  else
  {
    result = H08R6_ERR_WrongParams;
  }

  /* Respond to the command */
  if (H08R6_ERR_WrongParams == result)
  {
    strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongParam );
  }

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}



/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
