/*
    BitzOS (BOS) V0.1.5 - Copyright (C) 2017-2018 Hexabitz
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
uint8_t h08r6UnitMeasurement = UNIT_MEASUREMENT_MM;
EventGroupHandle_t handleNewReadyData = NULL;
uint8_t startMeasurementRanging = STOP_MEASUREMENT_RANGING;

/* Private variables ---------------------------------------------------------*/
int32_t offsetCalibration = 0;
uint8_t h08r6StatesVl53l0x = VL53L0x_STATE_FREE;
float h08r6MinRange = 0.0;
float h08r6MaxRange = 8000.0;
float h08r6BufStreamMem = 0;

static float distance = 0.0;

/* Private function prototypes -----------------------------------------------*/
static void Vl53l0xInit(void);
static VL53L0X_Error SettingModeMeasurement(uint8_t mode, uint32_t period, uint32_t timeout);
static Module_Status WaitForMeasurement(void);
static float GetMeasurementResult(void);
static float ConvertCurrentUnit(float distance);
static void SendMeasurementResult(uint8_t request, float distance, uint8_t module, uint8_t port, float *buffer);
static void CheckPressingEnterKey(void);

/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE demoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE Vl53l0xSampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE Vl53l0xStreamCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE Vl53l0xStopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE Vl53l0xUnitsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE Vl53l0xMaxCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure : demo */
const CLI_Command_Definition_t demoCommandDefinition =
{
	( const int8_t * ) "demo", /* The command string to type. */
	( const int8_t * ) "(H08R6) demo:\r\n Run a demo program to test module functionality\r\n\r\n",
	demoCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : sample */
const CLI_Command_Definition_t Vl53l0xSampleCommandDefinition =
{
  ( const int8_t * ) "sample", /* The command string to type. */
  ( const int8_t * ) "(H08R6) sample:\r\nTake one sample measurement\r\n\r\n",
  Vl53l0xSampleCommand, /* The function to run. */
  0 /* No parameters are expected. */
};

/* CLI command structure : stream */
const CLI_Command_Definition_t Vl53l0xStreamCommandDefinition =
{
  ( const int8_t * ) "stream", /* The command string to type. */
		( const int8_t * ) "(H08R6) stream:\r\nStream measurements to the CLI with this syntax:\n\r\tstream period(in ms) timeout(in ms)\n\rOr to a specific port \
in a specific module with this syntax:\r\n\tstream period timeout port(p1..px) module\r\n\r\n",
  Vl53l0xStreamCommand, /* The function to run. */
  -1 /* Multiple parameters are expected. */
};

/* CLI command structure : stop */
const CLI_Command_Definition_t Vl53l0xStopCommandDefinition =
{
  ( const int8_t * ) "stop", /* The command string to type. */
  ( const int8_t * ) "(H08R6) stop:\r\nStop continuous or timed ranging\r\n\r\n",
  Vl53l0xStopCommand, /* The function to run. */
  0 /* No parameters are expected. */
};

/* CLI command structure : units */
const CLI_Command_Definition_t Vl53l0xUnitsCommandDefinition =
{
  ( const int8_t * ) "units", /* The command string to type. */
  ( const int8_t * ) "(H08R6) units:\r\nSetup the range output unit: mm, cm, inch\r\n\r\n",
  Vl53l0xUnitsCommand, /* The function to run. */
  1 /* one parameter is expected. */
};

/* CLI command structure : max */
const CLI_Command_Definition_t Vl53l0xMaxCommandDefinition =
{
  ( const int8_t * ) "max", /* The command string to type. */
  ( const int8_t * ) "(H08R6) max:\r\nGet sample measurement of maximum distance\r\n\r\n",
  Vl53l0xMaxCommand, /* The function to run. */
  0 /* one parameter is expected. */
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

}

/*-----------------------------------------------------------*/

/* --- H08R6 message processing task.
*/
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
  Module_Status result = H08R6_OK;
  uint32_t period;
  uint32_t timeout;

  switch (code)
  {
    case CODE_H08R6_GET_INFO:
      break;
    case CODE_H08R6_SAMPLE:
      Sample_ToF(port, dst);
      break;
    case CODE_H08R6_STREAM_PORT:
      memcpy(&period, &messageParams[0], 4);
      memcpy(&timeout, &messageParams[4], 4);
      Stream_ToF_Port(period, timeout, port, dst);
      break;
    case CODE_H08R6_STREAM_MEM:
      memcpy(&period, &messageParams[0], 4);
      memcpy(&timeout, &messageParams[4], 4);
      Stream_ToF_Memory(period, timeout, &h08r6BufStreamMem);
      break;
    case CODE_H08R6_RESULT_MEASUREMENT:
      break;
    case CODE_H08R6_STOP_RANGING:
      Stop_ToF();
      break;
    case CODE_H08R6_SET_UNIT:
      SetRangeUnit(cMessage[port-1][4]);
      break;
    case CODE_H08R6_GET_UNIT:
      messageParams[0] = GetRangeUnit();
      SendMessageFromPort(port, myID, dst, CODE_H08R6_RESPOND_GET_UNIT, 1);
      break;
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
  FreeRTOS_CLIRegisterCommand( &demoCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &Vl53l0xSampleCommandDefinition);
  FreeRTOS_CLIRegisterCommand( &Vl53l0xStreamCommandDefinition);
  FreeRTOS_CLIRegisterCommand( &Vl53l0xStopCommandDefinition);
  FreeRTOS_CLIRegisterCommand( &Vl53l0xUnitsCommandDefinition);
  FreeRTOS_CLIRegisterCommand( &Vl53l0xMaxCommandDefinition);
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

static void Vl53l0xInit(void)
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
                                        (FixPoint1616_t)(18*65536));
  }

  if (status == VL53L0X_ERROR_NONE)
  {
    /* Timing budget for High accuracy */
    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&vl53l0x_HandleDevice, 200000);
  }

/*   if (status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_GetOffsetCalibrationDataMicroMeter(&vl53l0x_HandleDevice, &offsetCalibration);
  }

  if (status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetOffsetCalibrationDataMicroMeter(&vl53l0x_HandleDevice, offsetCalibration);
  } */

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

/* --- Send result measurement ranging to other UART port
*/
static void HandleTimeout(TimerHandle_t xTimer)
{
  uint32_t tid = 0;

  /* close DMA stream */
  tid = ( uint32_t ) pvTimerGetTimerID( xTimer );
  if (TIMERID_TIMEOUT_MEASUREMENT == tid)
  {
    startMeasurementRanging = STOP_MEASUREMENT_RANGING;
  }
}

/* --- Selection mode running for VL53L0CX
*/
static VL53L0X_Error SettingModeMeasurement(uint8_t mode, uint32_t period, uint32_t timeout)
{
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  TimerHandle_t xTimer = NULL;

  if (VL53L0x_MODE_SINGLE == mode)
  {
    status = VL53L0X_SetDeviceMode(&vl53l0x_HandleDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
  }
  else if (VL53L0x_MODE_CONTINUOUS == mode)
  {
    status = VL53L0X_SetDeviceMode(&vl53l0x_HandleDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  }
  else if (VL53L0x_MODE_CONTINUOUS_TIMED == mode)
  {
    if(VL53L0X_ERROR_NONE == status)
    {
      status = VL53L0X_SetDeviceMode(&vl53l0x_HandleDevice, VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING);
    }
    if(VL53L0X_ERROR_NONE == status)
    {
      status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(&vl53l0x_HandleDevice, period);
    }
  }
  else
  {
    /* nothing to do here */
  }

  if ((timeout > 0) && (timeout < 0xFFFFFFFF))
  {
    /* start software timer which will create event timeout */
    /* Create a timeout timer */
    xTimer = xTimerCreate( "Timeout Measurement", pdMS_TO_TICKS(timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
    /* Start the timeout timer */
    xTimerStart( xTimer, portMAX_DELAY );
  }

  /* start measurement */
  if (status == VL53L0X_ERROR_NONE)
  {
    VL53L0X_StartMeasurement(&vl53l0x_HandleDevice);
  }

  return status;
}

/* --- Wait event for finishing measurement ranging
*/
static Module_Status WaitForMeasurement(void)
{
  Module_Status result = H08R6_ERR_Timeout;
  EventBits_t tEvBits;

  tEvBits = xEventGroupWaitBits(handleNewReadyData, EVENT_READY_MEASUREMENT_DATA, pdTRUE, pdFALSE, (portMAX_DELAY / portTICK_RATE_MS));
  if ((tEvBits & EVENT_READY_MEASUREMENT_DATA) == EVENT_READY_MEASUREMENT_DATA)
  {
    result = H08R6_OK;
    xEventGroupClearBits(handleNewReadyData, EVENT_READY_MEASUREMENT_DATA);
  }
  return result;
}

/* --- Get measurement result
*/
static float GetMeasurementResult(void)
{
  VL53L0X_RangingMeasurementData_t measurementResult;
  VL53L0X_Error status = VL53L0X_ERROR_NONE;

  status = VL53L0X_GetRangingMeasurementData(&vl53l0x_HandleDevice, &measurementResult);

  if(VL53L0X_ERROR_NONE == status)
  {
    status = VL53L0X_ClearInterruptMask(&vl53l0x_HandleDevice, 0);
  }

  return (float)measurementResult.RangeMilliMeter;
}

/* --- Get measurement result and convert from "mm" to other units
 * Input : distance (mm)
*/
static float ConvertCurrentUnit(float distance)
{
  float temp = distance;

  if (UNIT_MEASUREMENT_CM == h08r6UnitMeasurement)
  {
    temp = distance / 10;
  }
  else if (UNIT_MEASUREMENT_INCH == h08r6UnitMeasurement)
  {
    temp = distance / 25.4; /* 1mm = (1/25.4)″ = 0.03937007874″ */
  }
  else
  {
    /* nothing to do here */
  }

  return temp;
}

/* --- Send measurement result
*/
static void SendMeasurementResult(uint8_t request, float distance, uint8_t module, uint8_t port, float *buffer)
{
  uint16_t numberOfParams;
  int8_t *pcOutputString;
  static const int8_t *pcDistanceMsg = ( int8_t * ) "Distance (%s): %.2f\r\n";
  static const int8_t *pcBufferMsg = ( int8_t * ) "Buffer address: (0x%x)\tValue (%s): %.2f\r\n";
  static const int8_t *pcOutMaxRange = ( int8_t * ) "MAX\r\n";
  float tempData;
  char *strUnit;

  /* Get CLI output buffer */
  pcOutputString = FreeRTOS_CLIGetOutputBuffer();
  tempData = ConvertCurrentUnit(distance);

  strUnit = malloc(6*sizeof(char));
  memset(strUnit, 0, (6*sizeof(char)));
  if (UNIT_MEASUREMENT_MM == h08r6UnitMeasurement)
  {
    sprintf( ( char * ) strUnit, "mm");
  }
  else if (UNIT_MEASUREMENT_CM == h08r6UnitMeasurement)
  {
    sprintf( ( char * ) strUnit, "cm");
  }
  else if (UNIT_MEASUREMENT_INCH == h08r6UnitMeasurement)
  {
    sprintf( ( char * ) strUnit, "inch");
  }
  else
  {
    /* nothing to do here */
  }

  if (tempData > h08r6MaxRange)
  {
    switch(request)
    {
      case REQ_SAMPLE_CLI:
      case REQ_STREAM_PORT_CLI:
      case REQ_STREAM_MEMORY_CLI:
        request = REQ_OUT_RANGE_CLI;
        break;
      default:
        request = REQ_OUT_RANGE_ARR;
        break;
    }
  }

  switch(request)
  {
    case REQ_SAMPLE_CLI:
    case REQ_STREAM_PORT_CLI:
      sprintf( ( char * ) pcOutputString, ( char * ) pcDistanceMsg, strUnit, tempData);
      writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
      break;
    case REQ_SAMPLE_ARR:
    case REQ_STREAM_PORT_ARR:
      memset(messageParams, 0, sizeof(messageParams));
      numberOfParams = sizeof(float);
      memcpy(messageParams, &tempData, sizeof(float));
      SendMessageFromPort(port, myID, module, CODE_H08R6_RESULT_MEASUREMENT, numberOfParams);
      break;
    case REQ_STREAM_MEMORY_CLI:
      memset(buffer, 0, sizeof(float));
      memcpy(buffer, &tempData, sizeof(float));
      sprintf( ( char * ) pcOutputString, ( char * ) pcBufferMsg, buffer, strUnit, tempData);
      writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
      break;
    case REQ_STREAM_MEMORY_ARR:
      memset(buffer, 0, sizeof(float));
      memcpy(buffer, &tempData, sizeof(float));
      break;
    case REQ_OUT_RANGE_CLI:
      sprintf( ( char * ) pcOutputString, ( char * ) pcOutMaxRange);
      writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
      break;
    case REQ_OUT_RANGE_ARR:
      messageParams[0] = (uint8_t)(-1);
      SendMessageFromPort(port, myID, module, CODE_H08R6_MAX_RANGE, 1);
      break;
    default:
      break;
  }
  free(strUnit);
}

/* --- Check for CLI stop key
*/
static void CheckPressingEnterKey(void)
{
  int8_t *pcOutputString;

  pcOutputString = FreeRTOS_CLIGetOutputBuffer();
  readPxMutex(PcPort, (char *)pcOutputString, sizeof(char), cmd500ms, 100);
  if ('\r' == pcOutputString[0])
  {
    startMeasurementRanging = STOP_MEASUREMENT_RANGING;
  }
}


/* -----------------------------------------------------------------------
  |                               APIs                                    |
   -----------------------------------------------------------------------
*/

/* --- Takes one sample measurement (triggers ST API Single Ranging)
*/
float Sample_ToF(uint8_t port, uint8_t module)
{
  SettingModeMeasurement(VL53L0x_MODE_SINGLE, 0, 0);
  if (H08R6_OK == WaitForMeasurement())
  {
    distance = GetMeasurementResult();
  }
  if (0 != port)
  {
    SendMeasurementResult(REQ_SAMPLE_ARR, distance, module, port, NULL);
  }

  return distance;
}

/* --- Stream measurements continuously to a port (triggers ST API Continuous Ranging)
*/
float Stream_ToF_Port(uint32_t period, uint32_t timeout, uint8_t port, uint8_t module)
{
  if (0 == period)
  {
    SettingModeMeasurement(VL53L0x_MODE_CONTINUOUS, 0, timeout);
  }
  else
  {
    SettingModeMeasurement(VL53L0x_MODE_CONTINUOUS_TIMED, period, timeout);
  }

  startMeasurementRanging = START_MEASUREMENT_RANGING;
  while(START_MEASUREMENT_RANGING == startMeasurementRanging)
  {
    if (H08R6_OK == WaitForMeasurement())
    {
      distance = GetMeasurementResult();
      if (0 != port)
      {
        SendMeasurementResult(REQ_STREAM_PORT_ARR, distance, module, port, NULL);
      }
    }
    CheckPressingEnterKey();
  }
  startMeasurementRanging = STOP_MEASUREMENT_RANGING;

  return distance;
}

/* --- Stream measurements continuously to a memory location (triggers ST API Continuous Ranging)
*/
void Stream_ToF_Memory(uint32_t period, uint32_t timeout, float* buffer)
{
  if (0 == period)
  {
    SettingModeMeasurement(VL53L0x_MODE_CONTINUOUS, 0, timeout);
  }
  else
  {
    SettingModeMeasurement(VL53L0x_MODE_CONTINUOUS_TIMED, period, timeout);
  }

  startMeasurementRanging = START_MEASUREMENT_RANGING;
  while(START_MEASUREMENT_RANGING == startMeasurementRanging)
  {
    if (H08R6_OK == WaitForMeasurement())
    {
      distance = GetMeasurementResult();
      SendMeasurementResult(REQ_STREAM_MEMORY_ARR, distance, 0, 0, buffer);
    }
    CheckPressingEnterKey();
  }
  startMeasurementRanging = STOP_MEASUREMENT_RANGING;
}

/* --- Stop measurements
*/
Module_Status Stop_ToF(void)
{
  uint32_t StopCompleted = 0;
  uint32_t loop = 0;
  Module_Status result = H08R6_OK;
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  VL53L0X_RangingMeasurementData_t measurementResult;

  VL53L0X_StopMeasurement(&vl53l0x_HandleDevice);
  do{
    status = VL53L0X_GetStopCompletedStatus(&vl53l0x_HandleDevice, &StopCompleted);
    if ((0 == StopCompleted) || (VL53L0X_ERROR_NONE != status))
    {
      break;
    }
    loop++;
    VL53L0X_PollingDelay(&vl53l0x_HandleDevice);
  } while (loop < VL53L0X_DEFAULT_MAX_LOOP);

  if (loop >= VL53L0X_DEFAULT_MAX_LOOP)
  {
    result = H08R6_ERR_Timeout;
  }
  else
  {
    if (VL53L0X_ERROR_NONE != status)
    {
      result = H08R6_ERROR;
    }
    else
    {
      startMeasurementRanging = STOP_MEASUREMENT_RANGING;

      status = VL53L0X_GetRangingMeasurementData(&vl53l0x_HandleDevice, &measurementResult);

      if(VL53L0X_ERROR_NONE == status)
      {
        status = VL53L0X_ClearInterruptMask(&vl53l0x_HandleDevice, 0);
      }
    }
  }
  xEventGroupClearBits(handleNewReadyData, EVENT_READY_MEASUREMENT_DATA);

  return result;
}

/* --- Set measurement unit
*/
Module_Status SetRangeUnit(uint8_t input)
{
  Module_Status result = H08R6_OK;

  switch(input)
  {
    case UNIT_MEASUREMENT_MM:
    case UNIT_MEASUREMENT_CM:
    case UNIT_MEASUREMENT_INCH:
      h08r6UnitMeasurement = input;
      break;
    default:
      result = H08R6_ERROR;
      break;
  }

  return result;
}

/* --- Get measurement unit
*/
uint8_t GetRangeUnit(void)
{
  return h08r6UnitMeasurement;
}

/* -----------------------------------------------------------------------
  |                             Commands                                  |
   -----------------------------------------------------------------------
*/

portBASE_TYPE demoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	static const int8_t *pcMessage = ( int8_t * ) "Streaming range measurements at 2 Hz for 10 seconds\r\n";
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Respond to the command */
	writePxMutex(PcPort, ( char * ) pcMessage, strlen(( char * ) pcMessage), 10, 10);
	SettingModeMeasurement(VL53L0x_MODE_CONTINUOUS_TIMED, 500, 10000);
	startMeasurementRanging = START_MEASUREMENT_RANGING;
	while(START_MEASUREMENT_RANGING == startMeasurementRanging)
	{
		if (H08R6_OK == WaitForMeasurement())
		{
			distance = GetMeasurementResult();
			SendMeasurementResult(REQ_STREAM_PORT_CLI, distance, 0, 0, NULL);
		}
		CheckPressingEnterKey();
	}
	startMeasurementRanging = STOP_MEASUREMENT_RANGING;
	
	strcpy( ( char * ) pcWriteBuffer, "\r\n");
	
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE Vl53l0xSampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) pcCommandString;
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  SettingModeMeasurement(VL53L0x_MODE_SINGLE, 0, 0);
  if (H08R6_OK == WaitForMeasurement())
  {
    distance = GetMeasurementResult();
  }
  SendMeasurementResult(REQ_SAMPLE_CLI, distance, 0, PcPort, NULL);

  /* clean terminal output */
  memset((char *) pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
  sprintf((char *)pcWriteBuffer, "\r\n");

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

static portBASE_TYPE Vl53l0xStreamCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
  int8_t *pcParameterString1; /* period */
  int8_t *pcParameterString2; /* timeout */
  int8_t *pcParameterString3; /* port or buffer */
  int8_t *pcParameterString4; /* module */
  portBASE_TYPE xParameterStringLength1 = 0;
  portBASE_TYPE xParameterStringLength2 = 0;
  portBASE_TYPE xParameterStringLength3 = 0;
  portBASE_TYPE xParameterStringLength4 = 0;
  uint32_t period = 0;
  uint32_t timeout = 0;
  uint8_t port = 0;
  uint8_t module = 0;
  Module_Status result = H08R6_OK;

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  /* Obtain the 1st parameter string: period */
  pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
  /* Obtain the 2nd parameter string: timeout */
  pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
  /* Obtain the 3rd parameter string: port */
  pcParameterString3 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 3, &xParameterStringLength3);
  /* Obtain the 4th parameter string: module */
  pcParameterString4 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 4, &xParameterStringLength4);

  if (NULL != pcParameterString1)
  {
    period = atoi( (char *)pcParameterString1);
  }
  else
  {
    result = H08R6_ERR_WrongParams;
  }
  if (NULL != pcParameterString2)
  {
    if (!strncmp((const char *)pcParameterString2, "inf", 3))
    {
      timeout = portMAX_DELAY;
    }
    else
    {
      timeout = atoi( (char *)pcParameterString2);
    }
  }
  else
  {
    result = H08R6_ERR_WrongParams;
  }

  if (0 == period)
  {
    SettingModeMeasurement(VL53L0x_MODE_CONTINUOUS, 0, timeout);
  }
  else
  {
    SettingModeMeasurement(VL53L0x_MODE_CONTINUOUS_TIMED, period, timeout);
  }

  if (NULL != pcParameterString3 && NULL != pcParameterString4) 
  {
    /* streaming data to port */
		if (pcParameterString3[0] == 'P') {
			port = ( uint8_t ) atol( ( char * ) pcParameterString3+1 );
		}
    module = atoi( (char *)pcParameterString4);

    startMeasurementRanging = START_MEASUREMENT_RANGING;
    while(START_MEASUREMENT_RANGING == startMeasurementRanging)
    {
      if (H08R6_OK == WaitForMeasurement())
      {
        distance = GetMeasurementResult();
        SendMeasurementResult(REQ_STREAM_PORT_ARR, distance, module, port, NULL);
      }
      CheckPressingEnterKey();
    }
    startMeasurementRanging = STOP_MEASUREMENT_RANGING;
  }
  else /* Stream to the CLI */
  {
    /* streaming data to memory */
    startMeasurementRanging = START_MEASUREMENT_RANGING;
    while(START_MEASUREMENT_RANGING == startMeasurementRanging)
    {
      if (H08R6_OK == WaitForMeasurement())
      {
        distance = GetMeasurementResult();
        SendMeasurementResult(REQ_STREAM_PORT_CLI, distance, 0, 0, &h08r6BufStreamMem);
      }
      CheckPressingEnterKey();
    }
    startMeasurementRanging = STOP_MEASUREMENT_RANGING;
  }

  if (H08R6_ERR_WrongParams == result)
  {
    sprintf( ( char * ) pcWriteBuffer, "Wrong parameter\r\n");
    writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
  }

  /* clean terminal output */
  memset((char *) pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
  sprintf((char *)pcWriteBuffer, "\r\n");

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

static portBASE_TYPE Vl53l0xStopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
  Module_Status result = H08R6_OK;

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) pcCommandString;
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  result = Stop_ToF();

  if (H08R6_OK == result)
  {
    sprintf( ( char * ) pcWriteBuffer, "Stop measurement: Success\r\n");
  }
  else
  {
    sprintf( ( char * ) pcWriteBuffer, "Stop measurement: Failure\r\n");
  }
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);

  /* clean terminal output */
  memset((char *) pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
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
    h08r6UnitMeasurement = UNIT_MEASUREMENT_MM;
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used measurement unit: mm\r\n" );
  }
  else if (!strncmp((const char *)pcParameterString1, "cm", 2))
  {
    h08r6UnitMeasurement = UNIT_MEASUREMENT_CM;
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used measurement unit: cm\r\n" );
  }
  else if (!strncmp((const char *)pcParameterString1, "inch", 4))
  {
    h08r6UnitMeasurement = UNIT_MEASUREMENT_INCH;
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

static portBASE_TYPE Vl53l0xMaxCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
  uint8_t temp = 0;
  float distance = 0;
  static const int8_t *pcMaxDistanceMsg = ( int8_t * ) "Maximum distance (mm): %.2f\r\n";

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  SettingModeMeasurement(VL53L0x_MODE_CONTINUOUS, 0, 10000);
  startMeasurementRanging = START_MEASUREMENT_RANGING;
  while((START_MEASUREMENT_RANGING == startMeasurementRanging) && (temp < 10))
  {
    if (H08R6_OK == WaitForMeasurement())
    {
      distance += GetMeasurementResult();
    }
    CheckPressingEnterKey();
    temp++;
  }
  startMeasurementRanging = STOP_MEASUREMENT_RANGING;

  h08r6MaxRange = distance / (temp - 1);

  sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMaxDistanceMsg, h08r6MaxRange);
  writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
  /* clean terminal output */
  memset((char *) pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
  sprintf((char *)pcWriteBuffer, "\r\n");

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}




/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
