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


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE vl53l0xTestBoardCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE vl53l0xSampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE vl53l0xReadCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE vl53l0xReadPeriodCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE vl53l0xStopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE vl53l0xUnitsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure : vl53l0x-test */
const CLI_Command_Definition_t vl53l0xTestBoardCommandDefinition =
{
	( const int8_t * ) "vl53l0x-test", /* The command string to type. */
	( const int8_t * ) "vl53l0x-test:\r\n Command to test PCB board design\r\n\r\n",
	vl53l0xTestBoardCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* CLI command structure : sample */
const CLI_Command_Definition_t vl53l0xSampleCommandDefinition =
{
	( const int8_t * ) "sample", /* The command string to type. */
	( const int8_t * ) "sample:\r\nTake one sample measurement\r\n\r\n",
	vl53l0xSampleCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* CLI command structure : read */
const CLI_Command_Definition_t vl53l0xReadCommandDefinition =
{
	( const int8_t * ) "read", /* The command string to type. */
	( const int8_t * ) "read:\r\nStream measurement\r\n\r\n",
	vl53l0xReadCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* CLI command structure : read period */
const CLI_Command_Definition_t vl53l0xReadPeriodCommandDefinition =
{
	( const int8_t * ) "read-period", /* The command string to type. */
		( const int8_t * ) "read-period:\r\nSet period streaming in msec between measurements\r\n\r\n",
	vl53l0xReadPeriodCommand, /* The function to run. */
	1 /* No parameters are expected. */
};

/* CLI command structure : stop */
const CLI_Command_Definition_t vl53l0xStopCommandDefinition =
{
	( const int8_t * ) "stop", /* The command string to type. */
	( const int8_t * ) "stop:\r\nStop continuous or timed ranging\r\n\r\n",
	vl53l0xStopCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* CLI command structure : units */
const CLI_Command_Definition_t vl53l0xUnitsCommandDefinition =
{
	( const int8_t * ) "units", /* The command string to type. */
		( const int8_t * ) "units:\r\nSetup the range output unit: mm, cm, inch\r\n\r\n",
	vl53l0xUnitsCommand, /* The function to run. */
	1 /* No parameters are expected. */
};
/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
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

	/* I2C initialization */
	MX_I2C_Init();

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
	FreeRTOS_CLIRegisterCommand( &vl53l0xTestBoardCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &vl53l0xSampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &vl53l0xReadCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &vl53l0xReadPeriodCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &vl53l0xStopCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &vl53l0xUnitsCommandDefinition);
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


/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   -----------------------------------------------------------------------
*/

/* --- Get the port for a given UART.
*/
float SampleToF(void)
{
	float result = 0;
  
  
  return result;
}

/* --- Get the port for a given UART.
*/
float ReadToF(uint32_t period)
{
	float result = 0;
  
  
  return result;
}

/* --- Get the port for a given UART.
*/
Module_Status StopToF(void)
{
	Module_Status result = H08R6_OK;
  
  
  return result;
}

/* --- Get the port for a given UART.
*/
Module_Status SetRangeUnit(float input)
{
	Module_Status result = H08R6_OK;
  
  
  return result;
}

/* --- Get the port for a given UART.
*/
float GetRangeUnit(void)
{
	float result = 0;
  
  
  return result;
}

/* -----------------------------------------------------------------------
	|															Commands																 	|
   -----------------------------------------------------------------------
*/
static portBASE_TYPE vl53l0xTestBoardCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
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

  vl53l0x_reset_xshut_pin();

	sprintf((char *)pcWriteBuffer, "\r\n");

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE vl53l0xSampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	sprintf( ( char * ) pcWriteBuffer, "vl53l0xSampleCommand\r\n");
	writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);

	sprintf((char *)pcWriteBuffer, "\r\n");

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE vl53l0xReadCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	sprintf( ( char * ) pcWriteBuffer, "vl53l0xReadCommand\r\n");
	writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);

	sprintf((char *)pcWriteBuffer, "\r\n");

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE vl53l0xReadPeriodCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	sprintf( ( char * ) pcWriteBuffer, "vl53l0xReadPeriodCommand\r\n");
	writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);

	sprintf((char *)pcWriteBuffer, "\r\n");

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE vl53l0xStopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	sprintf( ( char * ) pcWriteBuffer, "vl53l0xStopCommand\r\n");
	writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);

	sprintf((char *)pcWriteBuffer, "\r\n");

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE vl53l0xUnitsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	sprintf( ( char * ) pcWriteBuffer, "vl53l0xUnitsCommand\r\n");
	writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);

	sprintf((char *)pcWriteBuffer, "\r\n");

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}



/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
