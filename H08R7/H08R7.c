/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H08R7.c
 Description   : Source code for module H08R7.
 IR Time-if-Flight (ToF) Sensor (ST VL53L1CX)

 Required MCU resources :

 >> USARTs 1,2,3,4,5,6 for module ports (H08R7).
 >> I2C2 for the ToF sensor.
 >> GPIOB 1 for ToF interrupt (INT).
 >> GPIOA 5 for ToF shutdown (XSHUT).

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include <stdlib.h>
/* to Vl53l1xInit   */
VL53L1_Dev_t dev;
VL53L1_DEV Dev = &dev;
VL53L1_PresetModes PresetMode_User = VL53L1_PRESETMODE_AUTONOMOUS;
VL53L1_DistanceModes DistanceMode_User = VL53L1_DISTANCEMODE_LONG;
VL53L1_InterruptMode InterruptMode_User = INTERRUPT_DISABLE;
dynamicZone_s dynamicZone_s_User;
ToF_Structure ToFStructure_User;
Module_Status statusD = H08R7_OK;
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;
/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;
EventGroupHandle_t handleNewReadyData = NULL;
typedef void (*SampleMemsToPort)(uint8_t, uint8_t);
typedef void (*SampleMemsToString)(char*, size_t);
typedef void (*SampleMemsToBuffer)(uint16_t *buffer);
/* Module exported parameters ------------------------------------------------*/
float H08R7_range = 0.0f;
float temp __attribute__((section(".mySection")));
float sample __attribute__((section(".mySection")));
module_param_t modParam[NUM_MODULE_PARAMS] = { { .paramPtr = &H08R7_range,
		.paramFormat = FMT_FLOAT, .paramName = "range" } };

/* Private variables ---------------------------------------------------------*/
uint8_t port1, module1;
uint8_t port2 ,module2,mode2,mode1;
uint32_t Numofsamples1 ,timeout1;
uint8_t port3 ,module3,mode3;
uint32_t Numofsamples3 ,timeout3;
TaskHandle_t ToFHandle = NULL;
uint32_t tofPeriod, t0;
uint8_t  tofMode, tofState;
TimerHandle_t xTimerTof = NULL;
uint8_t coun;
uint16_t Dist;
uint8_t flag ;
static bool stopStream = false;
/* Private function prototypes -----------------------------------------------*/
void ToFTask(void *argument);
void Stream_ToF(uint32_t period, uint32_t timeout);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src, uint8_t dst, uint8_t inport,uint8_t outport);
void SampleDistanceToPort(uint8_t port, uint8_t module);
void SampleDistanceToString(char *cstring, size_t maxLen);
void SampleDistanceToStringCLI(char *cstring, size_t maxLen) ;
Module_Status StreamMemsToCLI(uint32_t period, uint32_t timeout,SampleMemsToString function);
Module_Status StreamMemsToPort(uint8_t module, uint8_t port,SampleMemsToPort function, uint32_t Numofsamples, uint32_t timeout);
static Module_Status PollingSleepCLISafe(uint32_t period,long Numofsamples);
 Module_Status StreamMemsToTerminal(uint32_t Numofsamples,uint32_t timeout, uint8_t Port, SampleMemsToString function);
/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE Vl53l1xSampleCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE Vl53l1xStreamcliCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE Vl53l1xStreamportCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE Vl53l1xSampleportportCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);

///*-----------------------------------------------------------*/
/* CLI command structure : sample */
const CLI_Command_Definition_t Vl53l1xSampleCommandDefinition = {
		(const int8_t*) "sample", /* The command string to type. */
		(const int8_t*) "sample:\r\nTake one sample measurement\r\n\r\n",
		Vl53l1xSampleCommand, /* The function to run. */
		0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : streamtocli */
const CLI_Command_Definition_t Vl53l0xStreamcliCommandDefinition =
		{ (const int8_t*) "streamtocli", /* The command string to type. */
				(const int8_t*) "streamtocli:\r\n Take several samples measurement\r\n\r\n",
				Vl53l1xStreamcliCommand, /* The function to run. */
				2 /* Multiple parameters are expected. */
		};
///*-----------------------------------------------------------*/
/* CLI command structure : streamtoport */
const CLI_Command_Definition_t Vl53l0xStreamportCommandDefinition =
		{ (const int8_t*) "streamtoport", /* The command string to type. */
				(const int8_t*) "streamtoport:\r\n export several samples measurementr\n\r\n",
				Vl53l1xStreamportCommand, /* The function to run. */
				3 /* No parameters are expected. */
		};
///*-----------------------------------------------------------*/
/* CLI command structure : sampletoport */
const CLI_Command_Definition_t Vl53l1xSampletoportCommandDefinition =
		{ (const int8_t*) "sampletoport", /* The command string to type. */
				(const int8_t*) "sampletoport:\r\n export one samples measurementr\r\n\r\n",
				Vl53l1xSampleportportCommand, /* The function to run. */
				1 /* one parameter is expected. */
		};

/* -----------------------------------------------------------------------
 |                        Private Functions                              |
 -----------------------------------------------------------------------
 */

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 6
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 12;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC
			| RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;

	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src, uint8_t dst, uint8_t inport,
		uint8_t outport) {

	uint8_t myOutport = 0, lastModule = 0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport = FindRoute(myID, dst);
	if (outport && dst == myID) { /* This is a 'via port' update and I'm the last module */
		myOutport = outport;
		lastModule = myID;
	} else if (outport == 0) { /* This is a remote update */
		if (NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if (src == myID) {
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		if (outport == 0)		// This is a remote module update
			sprintf((char*) pcOutputString, pcRemoteBootloaderUpdateMessage,
					dst);
		else
			// This is a 'via port' remote update
			sprintf((char*) pcOutputString,
					pcRemoteBootloaderUpdateViaPortMessage, dst, outport);

		strcat((char*) pcOutputString, pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport, (char*) pcOutputString,
				strlen((char*) pcOutputString), cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);

	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport, myID, myOutport, myID, BIDIRECTIONAL,
			0xFFFFFFFF, 0xFFFFFFFF, false);
}

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port) {
	UART_HandleTypeDef *huart = GetUart(port);

	huart->Init.BaudRate = 57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

/* --- H08R7 module initialization.
 */
void Module_Peripheral_Init(void) {

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	//Circulating DMA Channels ON All Module
	for (int i = 1; i <= NumOfPorts; i++) {
		if (GetUart(i) == &huart1) {
			index_dma[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			index_dma[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			index_dma[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			index_dma[i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart5) {
			index_dma[i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart6) {
			index_dma[i - 1] = &(DMA1_Channel6->CNDTR);
		}
	}

	/* create a event group for measurement ranging */
	handleNewReadyData = xEventGroupCreate();

	/* I2C initialization */
	MX_I2C_Init();

	/* Create a ToF task */
	xTaskCreate(ToFTask, (const char*) "ToFTask",
			(2 * configMINIMAL_STACK_SIZE), NULL,
			osPriorityNormal - osPriorityIdle, &ToFHandle);

}

void initialValue(void) {
	sample = 0;
}
/*-----------------------------------------------------------*/

/* --- Save array topology and Command Snippets in Flash RO ---
 */
uint8_t SaveToRO(void) {
	BOS_Status result = BOS_OK;
	HAL_StatusTypeDef FlashStatus = HAL_OK;
	uint16_t add = 8;
	uint16_t temp = 0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] = { 0 };

	HAL_FLASH_Unlock();
	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1, RO_START_ADDRESS);
	FlashStatus = FLASH_WaitForLastOperation(
			(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	FLASH_PageErase(FLASH_BANK_1, RO_MID_ADDRESS);
	//TOBECHECKED
	FlashStatus = FLASH_WaitForLastOperation(
			(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
	if (FlashStatus != HAL_OK) {
		return pFlash.ErrorCode;
	} else {
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}

	/* Save number of modules and myID */
	if (myID) {
		temp = (uint16_t) (N << 8) + myID;
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, RO_START_ADDRESS, temp);
		//TOBECHECKED
		FlashStatus = FLASH_WaitForLastOperation(
				(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
		if (FlashStatus != HAL_OK) {
			return pFlash.ErrorCode;
		} else {
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
		}

		/* Save topology */
		for (uint8_t i = 1; i <= N; i++) {
			for (uint8_t j = 0; j <= MaxNumOfPorts; j++) {
				if (array[i - 1][0]) {

					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
							RO_START_ADDRESS + add, array[i - 1][j]);
					//HALFWORD 	//TOBECHECKED
					FlashStatus = FLASH_WaitForLastOperation(
							(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
					if (FlashStatus != HAL_OK) {
						return pFlash.ErrorCode;
					} else {
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
						add += 8;
					}
				}
			}
		}
	}

	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for (uint8_t s = 0; s < numOfRecordedSnippets; s++) {
		if (snippets[s].cond.conditionType) {
			snipBuffer[0] = 0xFE;		// A marker to separate Snippets
			memcpy((uint32_t*) &snipBuffer[1], (uint8_t*) &snippets[s],
					sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for (uint8_t j = 0; j < (sizeof(snippet_t) / 4); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAdd,
						*(uint64_t*) &snipBuffer[j * 8]);
				//HALFWORD
				//TOBECHECKED
				FlashStatus = FLASH_WaitForLastOperation(
						(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 8;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for (uint8_t j = 0; j < ((strlen(snippets[s].cmd) + 1) / 4); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAdd,
						*(uint64_t*) (snippets[s].cmd + j * 4));
				//HALFWORD
				//TOBECHECKED
				FlashStatus = FLASH_WaitForLastOperation(
						(uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 8;
				}
			}
		}
	}

	HAL_FLASH_Lock();

	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void) {
	// Clear the array
	memset(array, 0, sizeof(array));
	N = 1;
	myID = 0;

	return SaveToRO();
}

/* --- H08R7 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
  Module_Status result = H08R7_OK;
  uint32_t Numofsamples;
  uint32_t timeout;
  switch (code)
  {
	case CODE_H08R7_GET_INFO:
		break;
	case CODE_H08R7_SAMPLE_PORT:
		SampletoPort(cMessage[port - 1][shift], cMessage[port - 1][1 + shift]);
		break;
	case CODE_H08R7_STREAM_PORT:
			Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift] << 24);
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift] << 24);
			 StreamMemsToPort(cMessage[port-1][shift], cMessage[port-1][shift+1],
						SampleDistanceToPort, Numofsamples, timeout);
		break;
	default:
		result = H08R7_ERR_UnknownMessage;
		break;
  }

  return result;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void) {
	FreeRTOS_CLIRegisterCommand(&Vl53l1xSampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&Vl53l0xStreamcliCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&Vl53l0xStreamportCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&Vl53l1xSampletoportCommandDefinition);

}

/*-----------------------------------------------------------*/

/* --- Get the port for a given UART.
 */
uint8_t GetPort(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART4)
		return P1;
	else if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART3)
		return P3;
	else if (huart->Instance == USART1)
		return P4;
	else if (huart->Instance == USART5)
		return P5;
	else if (huart->Instance == USART6)
		return P6;

	return 0;
}

/*-----------------------------------------------------------*/

/* --- ToF streaming task 
 */

void ToFTask(void *argument) {
	/* Initialization Tof VL53L1 */
	Module_Status st;
	do {
		st=Vl53l1xInit();
	} while (st != H08R7_OK);

	while (1) {

//		 Process data when it's ready from the sensor or when the period timer is expired
		if (tofState == REQ_MEASUREMENT_READY
				|| (HAL_GetTick() - t0) >= tofPeriod) {
			switch (tofMode) {
			case SAMPLE_TOF:

				if (tofModeMeasurement(Dev, PresetMode_User, DistanceMode_User,
						InterruptMode_User, dynamicZone_s_User,
						&ToFStructure_User) == STATUS_OK) {
					statusD = H08R7_OK;
				} else {
					statusD = H08R7_ERROR;
				}
				Dist = ToFStructure_User.ObjectNumber[0].tofDistanceMm;

				break;

			default:
				break;
			}

			t0 = HAL_GetTick();			// Reset the timer
		}

		switch (tofMode) {
				case STREAM_TO_PORT:
					 StreamMemsToPort( module1,port1,
								SampleDistanceToPort, Numofsamples1, timeout1);
					 break;
				case STREAM_TO_Terminal:
					StreamMemsToTerminal(Numofsamples3, timeout3, port3,
							SampleDistanceToStringCLI);
								 break;
		}


		tofState = REQ_IDLE;
		taskYIELD();
	}
}

/*-----------------------------------------------------------*/
Module_Status Vl53l1xInit(void) {
	Module_Status status = H08R7_OK;
	if (IRSensorInit(Dev) == STATUS_OK) {
		status = H08R7_OK;
	} else {
		status = H08R7_ERROR;
	}
	dynamicZone_s_User.dynamicMultiZone_user = DYNAMIC_MZONE_OFF;
	dynamicZone_s_User.dynamicRangingZone_user = DYNAMIC_ZONE_OFF;
	return status;
}
/*-----------------------------------------------------------*/

 Module_Status StreamMemsToPort(uint8_t module, uint8_t port,
		SampleMemsToPort function, uint32_t Numofsamples, uint32_t timeout) {
	Module_Status status = H08R7_OK;
	uint32_t period = timeout / Numofsamples;

	if (period < MIN_MEMS_PERIOD_MS)
		return H08R7_ERR_WrongParams;
	if (port == 0)
		return H08R7_ERR_WrongParams;
	if (port == PcPort) // Check if CLI is not enabled at that port!
		return H08R7_ERR_BUSY;

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		function(port, module);

		vTaskDelay(pdMS_TO_TICKS(period));
		if (stopStream) {
			status = H0BR7_ERR_TERMINATED;
			break;
		}
	}
	tofMode = DEFAULT;
	return status;
}
/*-----------------------------------------------------------*/
void SampleDistanceToPort(uint8_t port, uint8_t module) {
	uint16_t Distance; // Three Samples X, Y, Z
	static uint8_t temp[4];
	Module_Status status = H08R7_OK;


		status = Sample_ToF(&Distance);

		if (module == myID) {
			temp[0] = (uint8_t) ((*(uint32_t*) &Distance) >> 0);
			temp[1] = (uint8_t) ((*(uint32_t*) &Distance) >> 8);
			writePxITMutex(port, (char*) &temp[0], 2 * sizeof(uint8_t), 10);
		} else {
			if (H08R7_OK == status)
				messageParams[1] = BOS_OK;
			else
				messageParams[1] = BOS_ERROR;
			messageParams[0] = FMT_UINT16;
			messageParams[2] = (uint8_t) ((*(uint32_t*) &Distance) >> 0);
			messageParams[3] = (uint8_t) ((*(uint32_t*) &Distance) >> 8);
			SendMessageToModule(module, CODE_READ_RESPONSE,2 * sizeof(uint8_t) + 2);
		}

}
/*-----------------------------------------------------------*/
 Module_Status StreamMemsToCLI(uint32_t Numofsamples, uint32_t timeout,
		SampleMemsToString function) {
	Module_Status status = H08R7_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_MEMS_PERIOD_MS)
		return H08R7_ERR_WrongParams;

	// TODO: Check if CLI is enable or not

	if (1 == flag) {
		flag = 0;
		static char *pcOKMessage = (int8_t*) "Stop stream !\n\r";
		writePxITMutex(PcPort, pcOKMessage, strlen(pcOKMessage), 10);
		return status;
	}

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;
	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf[PcPort - 1][chr] == '\r') {
				UARTRxBuf[PcPort - 1][chr] = 0;
			}
		}
	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char*) pcOutputString, 100);

		writePxMutex(PcPort, (char*) pcOutputString,strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period,numTimes) != H08R7_OK)
			break;
	}

	memset((char*) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	sprintf((char*) pcOutputString, "\r\n");
	return status;
}
/*-----------------------------------------------------------*/
void SampleDistanceToStringCLI(char *cstring, size_t maxLen) {
		uint16_t distance = 0;
	if (tofModeMeasurement(Dev, PresetMode_User, DistanceMode_User,
						InterruptMode_User, dynamicZone_s_User,
						&ToFStructure_User) == STATUS_OK) {
					statusD = H08R7_OK;
				} else {
					statusD = H08R7_ERROR;
				}
	distance = ToFStructure_User.ObjectNumber[0].tofDistanceMm;

	snprintf(cstring, maxLen, "Distance: %d\r\n", distance);
}

void SampleDistanceToString(char *cstring, size_t maxLen) {
	uint16_t distance = 0;
	tofModeMeasurement(Dev, PresetMode_User, DistanceMode_User,
			InterruptMode_User, dynamicZone_s_User, &ToFStructure_User);
	distance = ToFStructure_User.ObjectNumber[0].tofDistanceMm;
	snprintf(cstring, maxLen, "Distance: %d\r\n", distance);
}
/*-----------------------------------------------------------*/
Module_Status StreamDistanceToCLI(uint32_t Numofsamples, uint32_t timeout) {
	return StreamMemsToCLI(Numofsamples, timeout, SampleDistanceToStringCLI);
}
/*-----------------------------------------------------------*/
static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples) {
	const unsigned DELTA_SLEEP_MS = 100; // milliseconds
	long numDeltaDelay = period / DELTA_SLEEP_MS;
	unsigned lastDelayMS = period % DELTA_SLEEP_MS;

	while (numDeltaDelay-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		// Look for ENTER key to stop the stream

		for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf[PcPort - 1][chr] == '\r' && Numofsamples > 0) {
				UARTRxBuf[PcPort - 1][chr] = 0;
				flag=1;
				return H0BR7_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H0BR7_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H08R7_OK;
}
/*-----------------------------------------------------------*/

void SampleDistanceBuff(uint16_t *buffer) {
	uint16_t distance;
	Sample_ToF(&distance);
	*buffer = distance;
}
/*-----------------------------------------------------------*/
static Module_Status StreamMemsToBuf(uint16_t *Buffer, uint32_t Numofsamples,
		uint32_t timeout, SampleMemsToBuffer function)

{
	Module_Status status = H08R7_OK;
	uint16_t buffer;
	uint32_t period = timeout / Numofsamples;

	if (period < MIN_MEMS_PERIOD_MS)
		return H08R7_ERR_WrongParams;

	// TODO: Check if CLI is enable or not

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		function(&buffer);
		Buffer[coun] = buffer;
		coun++;
		vTaskDelay(pdMS_TO_TICKS(period));
		if (stopStream) {
			status = H0BR7_ERR_TERMINATED;
			break;
		}
	}
	return status;
}
/*-----------------------------------------------------------*/
 Module_Status StreamMemsToTerminal(uint32_t Numofsamples,
		uint32_t timeout, uint8_t Port, SampleMemsToString function) {
	Module_Status status = H08R7_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_MEMS_PERIOD_MS)
		return H08R7_ERR_WrongParams;

	// TODO: Check if CLI is enable or not

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;

	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char*) pcOutputString, 100);

		writePxMutex(Port, (char*) pcOutputString,
				strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period,numTimes) != H08R7_OK)
			break;
	}

	memset((char*) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	sprintf((char*) pcOutputString, "\r\n");
	tofMode = DEFAULT;
	return status;
}

/* -----------------------------------------------------------------------
 |                               APIs                                    |
 -----------------------------------------------------------------------
 */

Module_Status Sample_ToF(uint16_t *Distance) {
	tofMode = SAMPLE_TOF;
	*Distance = Dist;
	return statusD;
}
/*-----------------------------------------------------------*/
Module_Status StreamDistanceToPort(uint8_t module,uint8_t port,uint32_t Numofsamples,uint32_t timeout) {
	Module_Status status = H08R7_OK;
	tofMode=STREAM_TO_PORT;
	port1 = port ;
	module1 =module;
	Numofsamples1=Numofsamples;
	timeout1=timeout;
	return status;
}
/*-----------------------------------------------------------*/
Module_Status StreamDistanceToBuffer(uint16_t *buffer, uint32_t Numofsamples,
		uint32_t timeout) {
	return StreamMemsToBuf(buffer, Numofsamples, timeout, SampleDistanceBuff);
}
/*-----------------------------------------------------------*/
Module_Status StreamDistanceToTerminal(uint8_t Port ,uint32_t Numofsamples, uint32_t timeout) {
	Module_Status status = H08R7_OK;
	tofMode=STREAM_TO_Terminal;
	port3 = Port ;
	Numofsamples3=Numofsamples;
	timeout3=timeout;
	return status;
}
/*-----------------------------------------------------------*/
Module_Status SampletoPort(uint8_t module, uint8_t port) {
	uint16_t Distance;
	static uint8_t temp[4] = { 0 };
	Module_Status status = H08R7_OK;

	if (port == 0 && module == myID) {
		return H08R7_ERR_WrongParams;
	}
	status = Sample_ToF(&Distance);

	if (module == myID) {
		temp[0] = (uint8_t) ((*(uint32_t*) &Distance) >> 0);
		temp[1] = (uint8_t) ((*(uint32_t*) &Distance) >> 8);
		writePxITMutex(port, (char*) &temp[0], 2 * sizeof(uint8_t), 10);
	} else {
		if (H08R7_OK == status)
			messageParams[1] = BOS_OK;
		else
			messageParams[1] = BOS_ERROR;
		messageParams[0] = FMT_UINT16;
		messageParams[2] = 1;
		messageParams[3] = (uint8_t) ((*(uint32_t*) &Distance) >> 0);
		messageParams[4] = (uint8_t) ((*(uint32_t*) &Distance) >> 8);
		SendMessageToModule(module, CODE_READ_RESPONSE,2 * sizeof(uint8_t) + 3);
	}

	return status;
}
/* -----------------------------------------------------------------------
 |                             Commands                                  |
 -----------------------------------------------------------------------
 */

/*-----------------------------------------------------------*/

static portBASE_TYPE Vl53l1xSampleCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H08R7_OK;

	StreamDistanceToCLI(1, 100);

	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE Vl53l1xStreamcliCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H08R7_OK;

	uint32_t Numofsamples, pTimeout;
	static int8_t *pcParameterString1, *pcParameterString2;
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0;

	(void) xWriteBufferLen;

	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);
	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&xParameterStringLength2);

	Numofsamples = atoi(pcParameterString1);
	pTimeout = atoi(pcParameterString2);
	StreamDistanceToCLI(Numofsamples, pTimeout);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE Vl53l1xStreamportCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H08R7_OK;

	uint8_t Port;
	uint32_t Numofsamples, pTimeout;
	static int8_t *pcParameterString1, *pcParameterString2, *pcParameterString3;
	portBASE_TYPE xParameterStringLength1 = 0, xParameterStringLength2 = 0,
			xParameterStringLength3 = 0;

	(void) xWriteBufferLen;

	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);
	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&xParameterStringLength2);
	pcParameterString3 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 3,
			&xParameterStringLength3);
	Port = atoi(pcParameterString1);
	Numofsamples = atoi(pcParameterString2);
	pTimeout = atoi(pcParameterString3);
	StreamMemsToPort(0, Port,SampleDistanceToPort, Numofsamples, pTimeout);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE Vl53l1xSampleportportCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H08R7_OK;
	uint8_t Port;
	static int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;

	(void) xWriteBufferLen;

	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);

	Port = atoi(pcParameterString1);

	SampleDistanceToPort(Port, 0);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}



/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
