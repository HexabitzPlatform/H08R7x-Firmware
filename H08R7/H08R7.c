/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H08R7.c
 Description   : Source code for module H08R7.

 (Description_of_module)
 IR Time-if-Flight (ToF) Sensor (ST VL53L1CX)

 (Description of Special module peripheral configuration):
 >>
 >>
 >>
 */

/* Includes ****************************************************************/
#include "BOS.h"
#include <stdlib.h>

/* Exported Typedef ******************************************************/
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

TimerHandle_t xTimerStream = NULL;
TaskHandle_t ToFHandle = NULL;
uint8_t tofMode ;
/* TOF Sensor Configuration */
VL53L1_Dev_t dev;
VL53L1_DEV Dev = &dev;
VL53L1_PresetModes PresetMode_User = VL53L1_PRESETMODE_AUTONOMOUS;
VL53L1_DistanceModes DistanceMode_User = VL53L1_DISTANCEMODE_LONG;
VL53L1_InterruptMode InterruptMode_User = INTERRUPT_DISABLE;
dynamicZone_s dynamicZone_s_User;
ToF_Structure ToFStructure_User;
uint16_t Dist=0;
/* Private Variables *******************************************************/
/* Streaming variables */
static bool stopStream = false;         /* Flag to indicate whether to stop streaming process */
uint8_t PortModule = 0u;                /* Module ID for the destination port */
uint8_t PortNumber = 0u;                /* Physical port number used for streaming */
uint8_t StreamMode = 0u;                /* Current active streaming mode (to port, terminal, etc.) */
uint8_t TerminalPort = 0u;              /* Port number used to output data to a terminal */
uint8_t StopeCliStreamFlag = 0u;        /* Flag to request stopping a CLI stream operation */
uint32_t SampleCount = 0u;              /* Counter to track the number of samples streamed */
uint32_t PortNumOfSamples = 0u;         /* Total number of samples to be sent through the port */
uint32_t TerminalNumOfSamples = 0u;     /* Total number of samples to be streamed to the terminal */
/* Global variables for sensor data used in ModuleParam */
uint16_t H08R7_distance = 0;

/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] = {
    { .ParamPtr = &H08R7_distance, .ParamFormat = FMT_UINT16, .ParamName = "distance" }
};

/* Local Typedef related to stream functions */
typedef void (*SampleToString)(char*, size_t);
typedef void (*SampleToBuffer)(uint16_t *buffer);

/* Private function prototypes *********************************************/
uint8_t ClearROtopology(void);
void Module_Peripheral_Init(void);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void RemoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);

/* Local function prototypes ***********************************************/
void ToFTask(void *argument);
Module_Status Vl53l1xInit(void);

/* Stream Functions */
void StreamTimeCallback(TimerHandle_t xTimerStream);

void SampleDistanceToString(char *cstring, size_t maxLen) ;

static Module_Status PollingSleepCLISafe(uint32_t period,long Numofsamples);
Module_Status SampleToTerminal(uint8_t dstPort, SampleToString dataFunction);
Module_Status StreamToCLI(uint32_t period, uint32_t timeout,SampleToString function);

/* Create CLI commands *****************************************************/
static portBASE_TYPE SampleSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/* CLI command structure ***************************************************/
const CLI_Command_Definition_t SampleCommandDefinition = {
    (const int8_t*)"sample",
    (const int8_t*)"sample:\r\n Syntax: sample [distance].\r\n\r\n",
    SampleSensorCommand,
    1
};

/***************************************************************************/
const CLI_Command_Definition_t StreamCommandDefinition = {
    (const int8_t*)"stream",
    (const int8_t*)"stream:\r\n Syntax: stream [distance] (Numofsamples) (timeout) [port] [module].\r\n\r\n",
    StreamSensorCommand,
    -1
};
/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/
/* @brief  System Clock Configuration
 *         This function configures the system clock as follows:
 *            - System Clock source            = PLL (HSE)
 *            - SYSCLK(Hz)                     = 64000000
 *            - HCLK(Hz)                       = 64000000
 *            - AHB Prescaler                  = 1
 *            - APB1 Prescaler                 = 1
 *            - HSE Frequency(Hz)              = 8000000
 *            - PLLM                           = 1
 *            - PLLN                           = 16
 *            - PLLP                           = 2
 *            - Flash Latency(WS)              = 2
 *            - Clock Source for UART1,UART2,UART3 = 16MHz (HSI)
 */
void SystemClock_Config(void){
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

   /** Configure the main internal regulator output voltage */
   HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

   /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // Enable both HSI and HSE oscillators
   RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Enable HSE (External High-Speed Oscillator)
   RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI (Internal High-Speed Oscillator)
   RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // No division on HSI
   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration value for HSI
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // Enable PLL
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // Set PLL source to HSE
   RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1; // Prescaler for PLL input
   RCC_OscInitStruct.PLL.PLLN = 16; // Multiplication factor for PLL
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
   RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
   RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
   HAL_RCC_OscConfig(&RCC_OscInitStruct);

   /** Initializes the CPU, AHB and APB buses clocks */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

   HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}

/***************************************************************************/
/* enable stop mode regarding only UART1 , UART2 , and UART3 */
BOS_Status EnableStopModebyUARTx(uint8_t port){

	UART_WakeUpTypeDef WakeUpSelection;
	UART_HandleTypeDef *huart =GetUart(port);

	if((huart->Instance == USART1) || (huart->Instance == USART2) || (huart->Instance == USART3)){

		/* make sure that no UART transfer is on-going */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_BUSY) == SET);

		/* make sure that UART is ready to receive */
		while(__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET);

		/* set the wake-up event:
		 * specify wake-up on start-bit detection */
		WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
		HAL_UARTEx_StopModeWakeUpSourceConfig(huart,WakeUpSelection);

		/* Enable the UART Wake UP from stop mode Interrupt */
		__HAL_UART_ENABLE_IT(huart,UART_IT_WUF);

		/* enable MCU wake-up by LPUART */
		HAL_UARTEx_EnableStopMode(huart);

		/* enter STOP mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
	}
	else
		return BOS_ERROR;

}

/***************************************************************************/
/* Enable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t wakeupPins){

	/* Clear the WUF FLAG */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

	/* Enable the WAKEUP PIN */
	switch(wakeupPins){

		case PA0_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
			break;

		case PA2_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
			break;

		case PB5_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
			break;

		case PC13_PIN:
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
			break;

		case NRST_PIN:
			/* do no thing*/
			break;
	}

	/* Enable SRAM content retention in Standby mode */
	HAL_PWREx_EnableSRAMRetention();

	/* Finally enter the standby mode */
	HAL_PWR_EnterSTANDBYMode();

	return BOS_OK;
}

/***************************************************************************/
/* Disable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t wakeupPins){

	/* The standby wake-up is same as a system RESET:
	 * The entire code runs from the beginning just as if it was a RESET.
	 * The only difference between a reset and a STANDBY wake-up is that, when the MCU wakes-up,
	 * The SBF status flag in the PWR power control/status register (PWR_CSR) is set */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET){
		/* clear the flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Disable  Wake-up Pinx */
		switch(wakeupPins){

			case PA0_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
				break;

			case PA2_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
				break;

			case PB5_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
				break;

			case PC13_PIN:
				HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
				break;

			case NRST_PIN:
				/* do no thing*/
				break;
		}

		IND_blink(1000);

	}
	else
		return BOS_OK;

}

/***************************************************************************/
/* Save Command Topology in Flash RO */
uint8_t SaveTopologyToRO(void)
{
	HAL_StatusTypeDef flashStatus =HAL_OK;
	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd = 8;
    uint16_t temp =0;

    /* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2,TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(flashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if(myID){

		/* Save module's ID */
		temp =(uint16_t )(N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS,temp);

		/* Wait for a Write operation to complete */
		flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

		if(flashStatus != HAL_OK){
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t row =1; row <= N; row++){
			for(uint8_t column =0; column <= MAX_NUM_OF_PORTS; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(Array[row - 1][0]){
					/* Save each element in topology array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,Array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(flashStatus != HAL_OK){
						/* return FLASH error code */
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						/* update new flash memory address */
						flashAdd += 8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Save Command Snippets in Flash RO */
uint8_t SaveSnippetsToRO(void)
{
	HAL_StatusTypeDef FlashStatus =HAL_OK;
    uint8_t snipBuffer[sizeof(Snippet_t) + 1] ={0};

    /* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
    /* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2,SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(FlashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for(uint8_t index = 0; index < NumOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(Snippets[index].Condition.ConditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&Snippets[index],sizeof(Snippet_t));
			/* Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(Snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(Snippet_t)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j*8]);
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for(uint8_t j = 0; j < ((strlen(Snippets[index].CMD) + 1)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(Snippets[index].CMD + j*4 ));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd += 8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Clear Array topology in SRAM and Flash RO */
uint8_t ClearROtopology(void){
	// Clear the array
	memset(Array,0,sizeof(Array));
	N =1;
	myID =0;

	return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
void RemoteBootloaderUpdate(uint8_t src, uint8_t dst, uint8_t inport,
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
		lastModule = Route[NumberOfHops(dst)-1]; /* previous module = Route[Number of hops - 1] */
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

/***************************************************************************/
/* Setup a port for remote ST factory bootloader update:
 * Set baudrate to 57600
 * Enable even parity
 * Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){

	UART_HandleTypeDef *huart =GetUart(port);
	HAL_UART_DeInit(huart);
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);
	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);

}

/***************************************************************************/
/* H0BR4 module initialization */
void Module_Peripheral_Init(void) {

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	//Circulating DMA Channels ON All Module
	for (int i = 1; i <= NUM_OF_PORTS; i++) {
		if (GetUart(i) == &huart1) {
			dmaIndex [i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			dmaIndex [i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			dmaIndex [i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			dmaIndex [i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart5) {
			dmaIndex [i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart6) {
			dmaIndex [i - 1] = &(DMA1_Channel6->CNDTR);
		}
	}

	/* I2C initialization */
	MX_I2C_Init();

	/* Create a ToF task */
	xTaskCreate(ToFTask, (const char*) "ToFTask", (2 * configMINIMAL_STACK_SIZE), NULL,
			osPriorityNormal - osPriorityIdle, &ToFHandle);

	/* Create a timeout software timer StreamSamplsToPort() API */
	xTimerStream = xTimerCreate("StreamTimer", pdMS_TO_TICKS(1000), pdTRUE, (void*) 1, StreamTimeCallback);

//	vTaskStartScheduler();
//	Vl53l1xInit();
}

/***************************************************************************/
/* H0BR4 message processing task */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
  Module_Status result = H08R7_OK;
  uint32_t Numofsamples;
  uint32_t timeout;

  switch (code)
  {
	case CODE_H08R7_SAMPLE_PORT:
		SampleToPort(cMessage[port - 1][shift], cMessage[port - 1][1 + shift]);
		break;

	default:
		result = H08R7_ERR_UNKNOWNMESSAGE;
		break;
  }

  return result;
}

/***************************************************************************/
/* Get the port for a given UART */
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

/***************************************************************************/
/* Register this module CLI Commands */
void RegisterModuleCLICommands(void) {
	FreeRTOS_CLIRegisterCommand(&SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&StreamCommandDefinition);

}

/***************************************************************************/
/* This functions is useful only for input (sensors) modules.
 * Samples a module parameter value based on parameter index.
 * paramIndex: Index of the parameter (1-based index).
 * value: Pointer to store the sampled float value.
 */
Module_Status GetModuleParameter(uint8_t paramIndex, float *value) {
    Module_Status status = H08R7_OK;

    switch (paramIndex) {
        /* Sample ToF Distance sensor */
        case 1:
        {
            uint16_t temp = 0;
            status = SampleTOF(&temp);
            if (status == H08R7_OK) *value = (float)temp;
            break;
        }

        /* Invalid parameter index */
        default:
            status = H08R7_ERR_WRONGPARAMS;
            break;
    }

    return status;
}

/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
/* ToF streaming task */
void ToFTask(void *argument) {

	uint32_t tofPeriod, t0;
	uint8_t  tofState;
	Module_Status Status = H08R7_OK;
	Module_Status st;

	/* Initialization Tof VL53L1 */
	do {
		st = Vl53l1xInit();
	} while (st != H08R7_OK);

	while (1) {

		/* Process data when it's ready from the sensor or when the period timer is expired */
//		if (tofState == REQ_MEASUREMENT_READY || (HAL_GetTick() - t0) >= tofPeriod) {
			switch (tofMode) {

			case SAMPLE_TOF:
				if (tofModeMeasurement(Dev, PresetMode_User, DistanceMode_User, InterruptMode_User, dynamicZone_s_User,
						&ToFStructure_User) == STATUS_OK) {
					Status = H08R7_OK;
				} else {
					Status = H08R7_ERROR;
				}
				Dist = ToFStructure_User.ObjectNumber [0].tofDistanceMm;
				break;

			default:
				break;
			}

			t0 = HAL_GetTick();			// Reset the timer
//		}

		tofState = REQ_IDLE;
		taskYIELD();
	}
}

/***************************************************************************/
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

/***************************************************************************/
/* Callback function triggered by a timer to manage data streaming.
 * xTimerStream: Handle of the timer that triggered the callback.
 */
void StreamTimeCallback(TimerHandle_t xTimerStream) {
	/* Increment sample counter */
	++SampleCount;
	/* Stream mode to port: Send samples to port */
	if (STREAM_MODE_TO_PORT == StreamMode) {
		if ((SampleCount <= PortNumOfSamples) || (0 == PortNumOfSamples)) {
			SampleToPort(PortModule, PortNumber);

		} else {
			xTimerStop(xTimerStream,0);

			SampleCount = 0;
		}
	}
	/* Stream mode to terminal: Export to terminal */
	else if (STREAM_MODE_TO_TERMINAL == StreamMode) {
		if ((SampleCount <= TerminalNumOfSamples)
				|| (0 == TerminalNumOfSamples)) {
			SampleToTerminal(TerminalPort, SampleDistanceToString);
		} else {
			xTimerStop(xTimerStream,0);

			SampleCount = 0;

		}
	}
}

/***************************************************************************/
 Module_Status StreamToCLI(uint32_t Numofsamples, uint32_t timeout,
		SampleToString function) {
	Module_Status status = H08R7_OK;
	int8_t *pcOutputString = NULL;
	uint32_t period = timeout / Numofsamples;
	if (period < MIN_PERIOD_MS)
		return H08R7_ERR_WRONGPARAMS;

	// TODO: Check if CLI is enable or not
	if (1 == StopeCliStreamFlag) {
		StopeCliStreamFlag = 0;
		static char *pcOKMessage = (int8_t*) "Stop stream !\n\r";
		writePxITMutex(pcPort, pcOKMessage, strlen(pcOKMessage), 10);
		return status;
	}

	if (period > timeout)
		timeout = period;

	long numTimes = timeout / period;
	stopStream = false;
	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf[pcPort - 1][chr] == '\r') {
				UARTRxBuf[pcPort - 1][chr] = 0;
			}
		}
	while ((numTimes-- > 0) || (timeout >= MAX_TIMEOUT_MS)) {
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();
		function((char*) pcOutputString, 100);

		writePxMutex(pcPort, (char*) pcOutputString,strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
		if (PollingSleepCLISafe(period,numTimes) != H08R7_OK)
			break;
	}

	memset((char*) pcOutputString, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);
	sprintf((char*) pcOutputString, "\r\n");
	return status;
}

 /***************************************************************************/
void SampleDistanceToString(char *cstring, size_t maxLen) {
	uint16_t distance = 0;
	do {
		SampleTOF(&distance);
	} while (distance == 0);

	snprintf(cstring, maxLen, "Distance: %d\r\n", distance);
}
/***************************************************************************/
void SampleDistanceBuff(uint16_t *buffer) {
	uint16_t distance;
	SampleTOF(&distance);
	*buffer = distance;
}

/***************************************************************************/
/* Streams a single sensor data sample to the terminal.
 * dstPort: Port number to stream data to.
 * dataFunction: Function to sample data (e.g., TOF distance).
 */
Module_Status SampleToTerminal(uint8_t dstPort, SampleToString dataFunction) {
	Module_Status status = H08R7_OK; /* Initialize operation status as success */
	int8_t *pcOutputString = NULL; /* Pointer to CLI output buffer */
	uint32_t period = 0u; /* Calculated period for the operation */
	char cstring[100] = { 0 }; /* Buffer for formatted output string */

	/* Get the CLI output buffer for writing */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	/* Sample data and format it into a string using the callback function */
	dataFunction(cstring, sizeof(cstring));

	/* Send the formatted string to the specified port */
	writePxMutex(dstPort, (char*) cstring, strlen((char*) cstring), cmd500ms,
			HAL_MAX_DELAY);

	/* Return final status indicating success or prior error */
	return status;
}

/***************************************************************************/
/*
 * @brief: Streams data to a buffer.
 * @param buffer: Pointer to the buffer where data will be stored.
 * @param function: Function to sample data (e.g., Distance).
 * @param Numofsamples: Number of samples to take.
 * @param timeout: Timeout period for the operation.
 * @retval: Module status indicating success or error.
 */
static Module_Status StreamToBuf(float *buffer, uint32_t Numofsamples, uint32_t timeout, SampleToBuffer function) {
    Module_Status status = H08R7_OK;
    uint16_t StreamIndex = 0;
    uint32_t period = timeout / Numofsamples;

    /* Check if the calculated period is valid */
    if (period < MIN_PERIOD_MS)
        return H08R7_ERR_WRONGPARAMS;

    stopStream = false;

    /* Stream data to buffer */
    while ((Numofsamples-- > 0) || (timeout >= MAX_TIMEOUT_MS)) {
        uint16_t sample;
        function(&sample);
        buffer[StreamIndex] = sample;
        StreamIndex++;

        /* Delay for the specified period */
        vTaskDelay(pdMS_TO_TICKS(period));

        /* Check if streaming should be stopped */
        if (stopStream) {
            status = H0BR7_ERR_TERMINATED;
            break;
        }
    }

    return status;
}


/***************************************************************************/
/* Polling and sleep function to safely manage CLI stream.
 * period: The period to sleep in milliseconds.
 * Numofsamples: The number of samples to take.
 */
static Module_Status PollingSleepCLISafe(uint32_t period, long Numofsamples) {
	const unsigned DELTA_SLEEP_MS = 100; // milliseconds
	long numDeltaDelay = period / DELTA_SLEEP_MS;
	unsigned lastDelayMS = period % DELTA_SLEEP_MS;

	while (numDeltaDelay-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		/* Look for ENTER key to stop the stream */
		for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf [pcPort - 1] [chr] == '\r' && Numofsamples > 0) {
				UARTRxBuf [pcPort - 1] [chr] = 0;
				StopeCliStreamFlag = 1;
				return H0BR7_ERR_TERMINATED;
			}
		}

		if (stopStream)
			return H0BR7_ERR_TERMINATED;
	}

	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));
	return H08R7_OK;
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status SampleTOF(uint16_t *Distance) {
    Module_Status Status = H08R7_OK;
	    tofMode = SAMPLE_TOF;
	    *Distance = Dist;

	return Status;
}

/***************************************************************************/
/*
 * @brief  Samples distance data from a ToF sensor and exports it to a specified port or module.
 * @param  dstModule: The module number to export data to.
 * @param  dstPort: The port number to export data to.
 * @retval Module_Status indicating success or failure of the operation.
 */
Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort) {
	static uint8_t temp [6] = { 0 }; /* Buffer for data transmission */
	Module_Status status = H08R7_OK; /* Initialize operation status as success */

	/* Check if the port and module ID are valid */
	if (dstPort == 0 && dstModule == myID) {
		return H08R7_ERR_WRONGPARAMS; /* Return error for invalid parameters */
	}

	/* Sample distance data from ToF sensor */
	uint16_t distance = 0;
	status = SampleTOF(&distance);

	/* If data is to be sent locally */
	if (dstModule == myID || dstModule == 0) {
		/* Pack data into temp buffer */
		temp [0] = (uint8_t) (distance);
		temp [1] = (uint8_t) (distance >> 8);

		writePxITMutex(dstPort, (char*) temp, sizeof(uint16_t), 10);
	} else {
		/* Send data to another module */
		MessageParams [1] = (status == H08R7_OK) ? BOS_OK : BOS_ERROR;
		MessageParams [0] = FMT_UINT16;
		MessageParams [2] = 1;
		MessageParams [3] = (uint8_t) (distance);
		MessageParams [4] = (uint8_t) (distance >> 8);

		SendMessageToModule(dstModule, CODE_READ_RESPONSE, sizeof(uint16_t) + 3);
	}

	/* Clear the temp buffer */
	memset(temp, 0, sizeof(temp));

	/* Return final status indicating success or prior error */
	return status;
}

/***************************************************************************/
/*
 * brief: Streams data to the specified port and module with a given number of samples.
 * param targetModule: The target module to which data will be streamed.
 * param portNumber: The port number on the module.
 * param portFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamToPort(uint8_t dstModule, uint8_t dstPort, uint32_t numOfSamples, uint32_t streamTimeout) {
	Module_Status Status = H08R7_OK;
	uint32_t SamplePeriod = 0u;

	/* Check timer handle and timeout validity */
	if ((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples))
		return H08R7_ERROR; /* Assuming H08R7_ERROR is defined in Module_Status */

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_PORT;
	PortModule = dstModule;
	PortNumber = dstPort;
	PortNumOfSamples = numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod = streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if (xTimerIsTimerActive(xTimerStream)) {
		if (pdFAIL == xTimerStop(xTimerStream, 100))
			return H08R7_ERROR;
	}

	/* Start the stream timer */
	if (pdFAIL == xTimerStart(xTimerStream, 100))
		return H08R7_ERROR;

	/* Update timer timeout - This also restarts the timer */
	if (pdFAIL == xTimerChangePeriod(xTimerStream, SamplePeriod, 100))
		return H08R7_ERROR;

	return Status;
}

/***************************************************************************/
/*
 * brief: Streams data to the specified terminal port with a given number of samples.
 * param targetPort: The port number on the terminal.
 * param dataFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamToTerminal(uint8_t dstPort,uint32_t numOfSamples,uint32_t streamTimeout){
	Module_Status Status =H08R7_OK;
	uint32_t SamplePeriod =0u;
	/* Check timer handle and timeout validity */
	if((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples))
		return H08R7_ERROR; /* Assuming H08R7_ERROR is defined in Module_Status */

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_TERMINAL;
	TerminalPort =dstPort;
	TerminalNumOfSamples =numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod =streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if(xTimerIsTimerActive(xTimerStream)){
		if(pdFAIL == xTimerStop(xTimerStream,100))
			return H08R7_ERROR;
	}

	/* Start the stream timer */
	if(pdFAIL == xTimerStart(xTimerStream,100))
		return H08R7_ERROR;

	/* Update timer timeout - This also restarts the timer */
	if(pdFAIL == xTimerChangePeriod(xTimerStream,SamplePeriod,100))
		return H08R7_ERROR;

	return Status;
}

/***************************************************************************/
/* Streams sensor data to a buffer.
 * buffer: Pointer to the buffer where data will be stored.
 * Numofsamples: Number of samples to take.
 * timeout: Timeout period for the operation.
 * function: Function pointer to the sampling function (e.g.,SampleDistanceBuff).
 */
Module_Status StreamToBuffer(float *buffer, uint32_t Numofsamples,uint32_t timeout) {

	return StreamToBuf(buffer, Numofsamples, timeout, SampleDistanceBuff);

}
/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
static portBASE_TYPE SampleSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
    const char *const DistanceCmdName = "distance";

    const char *pSensName = NULL;
    portBASE_TYPE sensNameLen = 0;

    // Make sure we return something
    *pcWriteBuffer = '\0';

    pSensName = (const char*)FreeRTOS_CLIGetParameter(pcCommandString, 1, &sensNameLen);

    if (pSensName == NULL) {
        snprintf((char*)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
        return pdFALSE;
    }

    do {
        if (!strncmp(pSensName, DistanceCmdName, strlen(DistanceCmdName))) {
            SampleToTerminal(pcPort, SampleDistanceToString);
        } else {
            snprintf((char*)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
        }

        return pdFALSE;
    } while (0);

    snprintf((char*)pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");
    return pdFALSE;
}

/***************************************************************************/
static bool StreamCommandParser(const int8_t *pcCommandString, const char **ppSensName, portBASE_TYPE *pSensNameLen,
                                bool *pPortOrCLI, uint32_t *pPeriod, uint32_t *pTimeout, uint8_t *pPort, uint8_t *pModule) {
    const char *pPeriodMSStr = NULL;
    const char *pTimeoutMSStr = NULL;

    portBASE_TYPE periodStrLen = 0;
    portBASE_TYPE timeoutStrLen = 0;

    const char *pPortStr = NULL;
    const char *pModStr = NULL;

    portBASE_TYPE portStrLen = 0;
    portBASE_TYPE modStrLen = 0;

    *ppSensName = (const char*)FreeRTOS_CLIGetParameter(pcCommandString, 1, pSensNameLen);
    pPeriodMSStr = (const char*)FreeRTOS_CLIGetParameter(pcCommandString, 2, &periodStrLen);
    pTimeoutMSStr = (const char*)FreeRTOS_CLIGetParameter(pcCommandString, 3, &timeoutStrLen);

    // At least 3 Parameters are required!
    if ((*ppSensName == NULL) || (pPeriodMSStr == NULL) || (pTimeoutMSStr == NULL))
        return false;

    // TODO: Check if Period and Timeout are integers or not!
    *pPeriod = atoi(pPeriodMSStr);
    *pTimeout = atoi(pTimeoutMSStr);
    *pPortOrCLI = true;

    pPortStr = (const char*)FreeRTOS_CLIGetParameter(pcCommandString, 4, &portStrLen);
    pModStr = (const char*)FreeRTOS_CLIGetParameter(pcCommandString, 5, &modStrLen);

    if ((pModStr == NULL) && (pPortStr == NULL))
        return true;
    if ((pModStr == NULL) || (pPortStr == NULL)) // If user has provided 4 Arguments.
        return false;

    *pPort = atoi(pPortStr);
    *pModule = atoi(pModStr);
    *pPortOrCLI = false;

    return true;
}

/***************************************************************************/
static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
    const char *const DistanceCmdName = "distance";

    uint32_t Numofsamples = 0;
    uint32_t timeout = 0;
    uint8_t port = 0;
    uint8_t module = 0;

    bool portOrCLI = true; // Port Mode => false and CLI Mode => true

    const char *pSensName = NULL;
    portBASE_TYPE sensNameLen = 0;

    // Make sure we return something
    *pcWriteBuffer = '\0';

    if (!StreamCommandParser(pcCommandString, &pSensName, &sensNameLen, &portOrCLI, &Numofsamples, &timeout, &port, &module)) {
        snprintf((char*)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
        return pdFALSE;
    }

    do {
        if (!strncmp(pSensName, DistanceCmdName, strlen(DistanceCmdName))) {
            if (portOrCLI) {
                StreamToCLI(Numofsamples, timeout, SampleDistanceToString);
            } else {
            	StreamToPort(module, port, Numofsamples, timeout);
            }
        } else {
            snprintf((char*)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
        }

        snprintf((char*)pcWriteBuffer, xWriteBufferLen, "\r\n");
        return pdFALSE;
    } while (0);

    snprintf((char*)pcWriteBuffer, xWriteBufferLen, "Error reading Sensor\r\n");
    return pdFALSE;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
