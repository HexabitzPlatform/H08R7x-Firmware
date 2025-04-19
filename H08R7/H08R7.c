/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H08R7.c
 Description   : Source code for module H08R7.

 (Description_of_module)
 IR Time-if-Flight (ToF) Sensor (ST VL53L1CX)

 Required MCU resources :

 (Description of Special module peripheral configuration):
 >> USARTs 1,2,3,4,5,6 for module ports (H08R7).
 >> I2C2 for the ToF sensor.
 >> GPIOB 1 for ToF interrupt (INT).
 >> GPIOA 5 for ToF shutdown (XSHUT).
 */

/* Includes ------------------------------------------------------------------*/
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

/* Private variables ---------------------------------------------------------*/
static bool stopStream = false;

/* Streaming variables */
uint8_t PortModule = 0u;           /* Module ID for port streaming */
uint8_t PortNumber = 0u;           /* Port number for streaming */
uint8_t StreamMode = 0u;                     /* Streaming mode selector (port or terminal) */
volatile uint8_t TerminalPort = 0u;          /* Port number for terminal streaming */
// uint8_t TerminalPort =0u;
uint8_t StopeCliStreamFlag = 0u;             /* Flag to stop CLI streaming */
volatile uint32_t PortSamples = 0u;         /* Current sample count for port (if needed separately) */
// uint32_t PortSamples =0u;
uint32_t SampleCount = 0u;                   /* Total sample counter */
uint32_t TerminalTimeout = 0u;               /* Timeout value for terminal streaming */
volatile uint32_t PortNumOfSamples = 0u;    /* Number of samples for port streaming */
// uint32_t PortNumOfSamples =0u;
volatile uint32_t TerminalNumOfSamples = 0u; /* Number of samples for terminal streaming */
//uint32_t TerminalNumOfSamples =0u;

uint16_t H08R7_distance = 0;
uint32_t tofPeriod, t0;
uint8_t  tofMode, tofState;
uint8_t coun;
uint16_t Dist=0;
uint8_t flag;
VL53L1_Dev_t dev;
VL53L1_DEV Dev = &dev;
VL53L1_PresetModes PresetMode_User = VL53L1_PRESETMODE_AUTONOMOUS;
VL53L1_DistanceModes DistanceMode_User = VL53L1_DISTANCEMODE_LONG;
VL53L1_InterruptMode InterruptMode_User = INTERRUPT_DISABLE;
dynamicZone_s dynamicZone_s_User;
ToF_Structure ToFStructure_User;
Module_Status statusD = H08R7_OK;

/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] = {
    { .ParamPtr = &H08R7_distance, .ParamFormat = FMT_UINT16, .ParamName = "distance" }
};

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
// extern uint8_t numOfRecordedSnippets;
// EventGroupHandle_t handleNewReadyData = NULL;
// typedef void (*SampleMemsToPort)(uint8_t, uint8_t);

/* Module exported parameters ------------------------------------------------*/
/* Global variable for ToF sensor data */

// float temp __attribute__((section(".mySection")));
// float sample __attribute__((section(".mySection")));

/* Private function prototypes *********************************************/
uint8_t ClearROtopology(void);
void Module_Peripheral_Init(void);
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);

/* Local function prototypes ***********************************************/
void ToFTask(void *argument);
void StreamTimeCallback(TimerHandle_t xTimerStream);

void SampleDistanceToPort(uint8_t port, uint8_t module);
void SampleDistanceToStringCLI(char *cstring, size_t maxLen) ;

typedef void (*SampleMemsToString)(char*, size_t);
typedef void (*SampleMemsToBuffer)(uint16_t *buffer);

Module_Status SampleToTerminal(uint8_t dstPort, SampleMemsToString dataFunction);
static Module_Status PollingSleepCLISafe(uint32_t period,long Numofsamples);
Module_Status StreamMemsToCLI(uint32_t period, uint32_t timeout,SampleMemsToString function);


/* Create CLI commands *****************************************************/
static portBASE_TYPE Vl53l1xSampleCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE Vl53l1xStreamcliCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE Vl53l1xStreamportCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE Vl53l1xSampleportportCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);

/* CLI command structure ***************************************************/
/* CLI command structure : sample */
const CLI_Command_Definition_t Vl53l1xSampleCommandDefinition = {
		(const int8_t*) "sample", /* The command string to type. */
		(const int8_t*) "sample:\r\nTake one sample measurement\r\n\r\n",
		Vl53l1xSampleCommand, /* The function to run. */
		0 /* No parameters are expected. */
};

/* CLI command structure ***************************************************/
/* CLI command structure : streamtocli */
const CLI_Command_Definition_t Vl53l0xStreamcliCommandDefinition =
		{ (const int8_t*) "streamtocli", /* The command string to type. */
				(const int8_t*) "streamtocli:\r\n Take several samples measurement\r\n\r\n",
				Vl53l1xStreamcliCommand, /* The function to run. */
				2 /* Multiple parameters are expected. */
		};

/***************************************************************************/
/* CLI command structure : streamtoport */
const CLI_Command_Definition_t Vl53l0xStreamportCommandDefinition =
		{ (const int8_t*) "streamtoport", /* The command string to type. */
				(const int8_t*) "streamtoport:\r\n export several samples measurementr\n\r\n",
				Vl53l1xStreamportCommand, /* The function to run. */
				3 /* No parameters are expected. */
		};

/***************************************************************************/
/* CLI command structure : sampletoport */
const CLI_Command_Definition_t Vl53l1xSampletoportCommandDefinition =
		{ (const int8_t*) "sampletoport", /* The command string to type. */
				(const int8_t*) "sampletoport:\r\n export one samples measurementr\r\n\r\n",
				Vl53l1xSampleportportCommand, /* The function to run. */
				1 /* one parameter is expected. */
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
			dmaIndex[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			dmaIndex[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			dmaIndex[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			dmaIndex[i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart5) {
			dmaIndex[i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart6) {
			dmaIndex[i - 1] = &(DMA1_Channel6->CNDTR);
		}
	}

	/* create a event group for measurement ranging */
	// handleNewReadyData = xEventGroupCreate();

	/* I2C initialization */
	MX_I2C_Init();

	/* Create a ToF task */
	xTaskCreate(ToFTask, (const char*) "ToFTask",
			(2 * configMINIMAL_STACK_SIZE), NULL,
			osPriorityNormal - osPriorityIdle, &ToFHandle);
	/* Create a timeout software timer StreamSamplsToPort() API */
		xTimerStream =xTimerCreate("StreamTimer",pdMS_TO_TICKS(1000),pdTRUE,(void* )1,StreamTimeCallback);


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
	case CODE_H08R7_GET_INFO:
		break;
	case CODE_H08R7_SAMPLE_PORT:
		SampleToPort(cMessage[port - 1][shift], cMessage[port - 1][1 + shift]);
		break;
	case CODE_H08R7_STREAM_PORT:
			Numofsamples = ((uint32_t) cMessage[port - 1][2 + shift] ) + ((uint32_t) cMessage[port - 1][3 + shift] << 8) + ((uint32_t) cMessage[port - 1][4 + shift] << 16) + ((uint32_t)cMessage[port - 1][5 + shift] << 24);
			timeout = ((uint32_t) cMessage[port - 1][6 + shift] ) + ((uint32_t) cMessage[port - 1][7 + shift] << 8) + ((uint32_t) cMessage[port - 1][8 + shift] << 16) + ((uint32_t)cMessage[port - 1][9 + shift] << 24);
			 StreamToPort(cMessage[port-1][shift], cMessage[port-1][shift+1], Numofsamples, timeout);
		break;
	default:
		result = H08R7_ERR_UnknownMessage;
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
	FreeRTOS_CLIRegisterCommand(&Vl53l1xSampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&Vl53l0xStreamcliCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&Vl53l0xStreamportCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&Vl53l1xSampletoportCommandDefinition);

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
            status = Sample_ToF(&temp);
            if (status == H08R7_OK) *value = (float)temp;
            break;
        }

        /* Invalid parameter index */
        default:
            status = H08R7_ERR_WrongParams;
            break;
    }

    return status;
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

//		switch (tofMode) {
//				case STREAM_TO_PORT:
//					 StreamMemsToPort( module1,port1,
//								SampleDistanceToPort, Numofsamples1, timeout1);
//					 break;
//				case STREAM_TO_Terminal:
//					StreamMemsToTerminal(Numofsamples3, timeout3, port3,
//							SampleDistanceToStringCLI);
//								 break;
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
/*-----------------------------------------------------------*/
//
// Module_Status StreamMemsToPort(uint8_t module, uint8_t port,
//		SampleMemsToPort function, uint32_t Numofsamples, uint32_t timeout) {
//	Module_Status status = H08R7_OK;
//	uint32_t period = timeout / Numofsamples;
//
//	if (period < MIN_MEMS_PERIOD_MS)
//		return H08R7_ERR_WrongParams;
//	if (port == 0)
//		return H08R7_ERR_WrongParams;
//	if (port == pcPort) // Check if CLI is not enabled at that port!
//		return H08R7_ERR_BUSY;
//
//	if (period > timeout)
//		timeout = period;
//
//	long numTimes = timeout / period;
//	stopStream = false;
//
//	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
//		function(port, module);
//
//		vTaskDelay(pdMS_TO_TICKS(period));
//		if (stopStream) {
//			status = H0BR7_ERR_TERMINATED;
//			break;
//		}
//	}
//	tofMode = DEFAULT;
//	return status;
//}
/***************************************************************************/
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
				MessageParams[1] = BOS_OK;
			else
				MessageParams[1] = BOS_ERROR;
			MessageParams[0] = FMT_UINT16;
			MessageParams[2] = (uint8_t) ((*(uint32_t*) &Distance) >> 0);
			MessageParams[3] = (uint8_t) ((*(uint32_t*) &Distance) >> 8);
			SendMessageToModule(module, CODE_READ_RESPONSE,2 * sizeof(uint8_t) + 2);
		}

}
/***************************************************************************/
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
	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
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
void SampleDistanceToStringCLI(char *cstring, size_t maxLen) {
	uint16_t distance = 0;
	do {
		Sample_ToF(&distance);
	} while (distance == 0);

	snprintf(cstring, maxLen, "Distance: %d\r\n", distance);
}

//void SampleDistanceToString(char *cstring, size_t maxLen) {
//	uint16_t distance = 0;
//	tofModeMeasurement(Dev, PresetMode_User, DistanceMode_User,
//			InterruptMode_User, dynamicZone_s_User, &ToFStructure_User);
//	distance = ToFStructure_User.ObjectNumber[0].tofDistanceMm;
//	snprintf(cstring, maxLen, "Distance: %d\r\n", distance);
//}
/***************************************************************************/
Module_Status StreamDistanceToCLI(uint32_t Numofsamples, uint32_t timeout) {
	return StreamMemsToCLI(Numofsamples, timeout, SampleDistanceToStringCLI);
}


/***************************************************************************/
void SampleDistanceBuff(uint16_t *buffer) {
	uint16_t distance;
	Sample_ToF(&distance);
	*buffer = distance;
}
/*-----------------------------------------------------------*/
//static Module_Status StreamMemsToBuf(uint16_t *Buffer, uint32_t Numofsamples,
//		uint32_t timeout, SampleMemsToBuffer function)
//
//{
//	Module_Status status = H08R7_OK;
//	uint16_t buffer;
//	uint32_t period = timeout / Numofsamples;
//
//	if (period < MIN_MEMS_PERIOD_MS)
//		return H08R7_ERR_WrongParams;
//
//	// TODO: Check if CLI is enable or not
//
//	if (period > timeout)
//		timeout = period;
//
//	long numTimes = timeout / period;
//	stopStream = false;
//
//	while ((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)) {
//		function(&buffer);
//		Buffer[coun] = buffer;
//		coun++;
//		vTaskDelay(pdMS_TO_TICKS(period));
//		if (stopStream) {
//			status = H0BR7_ERR_TERMINATED;
//			break;
//		}
//	}
//	return status;
//}

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
			SampleToTerminal(TerminalPort, SampleDistanceToStringCLI);
		} else {
			xTimerStop(xTimerStream,0);

			SampleCount = 0;

		}
	}
}

/***************************************************************************/
/* Streams a single sensor data sample to the terminal.
 * dstPort: Port number to stream data to.
 * dataFunction: Function to sample data (e.g., TOF distance).
 */
Module_Status SampleToTerminal(uint8_t dstPort, SampleMemsToString dataFunction) {
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

		// Look for ENTER key to stop the stream

		for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
			if (UARTRxBuf[pcPort - 1][chr] == '\r' && Numofsamples > 0) {
				UARTRxBuf[pcPort - 1][chr] = 0;
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

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status Sample_ToF(uint16_t *Distance) {
	tofMode = SAMPLE_TOF;
	*Distance = Dist;
	return statusD;
}
///*-----------------------------------------------------------*/
//Module_Status StreamDistanceToPort(uint8_t module,uint8_t port,uint32_t Numofsamples,uint32_t timeout) {
//	Module_Status status = H08R7_OK;
//	tofMode=STREAM_TO_PORT;
//	port1 = port ;
//	module1 =module;
//	Numofsamples1=Numofsamples;
//	timeout1=timeout;
//	return status;
//}
/*-----------------------------------------------------------*/
//Module_Status StreamDistanceToBuffer(uint16_t *buffer, uint32_t Numofsamples,
//		uint32_t timeout) {
//	return StreamMemsToBuf(buffer, Numofsamples, timeout, SampleDistanceBuff);
//}
///*-----------------------------------------------------------*/
//Module_Status StreamDistanceToTerminal(uint8_t Port ,uint32_t Numofsamples, uint32_t timeout) {
//	Module_Status status = H08R7_OK;
//	tofMode=STREAM_TO_Terminal;
//	port3 = Port ;
//	Numofsamples3=Numofsamples;
//	timeout3=timeout;
//	return status;
//}

/***************************************************************************/
/*
 * @brief  Samples distance data from a ToF sensor and exports it to a specified port or module.
 * @param  dstModule: The module number to export data to.
 * @param  dstPort: The port number to export data to.
 * @retval Module_Status indicating success or failure of the operation.
 */
Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort)
{
    static uint8_t temp[6] = {0};       /* Buffer for data transmission */
    Module_Status status = H08R7_OK;    /* Initialize operation status as success */

    /* Check if the port and module ID are valid */
    if (dstPort == 0 && dstModule == myID)
    {
        return H08R7_ERR_WrongParams;   /* Return error for invalid parameters */
    }

    /* Sample distance data from ToF sensor */
    uint16_t distance = 0;
    status = Sample_ToF(&distance);

    /* If data is to be sent locally */
    if (dstModule == myID || dstModule == 0)
    {
        /* Pack data into temp buffer */
        temp[0] = (uint8_t)(distance);
        temp[1] = (uint8_t)(distance >> 8);

        writePxITMutex(dstPort, (char*)temp, sizeof(uint16_t), 10);
    }
    else
    {
        /* Send data to another module */
        MessageParams[1] = (status == H08R7_OK) ? BOS_OK : BOS_ERROR;
        MessageParams[0] = FMT_UINT16;
        MessageParams[2] = 1;
        MessageParams[3] = (uint8_t)(distance);
        MessageParams[4] = (uint8_t)(distance >> 8);

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
Module_Status StreamToPort(uint8_t dstModule,uint8_t dstPort,uint32_t numOfSamples,uint32_t streamTimeout){
	Module_Status Status =H08R7_OK;
	uint32_t SamplePeriod =0u;

	/* Check timer handle and timeout validity */
	if((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples)){
		return H08R7_ERROR; /* Assuming H08R7_ERROR is defined in Module_Status */
	}

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_PORT;
	PortModule =dstModule;
	PortNumber =dstPort;
	PortNumOfSamples =numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod =streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if(xTimerIsTimerActive(xTimerStream)){
		if(pdFAIL == xTimerStop(xTimerStream,100)){
			return H08R7_ERROR;
		}
	}

	/* Start the stream timer */
	if(pdFAIL == xTimerStart(xTimerStream,100)){
		return H08R7_ERROR;
	}

	/* Update timer timeout - This also restarts the timer */
	if(pdFAIL == xTimerChangePeriod(xTimerStream,SamplePeriod,100)){
		return H08R7_ERROR;
	}

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
	if((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples)){
		return H08R7_ERROR; /* Assuming H08R7_ERROR is defined in Module_Status */
	}

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_TERMINAL;
	TerminalPort =dstPort;
	TerminalTimeout =streamTimeout;
	TerminalNumOfSamples =numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod =streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if(xTimerIsTimerActive(xTimerStream)){
		if(pdFAIL == xTimerStop(xTimerStream,100)){
			return H08R7_ERROR;
		}
	}

	/* Start the stream timer */
	if(pdFAIL == xTimerStart(xTimerStream,100)){
		return H08R7_ERROR;
	}

	/* Update timer timeout - This also restarts the timer */
	if(pdFAIL == xTimerChangePeriod(xTimerStream,SamplePeriod,100)){
		return H08R7_ERROR;
	}

	return Status;
}

/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
static portBASE_TYPE Vl53l1xSampleCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H08R7_OK;

	StreamDistanceToCLI(1, 100);

	return pdFALSE;
}

/***************************************************************************/
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

/***************************************************************************/
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
	StreamToPort(0, Port, Numofsamples, pTimeout);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/***************************************************************************/
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
/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
