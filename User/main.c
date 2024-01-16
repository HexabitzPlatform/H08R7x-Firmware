/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void){

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){}
}
uint32_t  f;
/*-----------------------------------------------------------*/
float buffer[60];

uint32_t period = 200;
uint32_t timeout = 5000;
uint8_t port = 3;
uint8_t module = 2;
bool verbose = 0 ;
/* User Task */
void UserTask(void *argument){
//Stream_ToF_Memory(100, 5000, buffer);

		Stream_ToF_Port (period, timeout, port, module, verbose);
	// put your code here, to run repeatedly.
	while(1){
//Stream_ToF_Port(period, timeout, port, module, verbose)
//	Stream_ToF_Memory(200, 5000, buffer);
		f++;


//		f=HAL_RCC_GetSysClockFreq();
	}
}

/*-----------------------------------------------------------*/
