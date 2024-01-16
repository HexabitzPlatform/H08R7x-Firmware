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

/*-----------------------------------------------------------*/
float buffer[60];
/* User Task */
void UserTask(void *argument){
//Stream_ToF_Memory(100, 5000, buffer);
	// put your code here, to run repeatedly.
	while(1){
//Stream_ToF_Port(period, timeout, port, module, verbose)
//	Stream_ToF_Memory(200, 5000, buffer);

	}
}

/*-----------------------------------------------------------*/
