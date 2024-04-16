/*
 BitzOS (BOS) V0.3.2 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int x ;

int main(void){

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){}
}

/*-----------------------------------------------------------*/
uint16_t b ;
uint16_t s[40];
/* User Task */
void UserTask(void *argument){
//	StreamDistanceToPort(6, 0, 10, 2000);
//	StreamDistanceToCLI(10, 2000);
//	StreamDistanceToBuffer(s, 10, 2000);
//	StreamDistanceToBuffer(s, 200, 5000);
	// put your code here, to run repeatedly.
	while(1){
//		Sample_ToF(&b);
		SampletoPort(0, 6);
		HAL_Delay(1000);
x++;
	}
}

/*-----------------------------------------------------------*/
