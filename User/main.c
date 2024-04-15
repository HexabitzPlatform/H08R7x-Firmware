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
void SampleDistanceBuff(uint16_t *buffer);
/*-----------------------------------------------------------*/
uint16_t b ;
uint16_t s[40];
/* User Task */
void UserTask(void *argument){

	StreamDistanceToBuffer(s, 200, 5000);
	// put your code here, to run repeatedly.
	while(1){
//		Sample_ToF(&b);
//		 SampleDistanceBuff(s);
x++;
	}
}

/*-----------------------------------------------------------*/
