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
Module_Status ss;
/* User Task */
void UserTask(void *argument){
	StreamDistanceToTerminal(10, 10000, 5);
	// put your code here, to run repeatedly.
	while(1){

x++;
	}
}

/*-----------------------------------------------------------*/
