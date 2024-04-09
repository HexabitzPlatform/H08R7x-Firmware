/*
 BitzOS (BOS) V0.3.2 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "Bracelet_IR_ToF.h"
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int x ;
float mm ;
int main(void){

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){
	Vl53l1xInit();
	// put your code here, to run repeatedly.
	while(1){
Sample_ToF(&mm);
//mm= es();
x++;
	}
}

/*-----------------------------------------------------------*/
