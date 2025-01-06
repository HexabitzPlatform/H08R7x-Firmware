/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/
uint16_t TOFBuffer[100];
uint16_t Index11 = 0;
/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void) {

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for (;;) {
	}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument) {


//	AddPortButton(MOMENTARY_NO, 2);   //Define a button connected to port P1
//	SetButtonEvents(2, 1, 0, 3, 0, 0, 0, 0, 0,1);    // Activate a click event and a pressed_for_x event for 3 seconds


	// put your code here, to run repeatedly.
	while (1) {

		Sample_ToF(&TOFBuffer[Index11]);
		HAL_Delay(50);
		Index11++;
		if (Index11 > 100)
			Index11 = 0;
	}
}

//void buttonClickedCallback(uint8_t port){
//
//	SendMessageToModule(1,CODE_PING, 0);
//	Delay_ms(100);
//
//	SendMessageToModule(2,CODE_PING, 0);
//	Delay_ms(100);
//
//	SendMessageToModule(1,CODE_PING, 0);
//	Delay_ms(100);
//
//	messageParams[0] = 50;
//	SendMessageToModule(1,CODE_H01R0_ON, 1);
//	Delay_ms(500);
//
//	SendMessageToModule(2,CODE_PING, 0);
//	Delay_ms(500);
//
//	SendMessageToModule(1,CODE_H01R0_OFF, 0);
//	Delay_ms(100);
//
//	SendMessageToModule(2,CODE_PING, 0);
//	Delay_ms(100);
//
//	messageParams[0] = 50;
//	SendMessageToModule(1,CODE_H01R0_ON, 1);
//	Delay_ms(500);
//
//	SendMessageToModule(1,CODE_H01R0_OFF, 0);
//	Delay_ms(100);
//
//}

/*-----------------------------------------------------------*/
