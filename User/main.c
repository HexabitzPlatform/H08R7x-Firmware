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
VL53L1_Dev_t                   dev;
VL53L1_DEV                     Dev = &dev;

VL53L1_PresetModes PresetMode_User = VL53L1_PRESETMODE_MULTIZONES_SCANNING;
VL53L1_DistanceModes DistanceMode_User = VL53L1_DISTANCEMODE_LONG;
VL53L1_InterruptMode InterruptMode_User = INTERRUPT_DISABLE;
dynamicZone_s dynamicZone_s_User;
ToF_Structure ToFStructure_User;

Status_TypeDef SSS=STATUS_OK;

float mm ;
int main(void){

	Module_Init();		//Initialize Module &  BitzOS
	  SSS=IRSensorInit(Dev);

	  dynamicZone_s_User.dynamicMultiZone_user=DYNAMIC_MZONE_OFF;
	  dynamicZone_s_User.dynamicRangingZone_user=DYNAMIC_ZONE_ON;
	//Don't place your code here.
	for(;;){}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){

	// put your code here, to run repeatedly.
	while(1){
		  SSS = tofModeMeasurement(Dev,
				  PresetMode_User,
				  DistanceMode_User,
				  InterruptMode_User,
		  	  	  dynamicZone_s_User,
				  &ToFStructure_User);

		  mm =ToFStructure_User.ObjectNumber[0].tofDistanceMm;
		  HAL_Delay(500);
	}
}

/*-----------------------------------------------------------*/
