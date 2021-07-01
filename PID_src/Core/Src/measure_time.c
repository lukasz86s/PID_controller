/*
 * measure_time.c
 *
 *  Created on: 30 cze 2021
 *      Author: fet
 */


#include "measure_time.h"
#include "tim.h"


//private var
static volatile uint32_t tim1;
static volatile TimeMesureStatus Status;



void appendTimeCounter(void){

	if(Status.t1) tim1++;

		//HAL_GPIO_TogglePin(vl53l0x_POWER_GPIO_Port, vl53l0x_POWER_Pin);
}
void startMesure_ms(uint8_t nr_tim){
	switch(nr_tim){
	case vl53l0x_tim:
		tim1 = 0;
		Status.t1 = 1;
		break;
	case 2:
		Status.t2 = 1;
		break;
	}
}
uint32_t getMesure_ms(uint8_t nr_tim){
	switch(nr_tim){
	case vl53l0x_tim:
		return tim1;
	}
	return 0;
}
uint32_t stopMesure_ms(uint8_t nr_tim){
	switch(nr_tim){
	case vl53l0x_tim:
		{
		Status.t1 = 0;
		return tim1;
		}
	}
	return 0;
}

void timeIt_Start_us(void){
	// reset counter value
	htim2.Instance->CNT = 0;
	//set state
	htim2.State = HAL_TIM_STATE_BUSY;
	//start count
	__HAL_TIM_ENABLE(&htim2);
}

uint32_t timeIt_GetCounter_us(void){
	// stop count
	__HAL_TIM_DISABLE(&htim2);
	// set status
	htim2.State = HAL_TIM_STATE_READY;
	//return counter value
	return htim2.Instance->CNT;

}
