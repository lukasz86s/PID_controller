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
	case 1:
		Status.t1 = 1;
		break;
	case 2:
		Status.t2 = 1;
		break;
	}
}
uint32_t getMesure_ms(uint8_t nr_tim){
	switch(nr_tim){
	case 1:
		return tim1;
	}
	return 0;
}
uint32_t stopMesure_ms(uint8_t nr_tim){
	uint32_t temp;
	switch(nr_tim){
	case 1:
		{
		Status.t1 = 0;
		temp = tim1;
		tim1 = 0;
		return temp;
		}
	}
	return 0;
}

void timeIt_Start_us(void){
	__HAL_TIM_ENABLE(&htim2);
	htim2.State = HAL_TIM_STATE_BUSY;
}

uint32_t timeIt_GetCounter_us(void){
	uint32_t temp;
	__HAL_TIM_DISABLE(&htim2);
	temp = htim2.Instance->CNT;
	//reset value todo: maybe put reset value in start?
	htim2.Instance->CNT = 0;
	htim2.State = HAL_TIM_STATE_READY;
	return temp;
}
