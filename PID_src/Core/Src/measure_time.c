/*
 * measure_time.c
 *
 *  Created on: 30 cze 2021
 *      Author: fet
 */


#include "measure_time.h"
#include "tim.h"


//private var
static volatile uint32_t tim0, tim1, tim2;
static volatile TimeMesureStatus Status;
static callback_func *timers_handler = (void *)0;



void appendTimeCounter(void){

	if(Status.t0) tim0++;
	if(Status.t1) tim1++;
	if(Status.t2) tim2++;
	if(timers_handler != (void*)0){
		timers_handler();
	}

		//HAL_GPIO_TogglePin(vl53l0x_POWER_GPIO_Port, vl53l0x_POWER_Pin);
}
void startMesure_ms(uint8_t nr_tim){
	switch(nr_tim){
	case VL53L0X_TIM:
		tim0 = 0;
		Status.t0 = 1;
		break;
	case BUTTON_TIM:
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
		case VL53L0X_TIM:
			return tim0;
			break;
		case BUTTON_TIM:
			return tim1;
			break;
		case 2:
			return tim2;
			break;
	}
	return 0;
}
uint32_t stopMesure_ms(uint8_t nr_tim){
	switch(nr_tim){
		case VL53L0X_TIM:
			Status.t0 = 0;
			return tim0;
		case BUTTON_TIM:
			Status.t1 = 0;
			return tim1;
		case 2:
			Status.t2 = 0;
			return tim2;

	}
	return 0;
}
uint8_t getMeasure_status(uint8_t nr_tim){
	switch(nr_tim){
		case VL53L0X_TIM:
			return Status.t0;

		case BUTTON_TIM:
			return Status.t1;

		case 2:
			return Status.t2;

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
void measure_time_callback_register(callback_func *func){
	timers_handler = func;
}
