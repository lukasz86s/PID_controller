/*
 * measure_time.h
 *
 *  Created on: 30 cze 2021
 *      Author: fet
 */

#ifndef INC_MEASURE_TIME_H_
#define INC_MEASURE_TIME_H_

#include "stm32f4xx_hal.h"

enum{
	VL53L0X_TIM,
	BUTTON_TIM,
	numberOfTim
};
typedef struct _TimeMesureStatus{
	volatile uint8_t t0 :1;
	volatile uint8_t t1 :1;
	volatile uint8_t t2 :1;
	volatile uint8_t t3 :1;

}TimeMesureStatus;
//
typedef void callback_func(void);
//functions
/* add this func to you timer callback IT (period 1ms) */
void appendTimeCounter(void);
//nr_tim - number of software timer
void startMesure_ms(uint8_t nr_tim);
uint32_t stopMesure_ms(uint8_t nr_tim);
uint32_t getMesure_ms(uint8_t nr_tim);
//get measured ticks
uint32_t timeIt_GetCounter_us(void);
// start measure time in us
void timeIt_Start_us(void);
void measure_time_callback_register(callback_func *func);
uint8_t getMeasure_status(uint8_t nr_tim);


#endif /* INC_MEASURE_TIME_H_ */
