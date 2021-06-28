/*
 * DWT_Delay.h
 *
 *  Created on: 6 cze 2021
 *      Author: fet
 */

#ifndef INC_DWT_DELAY_H_
#define INC_DWT_DELAY_H_

#include "stm32f4xx_hal.h"

uint32_t DWT_Delay_Init(void);

__STATIC_INLINE void DWT_Delay_us_(volatile uint32_t u32_microseconds){
	uint32_t initial_ticks = DWT->CYCCNT;
	uint32_t ticks = (HAL_RCC_GetHCLKFreq()/ 1000000);
	u32_microseconds *= ticks;
	while((DWT->CYCCNT - initial_ticks) < u32_microseconds);

}


#endif /* INC_DWT_DELAY_H_ */
