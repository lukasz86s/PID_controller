/*
 * dwt_Delay.c
 *
 *  Created on: 6 cze 2021
 *      Author: fet
 */
#include "dwt_Delay.h"

uint32_t DWT_Delay_Init(void){
	// disable TRC
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
	//enable TRC
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	//Disable clock cycle counter
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
	// Enable clock cycle counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	// reset the clock cycle counter val
	DWT->CYCCNT = 0;

	// wait 3 cycles
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");

	// check if clock cycle counter has started
	if(DWT->CYCCNT){
		return 0; // clock started
	}
	else
	{
		return 1; // clock not started
	}
}



