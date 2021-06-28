/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "dwt_Delay.h"
#include "vl53l0x.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t timeIt_GetCounter_us(void);
void timeIt_Start(void);
// only for work delete in the end
void checkDev_sendToTerminal(uint8_t addr);
void deviceList_sendToTerminal(void);
// -------------------------------------//
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DWT_Delay_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart1, (uint8_t *)"startuje init\n\r", 15, 50);
  uint8_t div_tab[100] = {0};

  uint8_t wynik =(uint8_t)vl53l0x_Init(0);
  HAL_UART_Transmit(&huart1, (uint8_t *)"ruszyło\n\r", 9, 50);
  uint16_t mesure_score = 0;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  //HAL_TIM_Base_Start(&htim2);
 // uint32_t counter_val = 0;
 // float convert = 0.0;
  char graphic_value[70] = {0};
  uint32_t graphic_len =0;
  bool dir = 0;
  while (1)
  {

	  mesure_score = vl53l0x_ReadRangeSingleMillimeters(0);
	  //second mesure
	  mesure_score += vl53l0x_ReadRangeSingleMillimeters(0);
	  //mean
	  mesure_score /= 2;

	  sprintf((char *)div_tab, "wynik pomiaru : %d \n\r", mesure_score);

	 // HAL_UART_Transmit(&huart1, div_tab, 30, 10);
	  graphic_len = (mesure_score/5);
	  for(int i = 0 ; i < 70; i++){
		  if(i < graphic_len)graphic_value[i] = '=';
		  else graphic_value[i] = 0;
	  }

	  //timeIt_Start();
	  //convert = (counter_val/1000.0);
	  //counter_val = timeIt_GetCounter_us();
	  sprintf((char *)div_tab, "graficznie: %s\n\r", graphic_value);
	  HAL_UART_Transmit(&huart1, div_tab, graphic_len+15, 10);

	  DWT_Delay_us_(100000);

	  //htim1.Instance->CCR1 =180;
	  uint16_t temp = htim1.Instance->CCR1;
	  if(temp < 200 && dir ==0) htim1.Instance->CCR1 += 1;
	  else dir = 1;

	  if( temp > 100 && dir == 1)htim1.Instance->CCR1 -= 1;
	  else dir = 0;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void timeIt_Start(void){
	__HAL_TIM_ENABLE(&htim2);
	htim2.State = HAL_TIM_STATE_BUSY;
}

uint32_t timeIt_GetCounter_us(void){
	uint32_t temp;
	__HAL_TIM_DISABLE(&htim2);
	temp = htim2.Instance->CNT;
	//reset value todo: mabe put reste value in start?
	htim2.Instance->CNT = 0;
	htim2.State = HAL_TIM_STATE_READY;
	return temp;
}

// function to remove after end project//-----------------------------------------------------------//
void checkDev_sendToTerminal(uint8_t addr){
	uint8_t div_tab[30] = {0};
	uint8_t recive_data = 10;
	uint8_t status = (uint8_t)HAL_I2C_Mem_Read(&hi2c1, (addr<<1), 0x1, 1, &recive_data, 1, 20);
	sprintf((char *) div_tab, "\n\r status: %d\n\r rejstr: %d\n\r", status, recive_data);
	HAL_UART_Transmit(&huart1, (uint8_t *)"\n\r", 3, 50);
	uint8_t count = 0;
	while(div_tab[count++] != 0);
	HAL_UART_Transmit(&huart1, div_tab, count, 50);
}

void deviceList_sendToTerminal(void){

	  for(int i = 0 ;i < 128; i++){
		  uint8_t div_tab[10];
		  if(HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 5) == HAL_OK){
			  sprintf((char *)div_tab, "dev: %d", i);
			  HAL_UART_Transmit(&huart1, div_tab, 10, 50);
			  HAL_UART_Transmit(&huart1, (uint8_t *)"\n\r", 3, 50);

		  }

	  }
}
//---------------------------------------------------------------------------------------------------//
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
