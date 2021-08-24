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
#include <stdlib.h>
#include "dwt_Delay.h"
#include "vl53l0x.h"
#include "measure_time.h"
#include "my_PID.h"
#include "../LCD_hd44780/LCD_init.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REPEAT_MEASURE 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum _menu{
		START,
		TARGET,
		KP,
		KI,
		KD,
		MENU_SIZE
	};
struct{
	uint16_t score;
	uint16_t score_old;
	uint16_t score_stored;
	uint16_t button :1;
}encoder;
char * menu_text[MENU_SIZE] = {"Start", "Target", "Kp", "Ki", "Kd"};

char lcd_text[17] = {"0\0"};

volatile uint16_t measure_score = 0;

float PID = 0.0;

float Kp = 1.4;
float Ki = 1.0;
float Kd = 0.9;
uint8_t pid_status = 0;

float err;

float measured_time_s = 0.0;
// target to get in mm
float target = 120.0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */


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
  MX_TIM10_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  init_lcd(DWT_Delay_us_);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //deviceList_sendToTerminal();

  // servo initial value for leveling
  htim1.Instance->CCR1 = 150;
  // start PWM to control servo
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  __HAL_TIM_CLEAR_FLAG(&htim10, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim10);
  //start channels 1 and 2 in timer3 to work with encoder
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);
  //todo: nie zgłasza przerwań od przepelnienia channel2

  //HAL_TIM_Base_Start(&htim2);

  //uint8_t div_tab[100] = {0};
 // HAL_UART_Transmit(&huart1, (uint8_t *)"startuje init\n\r", 15, 50);
  lcd_str("start vl53l0x");
  // power on
  HAL_GPIO_WritePin(vl53l0x_POWER_GPIO_Port, vl53l0x_POWER_Pin, RESET);
  // wait 35 ms
  DWT_Delay_us_(35000);
  // init vl53l0x
  uint8_t init = vl53l0x_Init(0);
  //sprintf((char *)div_tab, "init: %d\n\r", init);
  //HAL_UART_Transmit(&huart1, div_tab, 10, 50);
  sprintf((char*)lcd_text, "wynik init: %d", init);
  lcd_cls();
  lcd_str(lcd_text);
  DWT_Delay_us_(1000000);
  // servo power on
  HAL_GPIO_WritePin(SERVO_POWER_GPIO_Port, SERVO_POWER_Pin, RESET);

  sprintf((char*)lcd_text, menu_text[START]);
  lcd_cls();
  lcd_str(lcd_text);

  // start first measure time
  timeIt_Start_us();

  while (1)
  {
	  measure_score = 0;
	 for(uint8_t i = 0; i < REPEAT_MEASURE; i++)
	  {
		 measure_score += vl53l0x_ReadRangeSingleMillimeters(0);
	  }

	  // mean
	  measure_score /= REPEAT_MEASURE;

	  err = target - measure_score;

	  // get time in second
	  measured_time_s = (timeIt_GetCounter_us()/1000000.0);

	  // get pid value
	  PID = get_PID(err, measured_time_s, Kp, Ki, Kd);
	  htim1.Instance->CCR1 = 140 + (PID/10);
	  //DWT_Delay_us_(1000000);

	  timeIt_Start_us();
	  if(!HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)){
		  static uint8_t button;
		  if(!HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) && button){
			  encoder.button ^= 1;
		  	  button = 0;
		  }else{
		  button = 1;
		  }
	  }

	  encoder.score = htim3.Instance->CNT;

	  //simple menu
	  // if encoder has shifted
	  if(encoder.score != encoder.score_old){
		  int32_t temp;
		  // if button is 1 then change positon of menu, else change value of selected positon
		  if(!encoder.button){
			  // if encoder was used to change valuse of selected positon, reset it to positon value
			  if(abs(encoder.score_stored - encoder.score) > 4)
				  htim3.Instance->CNT = encoder.score_stored;
			  else
				  encoder.score_stored = encoder.score;
		  }
		  else{
			  temp =  encoder.score - encoder.score_old;
		  }

		  lcd_locate(0, 0);
		  lcd_cls();
		  switch((encoder.score_stored/4)%MENU_SIZE){
		  case START:
			  sprintf((char*)lcd_text, menu_text[START]);
			  lcd_str(lcd_text);
			  lcd_locate(1, 0);
			  if(encoder.button){
				  if(abs(temp) >= 4)
					  pid_status ^= 1;
			  }
			  if(pid_status == 0)
				  sprintf((char*)lcd_text, "OFF");
			  else
				  sprintf((char*)lcd_text, "ON");
			  lcd_str(lcd_text);
			  break;
		  case TARGET:
			  sprintf((char*)lcd_text, menu_text[TARGET]);
			  lcd_str(lcd_text);
			  lcd_locate(1, 0);
			  sprintf((char*)lcd_text, "%.0f", target);
			  lcd_str(lcd_text);
			  if(encoder.button){
				  if(temp >= 4)
					  target += 1.0;
				  else if(temp <= -4)
					  target -= 1.0;
			  }
			  break;
		  case KP:
			  sprintf((char*)lcd_text, menu_text[KP]);
			  lcd_str(lcd_text);
			  lcd_locate(1, 0);
			  sprintf((char*)lcd_text, "%.1f", Kp);
			  if(encoder.button){
				  if(temp >= 4)
					  Kp += 0.1;
				  else if(temp <= -4)
					  Kp -= 0.1;
			  }
			  lcd_str(lcd_text);
			  break;
		  case KI:
			  sprintf((char*)lcd_text, menu_text[KI]);
			  lcd_str(lcd_text);
			  lcd_locate(1, 0);
			  sprintf((char*)lcd_text, "%.1f", Ki);
			  if(encoder.button){
				  if(temp >= 4)
					  Ki += 0.1;
				  else if(temp <= -4)
					  Ki -= 0.1;
			  }
			  lcd_str(lcd_text);
			  break;
		  case KD:
			  sprintf((char*)lcd_text, menu_text[KD]);
			  lcd_str(lcd_text);
			  lcd_locate(1, 0);
			  sprintf((char*)lcd_text, "%.1f", Kd);
			  if(encoder.button){
				  if(temp >= 4)
					  Kd += 0.1;
				  else if(temp <= -4)
					  Kd -= 0.1;
			  }
			  lcd_str(lcd_text);
			  break;
		  }

	  }
	  encoder.score_old = encoder.score;
	  //lcd_locate(0, 0);

	  // sprintf((char *)div_tab, "P: %f\n\rI: %f\n\rD: %f\n\r", P, I, D);

	  //sprintf((char *)div_tab, "err: %f\n\r ", err);
	  //sprintf((char*)div_tab, "err: %d\n\r ", measure_score);

	  //HAL_UART_Transmit(&huart1, div_tab, 30, 20);

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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	appendTimeCounter();
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
