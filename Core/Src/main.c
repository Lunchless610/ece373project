/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <wave.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// SD卡响�?????????
#define R1_IDLE_STATE           (1 << 0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint16_t pressed_key = 0x0000;


extern SPI_HandleTypeDef hspi1; // CubeMX生成的SPI句柄，根据你的配置修�?????????
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void MX_GPIO_Init(void);
uint32_t tone[] = { 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
uint32_t waves[]= {1908, 1805, 1701, 1608, 1515, 1433, 1351, 1276, 1205, 1136, 1073, 1012};
uint32_t us = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t e_time[12] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
uint32_t s_time[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t finished = 0xFF;

uint8_t ADSR(uint8_t harmonics_wave, int note_on, uint32_t start_time, uint32_t note_off_time, double a, double d, double s, double r){
	// 初始化ADSR参数
	uint32_t attack_time = a * 1000 * 100;//10 * 100;  // 快速攻击
	uint32_t decay_time = d * 1000 * 100;//200 * 100;    // 衰减
	double sustain_level = s; //0.7; // 持续音量
	uint32_t release_time = r * 1000 * 100;//;500 * 100;  // 释放

	// 计算当前音量
	uint32_t current_time = us - start_time;
	if (start_time == 0xFFFFFFFF) return 0;
	double envelope = 0.0;
	uint8_t final_wave = harmonics_wave * envelope;

	if (current_time < attack_time) {
		final_wave = harmonics_wave * current_time / attack_time;
	} else if (current_time < attack_time + decay_time) {
	    envelope = 1.0 - ((current_time - attack_time) / decay_time) * (1.0 - sustain_level);
	    final_wave = harmonics_wave * envelope;
	} else if (note_on) {
	    envelope = sustain_level;
	    final_wave = harmonics_wave * envelope;
	} else {
		uint32_t release_elapsed = us - note_off_time;
	    envelope = sustain_level * (1.0 - release_elapsed / release_time);
	    if (envelope < 0) envelope = 0;
	    final_wave = harmonics_wave * envelope;
	}

	return final_wave;
}

uint8_t mask[8] ={0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

uint8_t Waving(){
	uint16_t result = 0;
	  for(int i = 0; i < 12; i++){
		  uint8_t note = 0;
		  uint8_t base = 0x40;
		  float amplitudes[] = {1.0, 0.4, 0.2, 0.1, 0.05, 0.01, 0.005, 0.001};
		  for (int j = 1; j < 8; j += 1){
			  if((us / (waves[i] / j)) % 2==0){
				  note += base * amplitudes[j-1];
			  }
		  }

		  int note_on = ((pressed_key >> i) & 1) == 1;
		  if(note_on){
			  if (s_time[i] == 0){
				  s_time[i] = us;
			  }
		  }
		  else if (s_time[i] != 0 && e_time[i] == 0xFFFFFFFF){
			  e_time[i] = us;
		  }

		  if (s_time[i] != 0){
			  note = ADSR(note, note_on, s_time[i], e_time[i], 0.01, 0.1, 0.5, 0.3);
			  result += note;
		  }
		  if (e_time[i] != 0xFFFFFFFF && (e_time[i] + 0.3 * 1000 * 100 < us || s_time[i] > us)){
		  			  s_time[i] = 0;
		  			  e_time[i] = 0xFFFFFFFF;
		  }
	  }
	  return result;
}

void Sounding(uint8_t waving){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (waving >> 7) & 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (waving >> 6) & 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (waving >> 5) & 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, (waving >> 4) & 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (waving >> 3) & 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (waving >> 2) & 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (waving >> 1) & 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, waving & 1);
}

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  SysTick->LOAD = 720 - 1;   // 设置定时器重装�??
  SysTick->VAL = 0x00;        // 清空当前计数�?????????????
  SysTick->CTRL = 0x07; // 设置时钟源为HCLK，启动定时器
  for(int i = 0; i < 12; i++){
	  waves[i] = 50000 / tone[i];
  }

  pressed_key = 0x0000;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  currentMillis = us;
	  if (previousMillis + 3 < currentMillis){
		 HAL_GPIO_EXTI_Callback(Row_4_In_Pin);
		 HAL_GPIO_EXTI_Callback(Row_3_In_Pin);
		 HAL_GPIO_EXTI_Callback(Row_2_In_Pin);
	 	 HAL_GPIO_EXTI_Callback(Row_1_In_Pin);
	 	 previousMillis = currentMillis;
	 	 uint32_t ss = 0;
	 	 for (int i =0; i<12;i+=1){
	 		 ss += s_time[i];
	 	 }
	 	Sounding(Waving());
	 	if (pressed_key == 0 && ss == 0){
		  us = 0x00;
		  currentMillis = 0;
		  previousMillis = 0;
		  Sounding(0);
	 	}

	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//  currentMillis = HAL_GetTick();
//  if (currentMillis - previousMillis > 1) {
  /*Configure GPIO pins : PB6 PB7 PB8 PB9 to GPIO_INPUT*/
//    GPIO_InitStructPrivate.Pin = Row_4_In_Pin|Row_3_In_Pin|Row_2_In_Pin|Row_1_In_Pin;
//    GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
//    GPIO_InitStructPrivate.Pull = GPIO_PULLDOWN;
//    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_MEDIUM;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);

    HAL_GPIO_WritePin(GPIOB, Col_3_Out_Pin, 1);
    HAL_GPIO_WritePin(GPIOB, Col_2_Out_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, Col_1_Out_Pin, 0);
    if(GPIO_Pin == Row_4_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_4_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0800; // #

    	}else {
    	  pressed_key = pressed_key & 0xf7ff; // #
    	}
    }
    else if(GPIO_Pin == Row_3_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_3_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0100; // 9
    	}else {
    	  pressed_key = pressed_key & 0xfeff; // 9
    	}
    }
    else if(GPIO_Pin == Row_2_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_2_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0020; // 6
    	}else {
    	  pressed_key = pressed_key & 0xffdf; // 6
    	}
    }
    else if(GPIO_Pin == Row_1_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_1_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0004; // 3
    	}else {
    	  pressed_key = pressed_key & 0xfffb; // 3
    	}
    }

    HAL_GPIO_WritePin(GPIOB, Col_3_Out_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, Col_2_Out_Pin, 1);
    HAL_GPIO_WritePin(GPIOB, Col_1_Out_Pin, 0);
    if(GPIO_Pin == Row_4_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_4_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0400; // 0
    	}else {
    	  pressed_key = pressed_key & 0xfbff; // 0
    	}
    }
    else if(GPIO_Pin == Row_3_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_3_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0080; // 8
    	}else {
    	  pressed_key = pressed_key & 0xff7f; // 8
    	}
    }
    else if(GPIO_Pin == Row_2_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_2_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0010; // 5
    	}else {
    	  pressed_key = pressed_key & 0xffef; // 5
    	}
    }
    else if(GPIO_Pin == Row_1_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_1_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0002; // 2
    	}else {
    	  pressed_key = pressed_key & 0xfffd; // 2
    	}
    }

    HAL_GPIO_WritePin(GPIOB, Col_3_Out_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, Col_2_Out_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, Col_1_Out_Pin, 1);
    if(GPIO_Pin == Row_4_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_4_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0200; // *
    	}else {
    	  pressed_key = pressed_key & 0xfdff; // *
    	}
    }
    else if(GPIO_Pin == Row_3_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_3_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0040; // 7
    	}else {
    	  pressed_key = pressed_key & 0xffbf; // 7
    	}
    }
    else if(GPIO_Pin == Row_2_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_2_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0008; // 4
    	}else {
    	  pressed_key = pressed_key & 0xfff7; // 4
    	}
    }
    else if(GPIO_Pin == Row_1_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_1_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0001; // 1
    	}else {
    	  pressed_key = pressed_key & 0xfffe; // 1
    	}
    }

    HAL_GPIO_WritePin(GPIOB, Col_3_Out_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, Col_2_Out_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, Col_1_Out_Pin, 0);

    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);

    /*Configure GPIO pins : PB6 PB7 PB8 PB9 back to EXTI*/
//    GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING;
//    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);
//    previousMillis = currentMillis;
//  }
}
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
