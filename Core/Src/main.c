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
#include <stdint.h>
#include <string.h>
#include <wave.h>
#include "sd.h"
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
GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint8_t pressed_key = 0;

extern SPI_HandleTypeDef hspi1; // CubeMX生成的SPI句柄，根据你的配置修�????
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void MX_GPIO_Init(void);
uint32_t tone[] = {247,262,294,330,349,392,440,294,523,587,659,698,784,1000};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DelayUS(uint32_t Delay) {
	  uint32_t delayReg;
	  uint32_t usNum = (uint32_t)(Delay*16);

	  delayReg = 0;
	  while(delayReg!=usNum) delayReg++;
}


void Sound(uint16_t frq)
{
	uint32_t time;
    if(frq != 1000)
    {
        time = 500000/((uint32_t)frq);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
        DelayUS(time);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
        DelayUS(time);
    }else
    	DelayUS(1000);
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);

//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);

  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  WAVHeader wh;
  //parseWAVHeader(wav_data, &wh);

  SD_Initialize();
  FATFS FatFs;  // FATFS文件系统对象
  FRESULT res;  // 操作结果

  // 挂载文件系统
  res = f_mount(&FatFs, "/", 1);
  if (res != FR_OK) {
    // 挂载失败，处理错�???
    Error_Handler();
  }

  FIL MyFile;  // 文件对象

  // 打开文件
  res = f_open(&MyFile, "single-piano-note-a2_100bpm_C_major.wav", FA_READ);
  if (res != FR_OK) {
    // 打开文件失败，处理错�???
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t addr = 44;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    for(i = 0; i < tone[j]/4 ; i ++){
//		  Sound(tone[j]);
//	  }
//
//	  j ++;
//
//	  if(j > 12){
//		  j = 0;
//	  }
	if (wh.numChannels == 2){
//		uint16_t chk = *(uint16_t *)(wav_data + addr);
//		uint8_t left = *(uint8_t *) &chk;
//		uint8_t right = *(uint8_t *) (&chk+2);
//		uint8_t out = left / 2 + right / 2;
//
//		uint16_t pins[] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
//		for (int i = 0; i < 8; i++) {
//			// 使用位运算提取每�??????4位的�??????
//			uint8_t bit_value = (out >> (7 - i)) & 1;
//			HAL_GPIO_WritePin(GPIOB, pins[i], bit_value);
//		}
	}
	addr += 2;
	if (addr > 460){
		addr = 44;
	}
	DelayUS(1000000 / wh.sampleRate);
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
  currentMillis = HAL_GetTick();
  if (currentMillis - previousMillis > 10) {
    /*Configure GPIO pins : PB6 PB7 PB8 PB9 to GPIO_INPUT*/
    GPIO_InitStructPrivate.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
    {
      pressed_key = 35; //ASCII value of #
    }
    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
    {
      pressed_key = 57; //ASCII value of 9
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
    }
    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
    {
      pressed_key = 54; //ASCII value of 6
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
    }
    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
    {
      pressed_key = 51; //ASCII value of 3
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
    {
      pressed_key = 48; //ASCII value of 0
    }
    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
    {
      pressed_key = 56; //ASCII value of 8
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
      // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
    }
    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
    {
      pressed_key = 53; //ASCII value of 5
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
    }
    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
    {
      pressed_key = 50; //ASCII value of 2
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
    if(GPIO_Pin == GPIO_PIN_6 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
    {
      pressed_key = 42; //ASCII value of *
    }
    else if(GPIO_Pin == GPIO_PIN_7 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
    {
      pressed_key = 55; //ASCII value of 7
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
    }
    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
    {
      pressed_key = 52; //ASCII value of 4
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
    }
    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
    {
      pressed_key = 49; //ASCII value of 1
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
      // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);

    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
    /*Configure GPIO pins : PB6 PB7 PB8 PB9 back to EXTI*/
    GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructPrivate.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);
    previousMillis = currentMillis;
  }
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
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
	  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
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
