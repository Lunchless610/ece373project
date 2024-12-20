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
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Delay.h"
#include "lcd1602.h"
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LCD_init(); // BJ LCD initiate
  // uchar startStr2[] = "SJTU JI - LAB3";
  uchar Str1[] = {"voltage:"};
  uchar Space = ' '; // LCDÏÔÊ¾¡®£º¡¯
  uchar dot = '.';   // LCDÏÔÊ¾".";
  uchar unit = 'v';  // LCDÏÔÊ¾"v";
  uchar Num0 = '0';  // LCDÏÔÊ¾Êý×Ö¡°0¡±

  uint16_t AD_Value; // ±£´æAD²ÉÑùµÃµ½µÄÖµ
  float voltage;     // ±£´æ¸¡µã
  uint8_t vol0;
  uint8_t vol1;
  uint8_t vol2;
  uint8_t volt0;
  uint8_t volt1;
  uint8_t volt2;
  uint8_t receiveData;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    HAL_ADC_Start(&hadc1); // Æô¶¯ADC×ª»»
	    HAL_ADC_PollForConversion(
	        &hadc1, 10); // µÈ´ý×ª»»Íê³É£¬µÚ¶þ¸ö²ÎÊý±íÊ¾³¬Ê±Ê±¼ä£¬µ¥Î»ms.
	    if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) {
	      AD_Value = HAL_ADC_GetValue(&hadc1); // ¶ÁÈ¡ADC×ª»»Êý¾Ý£¬Êý¾ÝÎª12Î»
	    }

	    voltage =
	        (float)AD_Value / 4095 * 3.3; // ½«ADÖµÏßÐÔ±ä»»µ½0~3.3µÄ·¶Î§£¬±íÊ¾µçÑ¹

	    volt0 = (uchar)((int)(voltage * 100) % 100); // Ð¡ÊýºóÃæÁ½Î»
	    volt1 = (uchar)((int)(voltage * 10) % 10);          // Ð¡ÊýºóÃæµÚÒ»Î»
	    volt2 = (uchar)((int)(voltage * 100) % 10);                 // Ð¡ÊýºóÃæµÚ¶þÎ»

	    /*ÒÔÏÂLCDÏÔÊ¾µçÑ¹AD²ÉÑùÖµ*/
	    LCD_Display_String(0, 0, Str1); // ÏÔÊ¾"voltage:"
	    LCD_Display_Char(unit, 13, 0);  // ÏÔÊ¾¡±v"
	    LCD_Display_Char(Space, 8, 0);  // ÏÔÊ¾¡±£º¡°
	    LCD_Display_Char(dot, 10, 0);   // ÏÔÊ¾¡°.¡±

	    vol0 = Num0 + (uchar)((int)(voltage)); // Ä£ÄâµçÑ¹µÄÕûÊý²¿·ÖÖµ
	    vol1 = Num0 + volt1; // Ä£ÄâµçÑ¹µÄÐ¡ÊýºóÃæµÚÒ»Î»
	    vol2 = Num0 + volt2; // Ä£ÄâµçÑ¹µÄÐ¡ÊýºóÃæµÚ¶þÎ»

	    LCD_Display_Char(vol0, 9, 0); // ÏÔÊ¾µçÑ¹ÖµµÄÕûÊý²¿·Ö
	    LCD_Display_Char(vol1, 11, 0); // ÏÔÊ¾µçÑ¹ÖµµÄÐ¡ÊýµÚÒ»Î»²¿·Ö
	    LCD_Display_Char(vol2, 12, 0); // ÏÔÊ¾µçÑ¹ÖµµÄÐ¡ÊýµÚ¶þÎ»²¿·Ö

	    Delay_ms(100);

	    if(HAL_UART_Receive(&huart1, &receiveData, 8, 0xFFFFFFFF) == HAL_OK){
	    	if(receiveData == 'A'){
	    		HAL_UART_Transmit(&huart1, &vol0, 8, 0xFFFFFFFF);
	    		HAL_UART_Transmit(&huart1, &vol1, 8, 0xFFFFFFFF);
	    		HAL_UART_Transmit(&huart1, &vol2, 8, 0xFFFFFFFF);
	    	}else if(receiveData == 'B'){
	    		HAL_UART_Transmit(&huart1, &volt0, 8, 1000);
	    		HAL_UART_Transmit(&huart1, &volt1, 8, 1000);
	    		HAL_UART_Transmit(&huart1, &volt2, 8, 1000);
	    	}else{
	    		uint8_t temp0='S';
	    		HAL_UART_Transmit(&huart1, &temp0, 8, 1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
