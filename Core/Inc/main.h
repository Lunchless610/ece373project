/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Col_3_Out_Pin GPIO_PIN_3
#define Col_3_Out_GPIO_Port GPIOB
#define Col_2_Out_Pin GPIO_PIN_4
#define Col_2_Out_GPIO_Port GPIOB
#define Col_1_Out_Pin GPIO_PIN_5
#define Col_1_Out_GPIO_Port GPIOB
#define Row_4_In_Pin GPIO_PIN_6
#define Row_4_In_GPIO_Port GPIOB
#define Row_3_In_Pin GPIO_PIN_7
#define Row_3_In_GPIO_Port GPIOB
#define Row_2_In_Pin GPIO_PIN_8
#define Row_2_In_GPIO_Port GPIOB
#define Row_1_In_Pin GPIO_PIN_9
#define Row_1_In_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
