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
#include <wave.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// WAV文件的元数据结构
typedef struct {
    char chunkID[4];       // "RIFF"
    uint32_t chunkSize;    // 文件大小
    char format[4];        // "WAVE"
    char subchunk1ID[4];   // "fmt "
    uint32_t subchunk1Size;// 16 for PCM
    uint16_t audioFormat;  // PCM = 1
    uint16_t numChannels;  // 声道数量
    uint32_t sampleRate;   // 采样�????????????
    uint32_t byteRate;     // 每秒字节�????????????
    uint16_t blockAlign;   // 每样本的字节�????????????
    uint16_t bitsPerSample;// 每样本的位数
    char subchunk2ID[4];   // "data"
    uint32_t subchunk2Size;// 数据大小
} WAVHeader;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// SD卡命�????????
#define CMD0    (0x40+0)  // GO_IDLE_STATE
#define CMD8    (0x40+8)  // SEND_IF_COND
#define CMD17   (0x40+17) // READ_SINGLE_BLOCK
#define CMD55   (0x40+55) // APP_CMD
#define ACMD41  (0x40+41) // SD_SEND_OP_COND
#define CMD58  58
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// SD卡响�????????
#define R1_IDLE_STATE           (1 << 0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint16_t pressed_key = 0x0000;

extern SPI_HandleTypeDef hspi1; // CubeMX生成的SPI句柄，根据你的配置修�????????
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void MX_GPIO_Init(void);
uint32_t tone[] = { 262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};
uint32_t waves[]= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DelayUS(uint32_t Delay) {
//	  uint32_t delayReg;
//	  uint32_t usNum = (uint32_t)(Delay*16);
//
//	  delayReg = 0;
//	  while(delayReg!=usNum) delayReg++;
	  SysTick->LOAD = 72 * Delay;   // 设置定时器重装�??
	  SysTick->VAL = 0x00;        // 清空当前计数�????????????
	  SysTick->CTRL = 0x00000005; // 设置时钟源为HCLK，启动定时器
	  while (!(SysTick->CTRL & 0x00010000))
	    ;                         // 等待计数�????????????0
	  SysTick->CTRL = 0x00000004; // 关闭定时�????????????
}


void Sound(uint16_t frq)
{
	uint32_t time;
    if(frq != 1000)
    {
        time = 500000/((uint32_t)frq);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);

        DelayUS(time);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);

        DelayUS(time);
    }else
    	DelayUS(1000);
}

void Waving(){
	uint16_t pin;
	  for(int i = 0; i < 12; i++){
		  if((pressed_key >> i) & 1 == 1){
			  switch(i){
			  	  case 0:
			  		  pin = GPIO_PIN_3;
			  		  break;
			  	case 1:
			  				  		  pin = GPIO_PIN_3;
			  				  		  break;
			  	case 2:
			  				  		  pin = GPIO_PIN_4;
			  				  		  break;
			  	case 3:
			  				  		  pin = GPIO_PIN_10;
			  				  		  break;
			  	case 4:
			  				  		  pin = GPIO_PIN_11;
			  				  		  break;
			  	case 5:
			  				  		  pin = GPIO_PIN_12;
			  				  		  break;
			  	case 6:
			  				  		  pin = GPIO_PIN_13;
			  				  		  break;
			  	case 7:
			  				  		  pin = GPIO_PIN_14;
			  				  		  break;
			  	case 8:
			  				  		  pin = GPIO_PIN_15;
			  				  		  break;
			  	case 9:
			  				  		  pin = GPIO_PIN_16;
			  				  		  break;
			  	case 10:
			  				  		  pin = GPIO_PIN_17;
			  				  		  break;
			  	case 11:
			  				  		  pin = GPIO_PIN_18;
			  				  		  break;

			  }
			  if(((720000000-(SysTick->VAL))/waves[i])%2==0){
				  HAL_GPIO_WritePin(GPIOB, pin, 1);
			  }else{
				  HAL_GPIO_WritePin(GPIOB, pin, 0);
			  }
		  }

	  }
	  if(SysTick->CTRL & 0x00010000){
		  SysTick->LOAD = 720000000;   // 设置定时器重装�??
		  SysTick->VAL = 0x00;        // 清空当前计数�????????????
		  SysTick->CTRL = 0x00000005; // 设置时钟源为HCLK，启动定时器
	  }
}

// 解析WAV文件�????????????
void parseWAVHeader(const uint8_t *data, WAVHeader *header) {
    memcpy(header->chunkID, data, 4);
    header->chunkSize = *(uint32_t *)(data + 4);
    memcpy(header->format, data + 8, 4);
    memcpy(header->subchunk1ID, data + 12, 4);
    header->subchunk1Size = *(uint32_t *)(data + 16);
    header->audioFormat = *(uint16_t *)(data + 20);
    header->numChannels = *(uint16_t *)(data + 22);
    header->sampleRate = *(uint32_t *)(data + 24);
    header->byteRate = *(uint32_t *)(data + 28);
    header->blockAlign = *(uint16_t *)(data + 32);
    header->bitsPerSample = *(uint16_t *)(data + 34);
    memcpy(header->subchunk2ID, data + 36, 4);
    header->subchunk2Size = *(uint32_t *)(data + 40);
}

uint8_t CRC7(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc = (crc << 1) | (data[i] >> 7);
    if (crc & 0x80) crc ^= 0x09;
    for (uint8_t j = 1; j < 8; j++) {
      crc = (crc << 1) | (data[i] >> (7 - j) & 1);
      if (crc & 0x80) crc ^= 0x09;
    }
  }
  return crc;
}

// 发�?�SD卡命�????????
uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg)
{
  uint8_t response;
  uint8_t command[6];

  command[0] = cmd;
  command[1] = (uint8_t)(arg >> 24);
  command[2] = (uint8_t)(arg >> 16);
  command[3] = (uint8_t)(arg >> 8);
  command[4] = (uint8_t)(arg);
//  command[5] = 0x95; // CRC (CMD0不需要CRC)


  if (cmd != CMD0) {
      uint8_t crc_data[5] = {cmd, (uint8_t)(arg >> 24), (uint8_t)(arg >> 16), (uint8_t)(arg >> 8), (uint8_t)arg};
      command[5] = CRC7(crc_data, 5) | 0x01; // CRC7 + end bit
    } else {
      command[5] = 0x95; // CMD0的CRC
    }

  HAL_SPI_TransmitReceive(&hspi1, command, &response, 1, HAL_MAX_DELAY);
  return response;
}

// 等待R1响应�????????0xFF (SD卡忙) 或非0xFF (SD卡准备好)
uint8_t SD_WaitReady(void)
{
    uint8_t res;
    uint32_t timeout = 500; // 超时时间

    do {
      res = SD_SendCommand(CMD0, 0);
      timeout--;
    } while ((res != 0x01) && timeout > 0 );

    return res;
}

// 初始化SD�????????
uint8_t SD_Initialize(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);

    uint16_t timeout = 1000; // 超时时间
    uint8_t dummy = 0xFF;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
    // 发�?�至�????????74个时�????????
    for (int i = 0; i < 10; i++)
        HAL_SPI_Transmit(&hspi1, &dummy, 1, HAL_MAX_DELAY);

    HAL_Delay(1);
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET); // 根据你的CS引脚修改
    uint8_t response = SD_SendCommand(CMD0, 0);//HAL_SPI_Receive(&hspi1, &response, 1, HAL_MAX_DELAY); // 发�?? CMD0
    // 发�?�CMD0进入IDLE状�??
    while (response != 0x01 && timeout > 0) {
        HAL_SPI_TransmitReceive(&hspi1, &dummy, &response, 1, HAL_MAX_DELAY);
        timeout--;
        if (response != 0xFF){
        	{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
			}
        }
    }
    if (timeout == 0) {
    	return 1; // 初始化失�????????
    }


    // 发�?�CMD8, �????????查SD卡版�????????
    response = SD_SendCommand(CMD8, 0x000001AA);
    if (response != 0x01) return 1; // 初始化失败，不支持CMD8

    // 发�?�ACMD41，初始化SD�????????
    do {
        response = SD_SendCommand(CMD55, 0);
        response = SD_SendCommand(ACMD41, 0x40000000);  // 支持高容量SD�????????
        timeout--;
    } while ((response & R1_IDLE_STATE) && timeout > 0);

    if (timeout == 0) return 1; // 初始化失�????????

    // 发�?�CMD58读取OCR
	response = SD_SendCommand(CMD58, 0);
	if (response != 0x00) return 1; // 初始化失�???????

	uint8_t ocr[4];
	HAL_SPI_Receive(&hspi1, ocr, 4, HAL_MAX_DELAY);

	// �???????查OCR寄存�???????
	if (!(ocr[0] & 0x80)) return 1; // 电源未准备好

	// �???????查CCS�??????? (Card Capacity Status), 判断SD卡类�???????
	if (ocr[0] & 0x40) {
	  // SDHC/SDXC �???????
	  // ...
	} else {
	  // SDSC �???????
	  // ...
	}


    // CS pin high (SD卡未选中)
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

    return 0; // 初始化成�????????
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
  HAL_GPIO_WritePin(GPIOB, Col_3_Out_Pin, 0);
  HAL_GPIO_WritePin(GPIOB, Col_2_Out_Pin, 0);
  HAL_GPIO_WritePin(GPIOB, Col_1_Out_Pin, 0);

//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);

  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);


//  WAVHeader wh;
//  parseWAVHeader(wav_data, &wh);
//
//  SD_Initialize();
//  FATFS FatFs;  // FATFS文件系统对象
//  FRESULT res;  // 操作结果
//
//  // 挂载文件系统
//  res = f_mount(&FatFs, "/", 0);
//  if (res != FR_OK) {
//    // 挂载失败，处理错�???????
//    Error_Handler();
//  }
//
//  FIL MyFile;  // 文件对象
//
//  // 打开文件
//  res = f_open(&MyFile, "single-piano-note-a2_100bpm_C_major.wav", FA_READ);
//  if (res != FR_OK) {
//    // 打开文件失败，处理错�???????
//    Error_Handler();
//  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  SysTick->LOAD = 720000000;   // 设置定时器重装�??
  SysTick->VAL = 0x00;        // 清空当前计数�????????????
  SysTick->CTRL = 0x00000005; // 设置时钟源为HCLK，启动定时器
  for(int i = 0; i < 12; i++){
	  waves[i] = 500000/tone[i];
  }
  uint16_t j;

  pressed_key = 0x0000;

  j = 3;
  uint16_t addr = 44;
  uint16_t freq = 1000;
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
	  HAL_GPIO_EXTI_Callback(Row_4_In_Pin);
	  HAL_GPIO_EXTI_Callback(Row_3_In_Pin);
	  HAL_GPIO_EXTI_Callback(Row_2_In_Pin);
 	  HAL_GPIO_EXTI_Callback(Row_1_In_Pin);

//	  for(int i = 0; i < 12; i++){
//		  if((pressed_key >> i) & 1 == 1){
//			  freq = tone[i];
//		  }else if(i == 11){
//			  freq = 1000;
//		  }
//
//	  }
//
//
//
//	  Sound(freq);
 	  Waving();

//	  i = HAL_GPIO_ReadPin(GPIOB, Row_4_In_Pin);


//	if (wh.numChannels == 2){
//		uint16_t chk = *(uint16_t *)(wav_data + addr);
//		uint8_t left = *(uint8_t *) &chk;
//		uint8_t right = *(uint8_t *) (&chk+2);
//		uint8_t out = left / 2 + right / 2;
//
//		uint16_t pins[] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
//		for (int i = 0; i < 8; i++) {
//			// 使用位运算提取每�?????????????4位的�?????????????
//			uint8_t bit_value = (out >> (7 - i)) & 1;
//			HAL_GPIO_WritePin(GPIOB, pins[i], bit_value);
//		}
//	}
//	addr += 2;
//	if (addr > 460){
//		addr = 44;
//	}
//	DelayUS(1000000 / wh.sampleRate);
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
    if(GPIO_Pin == Row_3_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_3_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0100; // 9
    	}else {
    	  pressed_key = pressed_key & 0xfeff; // 9
    	}
    }
    if(GPIO_Pin == Row_2_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_2_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0020; // 6
    	}else {
    	  pressed_key = pressed_key & 0xffdf; // 6
    	}
    }
    if(GPIO_Pin == Row_1_In_Pin)
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
    if(GPIO_Pin == Row_3_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_3_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0080; // 8
    	}else {
    	  pressed_key = pressed_key & 0xff7f; // 8
    	}
    }
    if(GPIO_Pin == Row_2_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_2_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0010; // 5
    	}else {
    	  pressed_key = pressed_key & 0xffef; // 5
    	}
    }
    if(GPIO_Pin == Row_1_In_Pin)
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
    if(GPIO_Pin == Row_3_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_3_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0040; // 7
    	}else {
    	  pressed_key = pressed_key & 0xffbf; // 7
    	}
    }
    if(GPIO_Pin == Row_2_In_Pin)
    {
    	if(HAL_GPIO_ReadPin(GPIOB, Row_2_In_Pin) != GPIO_PIN_RESET){
    	  pressed_key = pressed_key | 0x0008; // 4
    	}else {
    	  pressed_key = pressed_key & 0xfff7; // 4
    	}
    }
    if(GPIO_Pin == Row_1_In_Pin)
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
