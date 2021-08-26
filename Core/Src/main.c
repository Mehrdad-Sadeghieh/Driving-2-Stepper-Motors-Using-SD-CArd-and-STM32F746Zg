/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
	
	IN stm32f7xx_ll_sdmmc.c line 318 changed.
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "stdio.h"
#include "stdlib.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFSIZE       250
#define PROCESSSIZE    50
#define ReadBuffSize   10000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
FATFS MyFatfs;
FIL MyFile1, MyFile2;
FRESULT FR1, FR2 ,FR3;
UINT ByteCount;

int Start = 1 ,UserKey = 0, j = 0;
int EndReadStep = 0, LoopStepEnd = 48, ProcessStep, ProcessIND, BUFFIND, EndProcessStep, i;

uint8_t BUFF[100000] = {5}, Process[25000] = {5}, ReadBuff[25000] = {5}, GetSize[2] = {5}, FileSize, ReadStep, LoopStep; 
uint32_t OFS1 = 0, OFS2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void delay_us(uint16_t us);
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_SuspendTick();
  HAL_TIM_Base_Start(&htim1);
	
	HAL_SD_Init(&hsd1);
	
  FR3 = f_mount(&MyFatfs, SDPath, 0);
	
	FR1 = f_open(&MyFile1, "CODE.txt", FA_READ | FA_OPEN_ALWAYS);
	FR2 = f_open(&MyFile2, "STM32.txt", FA_WRITE | FA_CREATE_ALWAYS);
	if (FR1 == FR_OK) HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin, GPIO_PIN_SET);

	OFS1 = 0;
	OFS2 = 0;
	
	f_lseek(&MyFile1, OFS1);
	f_read(&MyFile1, GetSize, sizeof(GetSize), &ByteCount);
	OFS1 += 2;
	
	GetSize[0] = GetSize[0] - 48;
	GetSize[1] = GetSize[1] - 48;
	FileSize = 10*GetSize[0] + GetSize[1] - 1;
	
	for (ReadStep = 0; ReadStep <= FileSize; ReadStep++)
	{
		HAL_GPIO_TogglePin(GPIOB, LED_BLUE_Pin);
	  f_lseek(&MyFile1, OFS1);
	  f_read(&MyFile1, BUFF, sizeof(BUFF), &ByteCount);
	  OFS1 += 100000;
	
	  ProcessIND = 0;
		BUFFIND = 0;
		
	  for (ProcessStep = 0; ProcessStep <= 24999; ProcessStep++)
	  {
	    if (BUFF[BUFFIND] == 49 && BUFF[BUFFIND + 1] == 48 && BUFF[BUFFIND+ 2] == 49 && BUFF[BUFFIND + 3] == 48)
	    {
		    Process[ProcessIND] = 1;
		  	ProcessIND = ProcessIND + 1;
	      BUFFIND = BUFFIND + 4;
      }
				
	    if (BUFF[BUFFIND] == 48 && BUFF[BUFFIND + 1] == 49 && BUFF[BUFFIND + 2] == 48 && BUFF[BUFFIND + 3] == 49)
	    {
	    	Process[ProcessIND] = 5;
		  	ProcessIND = ProcessIND + 1;
	      BUFFIND = BUFFIND + 4;
      }
					 
	    if (BUFF[BUFFIND] == 48 && BUFF[BUFFIND + 1] == 48 && BUFF[BUFFIND + 2] == 48 && BUFF[BUFFIND + 3] == 48)
	    {
	    	Process[ProcessIND] = 9;
		  	ProcessIND = ProcessIND + 1;
	      BUFFIND = BUFFIND + 4;
	    }

	    if (BUFF[BUFFIND] == 49 && BUFF[BUFFIND + 1] == 48 && BUFF[BUFFIND + 2] == 48 && BUFF[BUFFIND + 3] == 49)
	    {
	    	Process[ProcessIND] = 2;
		  	ProcessIND = ProcessIND + 1;
	      BUFFIND = BUFFIND + 4;
	    }
					 
	    if (BUFF[BUFFIND] == 49 && BUFF[BUFFIND + 1] == 48 && BUFF[BUFFIND + 2] == 48 && BUFF[BUFFIND + 3] == 48)
	    {
	    	Process[ProcessIND] = 3;
        ProcessIND = ProcessIND + 1;
	      BUFFIND = BUFFIND + 4;
	    }

	    if (BUFF[BUFFIND] == 48 && BUFF[BUFFIND + 1] == 49 && BUFF[BUFFIND + 2] == 49 && BUFF[BUFFIND + 3] == 48)
	    {
	    	Process[ProcessIND] = 4;
		  	ProcessIND = ProcessIND + 1;
	      BUFFIND = BUFFIND + 4;
	    }

	    if (BUFF[BUFFIND] == 48 && BUFF[BUFFIND + 1] == 49 && BUFF[BUFFIND + 2] == 48 && BUFF[BUFFIND + 3] == 48)
	    {
	    	Process[ProcessIND] = 6;
		  	ProcessIND = ProcessIND + 1;
	      BUFFIND = BUFFIND + 4;
      }

	    if (BUFF[BUFFIND] == 48 && BUFF[BUFFIND + 1] == 48 && BUFF[BUFFIND + 2] == 49 && BUFF[BUFFIND + 3] == 48)
	    {
	    	Process[ProcessIND] = 7;
		  	ProcessIND = ProcessIND + 1;
	      BUFFIND = BUFFIND + 4;
	    }

	    if (BUFF[BUFFIND] == 48 && BUFF[BUFFIND + 1] == 48 && BUFF[BUFFIND + 2] == 48 && BUFF[BUFFIND + 3] == 49)
	    {
	    	Process[ProcessIND] = 8;
		  	ProcessIND = ProcessIND + 1;
	      BUFFIND = BUFFIND + 4;
	    }
			
			//if (BUFF[BUFFIND] != 48 || BUFF[BUFFIND] != 49)
			//{
			//	Process[ProcessIND] = 9;
		  //	ProcessIND = ProcessIND + 1;
	     // BUFFIND = BUFFIND + 4;
			//}
    }
		
	  f_lseek(&MyFile2, OFS2);
	  f_write(&MyFile2, Process, sizeof(Process), &ByteCount);
	  OFS2 += 25000;
  }
	
	f_close(&MyFile1);
	f_close(&MyFile2);
	HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_SET);
	delay_us(60000);
	
	FR2 = f_open(&MyFile2, "STM32.txt", FA_READ | FA_OPEN_ALWAYS);
	OFS2 = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	{
		if (Start)
		{
		  for (LoopStep = 0; LoopStep <= FileSize; LoopStep++)
		  {
			  f_lseek(&MyFile2, OFS2);
		  	f_read(&MyFile2, ReadBuff, sizeof(ReadBuff), &ByteCount);
		  	OFS2 = OFS2 + 25000;
			
		  	for (i = 0; i <= 24999; i++)
			  {
		      switch (ReadBuff[i])
          {
            case 1:
		          GPIOA->BSRR = GPIO_PIN_5;                                          // 5
	            GPIOA->BSRR = GPIO_PIN_7;								 													 // 7
		          GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		          GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		          delay_us(500);
	            GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
	            GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
              delay_us(500);			  
        	    break;
       	    case 2:
		          GPIOA->BSRR = GPIO_PIN_5;                                          // 5
		          GPIOA->BSRR = (uint32_t)GPIO_PIN_7 << 16;													 // 7
		          GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		          GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		          delay_us(500);
		          GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
	     	      GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
	            delay_us(500);
              break;
    	      case 3:
		          GPIOA->BSRR = GPIO_PIN_5;                                          // 5
	            GPIOA->BSRR = GPIO_PIN_7;																					 // 7
	            GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		          GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		          delay_us(500);
		          GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
		          delay_us(500);
		          break;
	          case 4:
	            GPIOA->BSRR = (uint32_t)GPIO_PIN_5 << 16;                          // 5
	            GPIOA->BSRR = GPIO_PIN_7;																					 // 7
		          GPIOA->BSRR = GPIO_PIN_4;																					 // 4
	            GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		          delay_us(500);
		          GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
		          GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
		          delay_us(500);
		          break;
	          case 5:
	            GPIOA->BSRR = (uint32_t)GPIO_PIN_5 << 16;                          // 5
	            GPIOA->BSRR = (uint32_t)GPIO_PIN_7 << 16;													 // 7
	            GPIOA->BSRR = GPIO_PIN_4;																					 // 4
	      	    GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		          delay_us(500);
		          GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
		          GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
		          delay_us(500);
		          break;
	          case 6:
		          GPIOA->BSRR = (uint32_t)GPIO_PIN_5 << 16;                          // 5
		          GPIOA->BSRR = GPIO_PIN_7;																					 // 7
		          GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		          GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		          delay_us(500);
		          GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
		          delay_us(500);
		          break;
	    	    case 7:
		          GPIOA->BSRR = GPIO_PIN_5;                                          // 5
		          GPIOA->BSRR = GPIO_PIN_7;																					 // 7
		          GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		          GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		          delay_us(500);
		          GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
		          delay_us(500);
	   	        break;
	         case 8:
		         GPIOA->BSRR = GPIO_PIN_5;                                           // 5
		         GPIOA->BSRR = (uint32_t)GPIO_PIN_7 << 16;				  								 // 7
		         GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		         GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		         delay_us(500);
		         GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
		         delay_us(500);
		         break;
	         case 9:
		         GPIOA->BSRR = GPIO_PIN_5;                                           // 5
		         GPIOA->BSRR = GPIO_PIN_7;																					 // 7
		         GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		         GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		         delay_us(1000);
		         break;
          }
		    }
		  }
		
		  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_RESET);
			Start = 0;
	  } 
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 215;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DETECT_SDIO_Pin */
  GPIO_InitStruct.Pin = DETECT_SDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DETECT_SDIO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1 ,0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < us)
	{
		
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
