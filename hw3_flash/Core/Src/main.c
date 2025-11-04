/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    char      msg[50];
    float     num1;
    uint32_t  num2;
    uint16_t  num3;
    uint64_t  num4;
} test_t;
typedef uint64_t data_NVRAM_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_USER_PAGE         (9U)
#define FLASH_USER_NB_PAGES     (1U)
#define FLASH_USER_START_ADDR   (FLASH_BASE + FLASH_PAGE_SIZE * FLASH_USER_PAGE)
#define FLASH_USER_END_ADDR     (FLASH_USER_START_ADDR + FLASH_USER_NB_PAGES * FLASH_PAGE_SIZE - 1)



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t WriteFlag = 0;

  union NVRAM_PAGE {
            test_t test;
            data_NVRAM_t data[FLASH_PAGE_SIZE / sizeof(data_NVRAM_t)];
          } DevNVRAM_Page;

  memset(DevNVRAM_Page.data, 0, sizeof(DevNVRAM_Page.data));
  DevNVRAM_Page.test.num1 = 3.14;
  DevNVRAM_Page.test.num2 = 0xBEEFBEEF;
  DevNVRAM_Page.test.num3 = 0xBABE;
  DevNVRAM_Page.test.num4 = 0xDEADBEEFDEADBEEF;
  strcpy(DevNVRAM_Page.test.msg, "MEMORY TEST MESSAGE");


  while (1)
  {
    if(HAL_GPIO_ReadPin(BTN_ON_BOARD_GPIO_Port, BTN_ON_BOARD_Pin) == GPIO_PIN_SET && !WriteFlag)
    {
      FLASH_EraseInitTypeDef EraseInitStruct;
      uint32_t PageError = 0;
      uint32_t Address = FLASH_USER_START_ADDR;


      if(Address < FLASH_USER_END_ADDR && Address > (FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE))
      {
        while (1)
        {
          HAL_GPIO_TogglePin(LED_ON_BOARD_GPIO_Port, LED_ON_BOARD_Pin);
          HAL_Delay(500);
        }
      }

      HAL_FLASH_Unlock();
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

      EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
      EraseInitStruct.Page      = FLASH_USER_PAGE;
      EraseInitStruct.NbPages   = FLASH_USER_NB_PAGES;


      /*Стиание*/
      if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
      {
        while (1)
        {
          HAL_GPIO_TogglePin(LED_ON_BOARD_GPIO_Port, LED_ON_BOARD_Pin);
          HAL_Delay(500);
        }
      }
      /*Запись*/
      for(uint32_t i = 0; Address < FLASH_USER_END_ADDR; i++)//while(Address < FLASH_USER_END_ADDR)
      {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, DevNVRAM_Page.data[i]) == HAL_OK)
        {
          Address += sizeof(data_NVRAM_t);
        }
        else
        {
          while (1)
          {
            HAL_GPIO_TogglePin(LED_ON_BOARD_GPIO_Port, LED_ON_BOARD_Pin);
            HAL_Delay(500);
          }
        }
      }
      WriteFlag = 1;
      HAL_FLASH_Lock();

      /*Сравнение исходной и записанной*/
      Address = FLASH_USER_START_ADDR;
      uint8_t MemoryError = 0x0;

      for(uint32_t i = 0; Address < FLASH_USER_END_ADDR; i++)
      {
        if (*(data_NVRAM_t *)Address != DevNVRAM_Page.data[i])
        {
          MemoryError++;
        }
        Address += sizeof(data_NVRAM_t);
      }

      if (!MemoryError)
      {
        HAL_GPIO_WritePin(LED_ON_BOARD_GPIO_Port, LED_ON_BOARD_Pin, GPIO_PIN_SET);
      }
      else
      {
        while (1)
        {
          HAL_GPIO_WritePin(LED_ON_BOARD_GPIO_Port, LED_ON_BOARD_Pin, GPIO_PIN_SET);
          HAL_Delay(200);
          HAL_GPIO_WritePin(LED_ON_BOARD_GPIO_Port, LED_ON_BOARD_Pin, GPIO_PIN_RESET);
          HAL_Delay(800);
        }
      }



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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_ON_BOARD_GPIO_Port, LED_ON_BOARD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_ON_BOARD_Pin */
  GPIO_InitStruct.Pin = BTN_ON_BOARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_ON_BOARD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_ON_BOARD_Pin */
  GPIO_InitStruct.Pin = LED_ON_BOARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ON_BOARD_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
