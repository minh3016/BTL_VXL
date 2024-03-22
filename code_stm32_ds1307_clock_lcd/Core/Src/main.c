/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "ds1307.h"
#include "CLCD.h"
#include "flash.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
I2C_HandleTypeDef hi2c1;
DS1307_Handle ds1307;
DS1307_TIME time;
I2C_HandleTypeDef i2c_handle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define addr 		 	((uint32_t)0x0800FC00)	//Page 63
#define addr 		((uint32_t)0x0800F800)	//Page 62

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
CLCD_Name LCD1;
char LCD_send[20];
int cmode=0;
int AlarmHour=7, AlarmMin=0;
int AlarmCheck=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const char *DAYS_OF_WEEK[7] = { "CN", "T2", "T3", "T4", "T5", "T6", "T7" };
DS1307_RESULT confDS1307(void) {

	ds1307.i2c = &i2c_handle;
	ds1307.DS1307_ADDRESS = 0xD0;
	ds1307.DS1307_CLOCK = 100000;
	ds1307.DS1307_I2Cx = I2C1;
	ds1307.TIMEOUT = 1000;
	DS1307_RESULT res;
	res = DS1307_Init(&ds1307);
	if (res != DS1307_RES_OK) {
		return DS1307_RES_ERROR;
	}
	res = DS1307_ClockResume(&ds1307);
	if (res != DS1307_RES_OK) {
		return DS1307_RES_ERROR;
	}
	res = DS1307_EnableSquareWave(&ds1307);
	if (res != DS1307_RES_OK) {
		return DS1307_RES_ERROR;
	}
	res = DS1307_SelectRate(&ds1307, DS1307_RATE_1HZ);
	if (res != DS1307_RES_OK) {
		return DS1307_RES_ERROR;
	}
	return DS1307_RES_OK;
}

void tick(int _n, int _dl)
{
	while(_n--)
	{
		HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, 1);
		HAL_Delay(_dl);
		HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, 0);
		HAL_Delay(_dl);
	}
}
int readButton()
{
	if (HAL_GPIO_ReadPin(SW1) == 0)
	{
		HAL_Delay(3);
		if (HAL_GPIO_ReadPin(SW1) == 0)
		{
			tick(1,30);
			return 1;
		}
	}
	if (HAL_GPIO_ReadPin(SW2) == 0)
	{
		HAL_Delay(3);
		if (HAL_GPIO_ReadPin(SW2) == 0)
		{
			tick(1,30);
			return 2;
		}
	}
	if (HAL_GPIO_ReadPin(SW3) == 0)
	{
		HAL_Delay(3);
		if (HAL_GPIO_ReadPin(SW3) == 0)
		{
			tick(1,30);
			return 3;
		}
	}
	return 0;
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  tick(1,80);
  CLCD_4BIT_Init(&LCD1, 16, 2, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin,
  									D4_GPIO_Port, D4_Pin, D5_GPIO_Port, D5_Pin,
  									D6_GPIO_Port, D6_Pin, D7_GPIO_Port, D7_Pin);
  HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);

  CLCD_SetCursor(&LCD1, 0, 0);
  CLCD_WriteString(&LCD1, "Start ...");

  AlarmHour = Flash_Read_Int(addr);
  AlarmMin  = Flash_Read_Int(addr+8);

  if (AlarmHour>23) AlarmHour=0;
  if (AlarmMin>59) AlarmMin=0;

  if (confDS1307() != DS1307_RES_OK) tick(1,1000);
  else tick(3,100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (cmode == 0)
	{
	  CLCD_Clear(&LCD1);
	  while (cmode==0)
	  {
		  time = DS1307_GetTime(&ds1307);
		  sprintf(LCD_send,"    %02d:%02d:%02d    ",time.hour,time.minute,time.second);
		  CLCD_SetCursor(&LCD1, 0, 0); CLCD_WriteString(&LCD1, LCD_send);
		  sprintf(LCD_send," %s, %02d/%02d/%04d ",DAYS_OF_WEEK[time.day],time.date,time.month,time.year);
		  CLCD_SetCursor(&LCD1, 0, 1); CLCD_WriteString(&LCD1, LCD_send);

		  if (time.hour==AlarmHour && time.minute==AlarmMin)
		  {
			  if (AlarmCheck==0) tick(3, 50);
		  }
		  else
		  {
			  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, 0);
			  AlarmCheck=0;
		  }

		  int x = readButton();
		  if (x==2) cmode=1; // cài th�?i gian
		  else if (x==1) cmode=2; // cài hẹn gi�?
		  else if (x==3 && AlarmCheck==0)
		  {
			  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, 0);
			  AlarmCheck=1;
		  }


	  }
	}
	if (cmode==1)
	{
	  CLCD_Clear(&LCD1);
	  CLCD_SetCursor(&LCD1, 0, 0); CLCD_WriteString(&LCD1, "CaiDat Thoi Gian");
	  HAL_Delay(1500);
	  CLCD_Clear(&LCD1);
	  int val[7];
	  int max[7]={23,59,59,6,31,12,2050};
	  int min[7]={0, 0, 0, 0,1, 1, 2010};
	  int i=0;
	  val[0]=time.hour;
	  val[1]=time.minute;
	  val[2]=time.second;
	  val[3]=time.day;
	  val[4]=time.date;
	  val[5]=time.month;
	  val[6]=time.year;
	  while (cmode==1)
	  {
		  sprintf(LCD_send,"    %02d:%02d:%02d    ",val[0],val[1],val[2]);
		  CLCD_SetCursor(&LCD1, 0, 0); CLCD_WriteString(&LCD1, LCD_send);
		  sprintf(LCD_send," %s, %02d/%02d/%04d ",DAYS_OF_WEEK[val[3]],val[4],val[5],val[6]);
		  CLCD_SetCursor(&LCD1, 0, 1); CLCD_WriteString(&LCD1, LCD_send);
		  HAL_Delay(10);
		  if (i==0) CLCD_SetCursor(&LCD1, 4, 0);
		  else if (i==1) CLCD_SetCursor(&LCD1, 7, 0);
		  else if (i==2) CLCD_SetCursor(&LCD1, 10, 0);
		  else if (i==3) CLCD_SetCursor(&LCD1, 1, 1);
		  else if (i==4) CLCD_SetCursor(&LCD1, 5, 1);
		  else if (i==5) CLCD_SetCursor(&LCD1, 8, 1);
		  else if (i==6) CLCD_SetCursor(&LCD1, 13, 1);
		  CLCD_WriteString(&LCD1, "__");

		  int x = readButton();
		  if (x==3) // MODE
		  {
			  i++;
			  if (i>6) // lưu
			  {
				  time.hour = val[0];
				  time.minute = val[1];
				  time.second = val[2];
				  time.day = val[3];
				  time.date = val[4];
				  time.month = val[5];
				  time.year = val[6];
				  DS1307_SetTime(&ds1307, time);
				  CLCD_Clear(&LCD1);
				  CLCD_SetCursor(&LCD1, 0, 0); CLCD_WriteString(&LCD1, "   Da Luu lai   ");
				  HAL_Delay(1000);
				  cmode=0;
			  }
		  }
		  else if (x==1) // DOWN
		  {
			  if (val[i]>min[i]) val[i]--;
			  else val[i]=max[i];
		  }
		  else if (x==2) // UP
		  {
			  if (val[i]<max[i]) val[i]++;
			  else val[i]=min[i];
		  }
	  }
	}
	if (cmode==2)
	{
	  CLCD_Clear(&LCD1);
	  CLCD_SetCursor(&LCD1, 0, 0); CLCD_WriteString(&LCD1, "Cai dat Bao Thuc");
	  HAL_Delay(1500);
	  CLCD_Clear(&LCD1);
	  int i=0;
	  while(cmode==2)
	  {
		  CLCD_SetCursor(&LCD1, 0, 0); CLCD_WriteString(&LCD1, "Bao Thuc:       ");
		  sprintf(LCD_send,"       %02d:%02d    ",AlarmHour,AlarmMin);
		  CLCD_SetCursor(&LCD1, 0, 1); CLCD_WriteString(&LCD1, LCD_send);

		  if (i==0) CLCD_SetCursor(&LCD1, 7, 1);
		  else if (i==1) CLCD_SetCursor(&LCD1, 10, 1);
		  CLCD_WriteString(&LCD1, "__");

		  int x = readButton();
		  if (x==3)
		  {
			  i++;
			  if (i>=2) // Lưu
			  {
				  Flash_Unlock();
				  Flash_Erase(addr);
				  Flash_Write_Int(addr, AlarmHour);
				  HAL_Delay(50);
				  Flash_Unlock();
				  Flash_Write_Int(addr+8, AlarmMin);
				  CLCD_Clear(&LCD1);
				  CLCD_SetCursor(&LCD1, 0, 0); CLCD_WriteString(&LCD1, "   Da Luu lai   ");
				  HAL_Delay(1000);
				  cmode=0;
			  }
		  }
		  else if (x==1) // Giảm
		  {
			  if (i==0)
			  {
				  if (AlarmHour>0) AlarmHour--; else AlarmHour=23;
			  }
			  else
			  {
				  if (AlarmMin>0) AlarmMin--; else AlarmMin=59;
			  }
		  }
		  else if (x==2) // Tang
		  {
			  if (i==0)
			  {
				  if (AlarmHour<23) AlarmHour++; else AlarmHour=0;
			  }
			  else
			  {
				  if (AlarmMin<59) AlarmMin++; else AlarmMin=0;
			  }
		  }
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUZZ_Pin|D5_Pin|D4_Pin|EN_Pin
                          |RW_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D7_Pin|D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZ_Pin LED_Pin D5_Pin D4_Pin
                           EN_Pin RW_Pin RS_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin|LED_Pin|D5_Pin|D4_Pin
                          |EN_Pin|RW_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D7_Pin D6_Pin */
  GPIO_InitStruct.Pin = D7_Pin|D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
