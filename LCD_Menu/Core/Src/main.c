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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "LCD_I2C.h"
#include "LCD_Menu.h"
#include "LCD_Menu_Data.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_ADDRESS 0x20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

LCD_HandleTypeDef hlcd;

volatile uint32_t counter = 0;
uint32_t prev_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void initializeMenu(void);
uint8_t checkEncoderButtonChange (uint32_t interval, uint8_t * prevState);
GPIO_PinState readEncoderButton ();
uint32_t millis();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LCD_MENU_List menuLists[3];
LCD_MENU_Item menuItems[10];
LCD_MENU_Data dataItems[10];

char * hometxt[4] =
{
		"Set temp",
		"Current temp 1",
		"Current temp 2",
		"Process Name"
};

char * profiles[4] =
{
		"^..",
		"PEEK",
		"PPE",
		"milk jug",
};

char * temperatures[4] =
{
		"^..",
		"group 1",
		"group 2",
		""
};

char * speed[4] =
{
		"^..",
		"Kp: 1",
		"Ki: 0.5",
		"",
};

char * testStringArray1[4] =
{
		"^..",
		"gfjh",
		"kfghjk",
		"hjh"
};

char * testStringArray2[4] =
{
		"^..",
		"Poudrier",
		"Tevin",
		""
};

char * credits[4] =
{
		"Tevin Poudrier",
		"Jonah Shadley",
		"Colson Miller",
		"Josiah Mart"
};

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
  SysTick_Config(SystemCoreClock / 1000); //set to 1ms per tick

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  initializeMenu();




  LCD_MENU_List* headNode = &menuLists[0];
  LCD_MENU_List* currentList = headNode;
  LCD_MENU_Item* currentItem = NULL;

  LCD_MENU_PrintList (headNode);
  HAL_Delay(1000);

  uint8_t dir;
  uint8_t prevEncVal = 0;

  uint8_t inItem = 0;
  uint8_t editingData = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  dir = (TIM3->CR1 & 0x0010) >> 4;

	  //Move cursor
	  if (inItem == 0) {
		  if (prevEncVal - TIM3->CNT > 2 && dir == 1) {
			  currentList = LCD_MENU_MoveListCursor(currentList, MV_CURSOR_DOWN);
		  }
		  else if (TIM3->CNT - prevEncVal > 2 && dir == 0) {
			  currentList = LCD_MENU_MoveListCursor(currentList, MV_CURSOR_UP);
		  }
	  }
	  else if(inItem == 1 && editingData == 0) {
		  if (prevEncVal - TIM3->CNT > 2 && dir == 1) {
			  currentItem = LCD_MENU_MoveItemCursor(currentItem, MV_CURSOR_DOWN);
		  }
		  else if (TIM3->CNT - prevEncVal > 2 && dir == 0) {
			  currentItem = LCD_MENU_MoveItemCursor(currentItem, MV_CURSOR_UP);
		  }

	  }


	  //Select item
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 1 && inItem == 0 && editingData == 0) {
		  inItem = 1;
		  currentItem = currentList->items[currentList->cursor];
		  LCD_MENU_PrintItem(currentItem);
	  }
	  //Select item action
	  else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 1 && inItem == 1 && editingData == 0) {
		  if (currentItem->type == ITEM_TYPE_DISPLAY) {
			  inItem = 0;
			  LCD_MENU_PrintList (currentList);
		  }
		  else if (currentItem->rowType[currentItem->cursor] == ITEM_ACTION_RETURN) {
			  inItem = 0;
			  LCD_MENU_PrintList (currentList);
		  }
		  else if (currentItem->rowType[currentItem->cursor] == ITEM_ACTION_DATA) {
			  editingData = 1;
			  LCD_SetCursor(&hlcd, 16, currentItem->cursor);
			  LCD_PrintChar(&hlcd, '>');
		  }


	  }

	  //Editing data
	  if (editingData == 1) {
		  if (prevEncVal != TIM3->CNT) {
			  LCD_MENU_DataIncrement(currentItem->dataElement[currentItem->cursor], dir);
		  }
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 1) {
			  editingData = 0;
			  LCD_SetCursor(&hlcd, 16, currentItem->cursor);
			  LCD_PrintChar(&hlcd, ' ');
		  }
	  }

	  if (counter - prev_count > 10) {
		  prevEncVal = TIM3->CNT;
	  }


	  HAL_Delay(50);


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void initializeMenu(void) {
	LCD_Init(&hlcd, &hi2c1, LCD_ADDRESS);

	LCD_MENU_ListInit(&hlcd, &menuLists[0]);
	LCD_MENU_ListInit(&hlcd, &menuLists[1]);
	LCD_MENU_ListInit(&hlcd, &menuLists[2]);

	LCD_MENU_ExtendList(&menuLists[0], &menuLists[1]);
	//LCD_MENU_ExtendList(&menuLists[1], &menuLists[2]);

	LCD_MENU_ItemInit(&hlcd, &menuItems[0], "Home", hometxt, ITEM_TYPE_DISPLAY);

	LCD_MENU_ItemInit(&hlcd, &menuItems[1], "Profiles", profiles, ITEM_TYPE_CONFIG);
	LCD_MENU_ItemSetAction(&menuItems[1], 0, ITEM_ACTION_RETURN);

	//Temperatures
	LCD_MENU_ItemInit(&hlcd, &menuItems[2], "Temperatures", temperatures, ITEM_TYPE_CONFIG);
	LCD_MENU_ItemSetAction(&menuItems[2], 0, ITEM_ACTION_RETURN);
	LCD_MENU_DataInit(&dataItems[0], &hlcd, 1, 17);
	LCD_MENU_DataInit(&dataItems[1], &hlcd, 2, 17);
	dataItems[0].value = 200;
	dataItems[1].value = 220;
	LCD_MENU_ItemAddData(&menuItems[2], &dataItems[0]);
	LCD_MENU_ItemAddData(&menuItems[2], &dataItems[1]);

	LCD_MENU_ItemInit(&hlcd, &menuItems[3], "Speed", speed, ITEM_TYPE_CONFIG);
	LCD_MENU_ItemSetAction(&menuItems[3], 0, ITEM_ACTION_RETURN);

	LCD_MENU_ItemInit(&hlcd, &menuItems[4], "test1234", testStringArray1, ITEM_TYPE_CONFIG);
	LCD_MENU_ItemSetAction(&menuItems[4], 0, ITEM_ACTION_RETURN);

	LCD_MENU_ItemInit(&hlcd, &menuItems[5], "test0312", testStringArray2, ITEM_TYPE_CONFIG);
	LCD_MENU_ItemSetAction(&menuItems[5], 0, ITEM_ACTION_RETURN);

	LCD_MENU_ItemInit(&hlcd, &menuItems[6], "Credits", credits, ITEM_TYPE_DISPLAY);

	for (uint8_t i = 0, j = -1; i < 7; i++) {
	  if (i % 4 == 0) {
		  j++;
	  }
	  LCD_MENU_AddItemToList(&menuLists[j], &menuItems[i]);
	}
}

uint8_t checkEncoderButtonChange (uint32_t interval, uint8_t * prevState) {
	if (millis() % interval == 0) {
		uint8_t newState = readEncoderButton();
		if (newState != *prevState) {
			return 1;
			*prevState = newState;
		}
	}
	return 0;
}

GPIO_PinState readEncoderButton () {
	return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
}

uint32_t millis() {
	return counter;
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