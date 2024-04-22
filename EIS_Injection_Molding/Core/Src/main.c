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
#include "encoder.h"
#include <stdio.h>
#include "LCD_I2C.h"
#include "LCD_Menu.h"
#include "LCD_Menu_Data.h"
#include "MAX31855.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct __plastic {
	char name[21]; //name can only be 20 characters long, extra byte for null character
	uint16_t nozzleTemp;
	uint16_t barrelTemp;
	uint32_t heatingTime; //heating time in ms
} Plastic_Type;

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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ENC_Handle encoder;

LCD_HandleTypeDef hlcd;

MAX31855_HandleTypeDef hmax1;
MAX31855_HandleTypeDef hmax2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
uint16_t getTemperature (MAX31855_HandleTypeDef* thermocouple, uint16_t tempBuffer[], uint8_t * count, uint8_t size);
void printHomeDisplay ();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t counter = 0;
uint32_t prevCountPrintDebug;
uint32_t prevCountInjectTime;
uint32_t prevCountLED;

//Create main menu
LCD_MENU_List mainMenu;
LCD_MENU_List mainMenu2;

//Create menu items
LCD_MENU_Item homeDisplay;
LCD_MENU_Item debugMenu;
LCD_MENU_Item temperatureMenu;
LCD_MENU_Item injectControlMenu;
LCD_MENU_Item cancelInjectionMenu;
LCD_MENU_Item creditsMenu;

//Create data elements for menu
LCD_MENU_Data setNozzleTemp;
LCD_MENU_Data setBarrelTemp;
LCD_MENU_Data barrelTemp;
LCD_MENU_Data nozzleTemp;
LCD_MENU_Data heatEnable;
LCD_MENU_Data pistonEnable;
LCD_MENU_Data doorEnable;
LCD_MENU_Data ledEnable;
LCD_MENU_Data heatingSpeed;
LCD_MENU_Data injectionEnable;

//Create plastic types
Plastic_Type plastics [5];
Plastic_Type * plastic_HDPE = &plastics[0];

char * hometxt[4] =
{
		"Set temp",
		"Nozzle temp",
		"Barrel temp",
		"Heating"
};

char * debugTxt[4] =
{
		"^..",
		"Piston",
		"H Door",
		"LED"
};

char * temperatureTxt[4] =
{
		"^..",
		"Set Temp 1",
		"Heating",
		"Speed"
};

char * injectSelectionTxt[4] =
{
		"^..",
};

char * cancelInjectionTxt[4] =
{
		"Cancel Injection?",
		"Yes",
		"No",
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
	uint8_t MSG[100] = {'\0'};

	uint8_t thermoBufferSize = 10;
	uint16_t thermo1Buffer[10] = {0};
	uint8_t thermo1BufferCount = 0;

	uint16_t thermo2Buffer[10] = {0};
	uint8_t thermo2BufferCount = 0;


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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //Define Plastics
  plastic_HDPE->nozzleTemp = 220;
  plastic_HDPE->barrelTemp = 210;
  plastic_HDPE->heatingTime = 1200000; // heatTime in ms
  strcpy(plastic_HDPE->name, "HDPE");

  //Initalize peripherials
  ENC_Init(&encoder, &htim3, GPIOA, GPIO_PIN_9);
  LCD_Init(&hlcd, &hi2c1, LCD_ADDRESS);
  MAX_Init(&hmax1, &hspi1, GPIO_PIN_15, GPIOA);
  MAX_Init(&hmax2, &hspi1, GPIO_PIN_5, GPIOB);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  //Initalize Menu Lists
  LCD_MENU_ListInit(&hlcd, &mainMenu);
  LCD_MENU_ListInit(&hlcd, &mainMenu2);
  LCD_MENU_ExtendList(&mainMenu, &mainMenu2);

  //Initialize Data Elements
  LCD_MENU_DataInit(&setNozzleTemp, &hlcd); // set temp
  LCD_MENU_DataInit(&setBarrelTemp, &hlcd);
  LCD_MENU_DataInit(&nozzleTemp, &hlcd); // current temp 1
  LCD_MENU_DataInit(&barrelTemp, &hlcd); // current temp 2
  LCD_MENU_DataInit(&heatEnable, &hlcd); // heating on/off
  LCD_MENU_DataInit(&pistonEnable, &hlcd); // piston
  LCD_MENU_DataInit(&doorEnable, &hlcd); // h door
  LCD_MENU_DataInit(&ledEnable, &hlcd); // led
  LCD_MENU_DataInit(&injectionEnable, &hlcd);


  //Initialize Menu Items
  LCD_MENU_ItemInit(&hlcd, &homeDisplay, "Home", hometxt, ITEM_TYPE_DISPLAY);
  LCD_MENU_ItemInit(&hlcd, &debugMenu, "Debug", debugTxt, ITEM_TYPE_CONFIG);
  LCD_MENU_ItemInit(&hlcd, &temperatureMenu, "Temperature", temperatureTxt, ITEM_TYPE_CONFIG);
  LCD_MENU_ItemInit(&hlcd, &injectControlMenu, "Start Injection", injectSelectionTxt, ITEM_TYPE_CONFIG);
  LCD_MENU_ItemInit(&hlcd, &cancelInjectionMenu, "CancelInjection", cancelInjectionTxt, ITEM_TYPE_CONFIG);
  LCD_MENU_ItemInit(&hlcd, &creditsMenu, "Credits", credits, ITEM_TYPE_DISPLAY);


  //Add data to Home Display
  LCD_MENU_ItemAddData(&homeDisplay, &setNozzleTemp, 0,  ITEM_ACTION_DATA); // set temp
  LCD_MENU_ItemAddData(&homeDisplay, &nozzleTemp, 1, ITEM_ACTION_DATA); // current temp 1
  LCD_MENU_ItemAddData(&homeDisplay, &barrelTemp, 2, ITEM_ACTION_DATA); // current temp 2
  LCD_MENU_ItemAddData(&homeDisplay, &heatEnable, 3, ITEM_ACTION_TOGGLE); // heating on/off


  //Add data items to Debug menu
  LCD_MENU_ItemAddData(&debugMenu, &pistonEnable, 1, ITEM_ACTION_TOGGLE);
  LCD_MENU_ItemAddData(&debugMenu, &doorEnable, 2, ITEM_ACTION_TOGGLE);
  LCD_MENU_ItemAddData(&debugMenu, &ledEnable, 3, ITEM_ACTION_TOGGLE);
  LCD_MENU_ItemSetAction(&debugMenu, 0, ITEM_ACTION_RETURN); //return to main menu

  //Add data items to temperature menu
  LCD_MENU_ItemAddData(&temperatureMenu, &setNozzleTemp, 1, ITEM_ACTION_DATA); //set temp
  LCD_MENU_ItemAddData(&temperatureMenu, &heatEnable, 2, ITEM_ACTION_TOGGLE); // heating on/off
  LCD_MENU_ItemAddData(&temperatureMenu, &heatingSpeed, 3, ITEM_ACTION_DATA); // heating speed
  LCD_MENU_ItemSetAction(&temperatureMenu, 0, ITEM_ACTION_RETURN);

  //Add data items to injection menu
  LCD_MENU_ItemSetAction(&injectControlMenu, 0, ITEM_ACTION_RETURN);
  LCD_MENU_ItemSetAction(&injectControlMenu, 1, ITEM_ACTION_SELECT);

  //Add plastics to injection memu
  LCD_MENU_ItemSetString(&injectControlMenu, 1, plastic_HDPE->name);

  //Add actions to cancel menu
  LCD_MENU_ItemSetAction(&cancelInjectionMenu, 1, ITEM_ACTION_SELECT);
  LCD_MENU_ItemSetAction(&cancelInjectionMenu, 2, ITEM_ACTION_SELECT);

  //Add menu items to main menu
  LCD_MENU_AddItemToList(&mainMenu, &homeDisplay);
  LCD_MENU_AddItemToList(&mainMenu, &injectControlMenu);
  LCD_MENU_AddItemToList(&mainMenu, &debugMenu);
  LCD_MENU_AddItemToList(&mainMenu, &temperatureMenu);
  LCD_MENU_AddItemToList(&mainMenu2, &creditsMenu);

  sprintf( (char*) MSG, "EIS Injection Molding %d\n\r", MAX_GetCelcius(&hmax1));
  HAL_UART_Transmit(&huart2, MSG, strlen( (char*) MSG), 100);

  uint8_t menuState = ST_MENU;
  uint8_t injectState = ST_STANDBY;
  uint8_t cancelRequest = LOW;

  LCD_MENU_List* headList = &mainMenu;
  LCD_MENU_List* currentList = headList;
  LCD_MENU_Item* currentItem = &homeDisplay;
  Plastic_Type* currentPlastic = plastic_HDPE;

  LCD_MENU_PrintList (headList);

  //set default temperatures
  setNozzleTemp.value = 220; //set temp to 5
  setBarrelTemp.value = 210;
  heatingSpeed.value = 210; // heat bands duty cycle 0-255

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Navigate main menu
	  if (menuState == ST_MENU) {
		  if (ENC_ReadSwitch(&encoder) == HIGH) {
			  menuState = ST_ITEM;
			  currentItem = currentList->items[currentList->cursor];
			  LCD_MENU_PrintItem(currentItem);
		  }
		  else if (ENC_CountChange(&encoder) == TRUE) {
			  uint8_t direction = ENC_GetDirection(&encoder);
			  currentList = LCD_MENU_MoveListCursor(currentList, direction);
		  }
	  }
	  //Navigate menu item
	  else if (menuState == ST_ITEM) {
		  if (currentItem->type == ITEM_TYPE_DISPLAY) {
			  LCD_MENU_UpdateItemData(currentItem);
		  }

		  if (ENC_ReadSwitch(&encoder) == HIGH) {
			  //Return to main menu
			  if (currentItem->type == ITEM_TYPE_DISPLAY) {

				  menuState = ST_MENU;
				  LCD_MENU_PrintList (currentList);
			  }
			  //return to main menu
			  else if (currentItem->rowType[currentItem->cursor] == ITEM_ACTION_RETURN) {
				  menuState = ST_MENU;
				  LCD_MENU_PrintList (currentList);
			  }
			  //Edit data
			  else if (currentItem->rowType[currentItem->cursor] == ITEM_ACTION_DATA) {
				  menuState = ST_DATA;
			  }
			  //Toggle data
			  else if (currentItem->rowType[currentItem->cursor] == ITEM_ACTION_TOGGLE) {
				  LCD_MENU_DataToggle(currentItem->dataElement[currentItem->cursor]);
				  LCD_MENU_UpdateItemData(currentItem);
			  }
			  //Select injection profile
			  else if (currentItem->rowType[currentItem->cursor] == ITEM_ACTION_SELECT) {
				  currentPlastic = &plastics[currentItem->cursor - 1];
				  menuState = ST_INJECT;
				  injectState = ST_HEAT;
				  currentItem = &homeDisplay;
				  LCD_MENU_PrintItem(&homeDisplay);
			  }

		  }
		  //Move cursor
		  else if (ENC_CountChange(&encoder) == TRUE) {
			  uint8_t direction = ENC_GetDirection(&encoder);
			  currentItem = LCD_MENU_MoveItemCursor(currentItem, direction);
		  }
	  }
	  //Editing Data
	  else if (menuState == ST_DATA) {
		  //Return to menu item
		  if (ENC_ReadSwitch(&encoder) == HIGH) {
			  menuState = ST_ITEM;
		  }
		  //Increment data based on encoder direction
		  else if (ENC_CountChange(&encoder) == TRUE) {
			  LCD_MENU_DataIncrement(currentItem->dataElement[currentItem->cursor], ENC_GetDirection(&encoder));
			  LCD_MENU_UpdateItemData(currentItem);
		  }
	  }

	  /**
	   * Injection Process active
	   * Lock display on home info screen
	   * User can cancel procces by pressing encoder button and confirming
	   */
	  else if (menuState == ST_INJECT) {
		  //update display
		  LCD_MENU_UpdateItemData(currentItem);
		  uint8_t switchValue = ENC_ReadSwitch(&encoder);

		  //Request to cancel injection
		  if (switchValue && cancelRequest == LOW) {
			  cancelRequest = HIGH;
			  currentItem = &cancelInjectionMenu;
			  LCD_MENU_PrintItem(currentItem);
		  }
		  //Select Yes or no to cancel injection
		  else if (switchValue && cancelRequest == HIGH) {
			  //cancel
			  if (currentItem->cursor == 1) {
				  cancelRequest = LOW;
				  menuState = ST_MENU;
				  injectState = ST_STANDBY;
				  LCD_MENU_PrintList(headList);
			  }
			  //do not cancel
			  else if (currentItem->cursor == 2) {
				  cancelRequest = LOW;
				  currentItem = &homeDisplay;
				  LCD_MENU_PrintItem(currentItem);
			  }

		  }
		  //Navigate cancelation menu
		  else if (cancelRequest == HIGH) {
			  if (ENC_CountChange(&encoder) == TRUE) {
				  uint8_t direction = ENC_GetDirection(&encoder);
				  currentItem = LCD_MENU_MoveItemCursor(currentItem, direction);
			  }
		  }
	  }

	  //Injection Process Control
	  switch (injectState) {
	  //wating to inject
	  case ST_STANDBY:
		  injectionEnable.value = LOW;
		  prevCountInjectTime = counter;
		  break;

	  //heating
	  case ST_HEAT:
		  setNozzleTemp.value = currentPlastic->nozzleTemp;
		  setBarrelTemp.value = currentPlastic->barrelTemp;
		  heatEnable.value = HIGH;

		  //heating complete
		  if (nozzleTemp.value >= currentPlastic->nozzleTemp && barrelTemp.value >= currentPlastic->barrelTemp) {
			  injectState = ST_INSERT;
			  prevCountInjectTime = counter;

			  //turn off heat bands
			  TIM1->CCR2 = 0;
			  TIM1->CCR1 = 0;
		  }

		  else {
			  //turn off heat bands
			  TIM1->CCR2 = 0;
			  TIM1->CCR1 = 0;
		  }
		  break;

	  //insert plastc, hold door open for 10 seconds
	  case ST_INSERT:
		  doorEnable.value = HIGH;
		  if (counter - prevCountInjectTime > 10000) {
			  injectState = ST_MELTING;
			  prevCountInjectTime = counter;
			  doorEnable.value = LOW;
		  }

		  break;

	  //melting plastic
	  case ST_MELTING:
		  if (counter - prevCountInjectTime > currentPlastic->heatingTime) {
			  injectState = ST_INJECT_PLASTIC;
			  prevCountInjectTime = counter;
		  }
		  break;

	  //inject plastic
	  case ST_INJECT_PLASTIC:
		  pistonEnable.value = HIGH;
		  if (counter - prevCountInjectTime > 30000) {
			  injectState = ST_STANDBY;
			  menuState = ST_ITEM;
			  prevCountInjectTime = counter;
			  pistonEnable.value = LOW;
			  heatEnable.value = LOW;
		  }
		  break;

	  }

	  nozzleTemp.value = getTemperature(&hmax1, thermo1Buffer, &thermo1BufferCount, thermoBufferSize);
	  barrelTemp.value = getTemperature(&hmax2, thermo2Buffer, &thermo2BufferCount, thermoBufferSize);

	  //Heating
	  if (heatEnable.value == HIGH) {
		  //turn on nozzle heat bands if below set temp
		  if (nozzleTemp.value < setNozzleTemp.value) {
			  TIM1->CCR1 = (uint8_t) heatingSpeed.value;

		  }
		  else {
			  //turn off heat bands
			  TIM1->CCR1 = 0;
			  if (injectState == ST_STANDBY) {
				  heatEnable.value = LOW;
			  }
		  }

		  //turn on barrel heat bands if below set temp
		  if (barrelTemp.value < setBarrelTemp.value) {
			  TIM1->CCR2 = (uint8_t) heatingSpeed.value;
		  }
		  else {
			  //turn off heat bands
			  TIM1->CCR2 = 0;

			  if (injectState == ST_STANDBY) {
				  heatEnable.value = LOW;
			  }

		  }
	  }
	  else {
		  //turn off heat bands
		  TIM1->CCR2 = 0;
		  TIM1->CCR1 = 0;
	  }

	  //Status LED
	  if (injectState != ST_STANDBY) {
		  //blink LED during injection process
		  if (counter - prevCountLED > 500) {
			  //toggle LED
			  ledEnable.value = !ledEnable.value & 0x1;
			  prevCountLED = counter;
		  }
	  }
	  else {
		  if (nozzleTemp.value > 40 || barrelTemp.value > 40) {
			  ledEnable.value = HIGH;
		  }
		  else {
			  ledEnable.value = LOW;
		  }
	  }


	  //Update GPIO values;
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, ledEnable.value);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, doorEnable.value);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, pistonEnable.value);

	  HAL_Delay(10);

	  //Print debug messages to com port
	  if (counter - prevCountPrintDebug > 1000) {
		  if (heatEnable.value == HIGH) {
			  if (currentPlastic->nozzleTemp > nozzleTemp.value) {
				  nozzleTemp.value++;
			  }
			  if (currentPlastic->barrelTemp > barrelTemp.value) {
				  barrelTemp.value++;
			  }
		  }

		  sprintf( (char*) MSG, "----Application state: %d----\r\n", menuState);
		  HAL_UART_Transmit(&huart2, MSG, strlen( (char*) MSG), 100);

		  sprintf( (char*) MSG, "Injection state: %d\r\n", injectState);
		  HAL_UART_Transmit(&huart2, MSG, strlen( (char*) MSG), 100);

		  sprintf( (char*) MSG, "LED: %d, Door: %d, Piston: %d\r\n", ledEnable.value, doorEnable.value, pistonEnable.value);
		  HAL_UART_Transmit(&huart2, MSG, strlen( (char*) MSG), 100);

		  sprintf( (char*) MSG, "HeatEN: %d, nozzle: %d, barrel: %d, heatSpeed: %d\r\n", heatEnable.value, nozzleTemp.value, barrelTemp.value, heatingSpeed.value);
		  HAL_UART_Transmit(&huart2, MSG, strlen( (char*) MSG), 100);

		  sprintf( (char*) MSG, "Set temps, nozzle: %d, barrel: %d\r\n", currentPlastic->nozzleTemp, currentPlastic->barrelTemp);
		  HAL_UART_Transmit(&huart2, MSG, strlen( (char*) MSG), 100);

		  sprintf( (char*) MSG, "Count-prevInjectCount: %lu, wait time: %lu\r\n\n", counter - prevCountInjectTime, currentPlastic->heatingTime );
		  HAL_UART_Transmit(&huart2, MSG, strlen( (char*) MSG), 100);
		  prevCountPrintDebug = counter;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  HAL_GPIO_WritePin(GPIOA, LED_Pin|SOLENOID_1_Pin|SOLENOID_2_Pin|SPI_CS_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_1_GPIO_Port, SPI_CS_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin SOLENOID_1_Pin SOLENOID_2_Pin SPI_CS_2_Pin */
  GPIO_InitStruct.Pin = LED_Pin|SOLENOID_1_Pin|SOLENOID_2_Pin|SPI_CS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_SW_Pin */
  GPIO_InitStruct.Pin = ENCODER_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_1_Pin */
  GPIO_InitStruct.Pin = SPI_CS_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//Ring buffer implementation
uint16_t getTemperature (MAX31855_HandleTypeDef* thermocouple, uint16_t tempBuffer[], uint8_t * count, uint8_t size)
{

	//Get measurement
	tempBuffer[*count] = MAX_GetCelcius(thermocouple);
	*count = (*count + 1) % size;

	//Calculate total
	uint32_t total = 0;
    for (uint8_t i = 0; i < size; i++)
    {
	    total += tempBuffer[i];
    }

    //Calculate and return average
    uint16_t average = total / size;
    return average;
}

//void printHomeDisplay () {
//	char string[21] = {0};
//
//	//print nozzle data
//	LCD_SetCursor(&hlcd, 0, 0);
//	snprintf(string, 21, "Nozzle: %d/%d", setNozzleTemp.value, nozzleTemp.value);
//	LCD_Print(&hlcd, string);
//
//	//print barrel data
//	LCD_SetCursor(&hlcd, 0, 1);
//	snprintf(string, 21, "Barrel: %d/%d", setBarrelTemp.value, barrelTemp.value);
//	LCD_Print(&hlcd, string);
//}

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
