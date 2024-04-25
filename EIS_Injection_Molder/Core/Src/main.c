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

SPI_HandleTypeDef hspi2;

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
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
uint16_t getTemperature (MAX31855_HandleTypeDef* thermocouple, uint16_t tempBuffer[], uint8_t * count, uint8_t size);
void printHomeDisplay (uint8_t injectState, Plastic_Type* plastic);

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
Plastic_Type* plastic_PP = &plastics[1];


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

	char * hometxt[4] =
	{
			"Set temp            ",
			"Nozzle temp         ",
			"Barrel temp         ",
			"Heating             "
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
			"HDPE",
			"PP",
			""
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
  MX_SPI2_Init();
  MX_TIM1_Init();
  //MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //Define Plastics
  plastic_HDPE->nozzleTemp = 220;
  plastic_HDPE->barrelTemp = 210;
  plastic_HDPE->heatingTime = 1200000; // heatTime in ms
  strcpy(plastic_HDPE->name, "HDPE");

  plastic_PP->nozzleTemp = 245;
  plastic_PP->barrelTemp = 238;
  plastic_PP->heatingTime = 1200000; // heatTime in ms
  strcpy(plastic_HDPE->name, "PP");

  //Initalize peripherials
  ENC_Init(&encoder, &htim3, GPIOA, GPIO_PIN_9);
  LCD_Init(&hlcd, &hi2c1, LCD_ADDRESS);
  MAX_Init(&hmax1, &hspi2, GPIO_PIN_15, GPIOA);
  MAX_Init(&hmax2, &hspi2, GPIO_PIN_5, GPIOB);
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
  LCD_MENU_ItemSetAction(&injectControlMenu, 2, ITEM_ACTION_SELECT);

  //Add plastics to injection memu
  LCD_MENU_ItemSetString(&injectControlMenu, 1, plastic_HDPE->name);
  LCD_MENU_ItemSetString(&injectControlMenu, 2, plastic_PP->name);

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
		  if (counter - prevCountInjectTime > 60000) {
			  injectState = ST_STANDBY;
			  menuState = ST_ITEM;
			  prevCountInjectTime = counter;
			  pistonEnable.value = LOW;
			  heatEnable.value = LOW;
		  }
		  break;

	  }

	 // nozzleTemp.value = getTemperature(&hmax1, thermo1Buffer, &thermo1BufferCount, thermoBufferSize);
	 // barrelTemp.value = getTemperature(&hmax2, thermo2Buffer, &thermo2BufferCount, thermoBufferSize);

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
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
