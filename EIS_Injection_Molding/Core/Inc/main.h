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
#include "stm32c0xx_hal.h"

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

//menu states
#define ST_MENU 0
#define ST_ITEM 1
#define ST_DATA 2
#define ST_INJECT 3
#define ST_CANCEL 4

//injection states
#define ST_STANDBY 0
#define ST_INSERT 1
#define ST_HEAT 2
#define ST_MELTING 3
#define ST_INJECT_PLASTIC 4

#define TRUE 1
#define FALSE 0
#define HIGH 1
#define LOW 0

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define User_Button_Pin GPIO_PIN_13
#define User_Button_GPIO_Port GPIOC
#define User_Button_EXTI_IRQn EXTI4_15_IRQn
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define SOLENOID_1_Pin GPIO_PIN_6
#define SOLENOID_1_GPIO_Port GPIOA
#define SOLENOID_2_Pin GPIO_PIN_7
#define SOLENOID_2_GPIO_Port GPIOA
#define ENCODER_SW_Pin GPIO_PIN_9
#define ENCODER_SW_GPIO_Port GPIOA
#define SPI_CS_2_Pin GPIO_PIN_15
#define SPI_CS_2_GPIO_Port GPIOA
#define SPI_CS_1_Pin GPIO_PIN_5
#define SPI_CS_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
