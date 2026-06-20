/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define E2A_Pin GPIO_PIN_0
#define E2A_GPIO_Port GPIOA
#define E2B_Pin GPIO_PIN_1
#define E2B_GPIO_Port GPIOA
#define STBY_Pin GPIO_PIN_3
#define STBY_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_4
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_5
#define AIN2_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOA
#define E1A_Pin GPIO_PIN_8
#define E1A_GPIO_Port GPIOA
#define E1B_Pin GPIO_PIN_9
#define E1B_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_0
#define BIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_1
#define BIN1_GPIO_Port GPIOB
#define TRACK1_Pin GPIO_PIN_9
#define TRACK1_GPIO_Port GPIOB
#define TRACK2_Pin GPIO_PIN_8
#define TRACK2_GPIO_Port GPIOB
#define TRACK3_Pin GPIO_PIN_7
#define TRACK3_GPIO_Port GPIOB
#define TRACK4_Pin GPIO_PIN_6
#define TRACK4_GPIO_Port GPIOB
#define TRACK5_Pin GPIO_PIN_5
#define TRACK5_GPIO_Port GPIOB
#define TRACK6_Pin GPIO_PIN_4
#define TRACK6_GPIO_Port GPIOB
#define TRACK7_Pin GPIO_PIN_3
#define TRACK7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
