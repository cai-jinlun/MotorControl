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
#include "stm32f4xx_hal.h"

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
#define KEY1_Pin GPIO_PIN_1
#define KEY1_GPIO_Port GPIOF
#define KEY2_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOF
#define KEY3_Pin GPIO_PIN_3
#define KEY3_GPIO_Port GPIOF
#define H1_Hall_A_Pin GPIO_PIN_9
#define H1_Hall_A_GPIO_Port GPIOE
#define H1_Hall_B_Pin GPIO_PIN_11
#define H1_Hall_B_GPIO_Port GPIOE
#define CAN_Silent_Pin GPIO_PIN_12
#define CAN_Silent_GPIO_Port GPIOE
#define H1_PWM2_Pin GPIO_PIN_10
#define H1_PWM2_GPIO_Port GPIOB
#define H3_PWM_Pin GPIO_PIN_11
#define H3_PWM_GPIO_Port GPIOB
#define H3_INA_Pin GPIO_PIN_12
#define H3_INA_GPIO_Port GPIOB
#define H3_INB_Pin GPIO_PIN_13
#define H3_INB_GPIO_Port GPIOB
#define H_SCK_Pin GPIO_PIN_10
#define H_SCK_GPIO_Port GPIOC
#define H_MISO_Pin GPIO_PIN_11
#define H_MISO_GPIO_Port GPIOC
#define H_MOSI_Pin GPIO_PIN_12
#define H_MOSI_GPIO_Port GPIOC
#define H_CS_Pin GPIO_PIN_0
#define H_CS_GPIO_Port GPIOD
#define H1_PWM1_Pin GPIO_PIN_3
#define H1_PWM1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
