/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define ACC_CS_Pin GPIO_PIN_4
#define ACC_CS_GPIO_Port GPIOE
#define GYRO_CS_Pin GPIO_PIN_14
#define GYRO_CS_GPIO_Port GPIOC
#define PWM_M6_IN2_D_Pin GPIO_PIN_0
#define PWM_M6_IN2_D_GPIO_Port GPIOA
#define PWM_M6_IN1_D_Pin GPIO_PIN_3
#define PWM_M6_IN1_D_GPIO_Port GPIOA
#define PWM_M5_IN2_D_Pin GPIO_PIN_6
#define PWM_M5_IN2_D_GPIO_Port GPIOA
#define PWM_M5_IN1_D_Pin GPIO_PIN_9
#define PWM_M5_IN1_D_GPIO_Port GPIOE
#define PWM_M8_IN2_D_Pin GPIO_PIN_11
#define PWM_M8_IN2_D_GPIO_Port GPIOE
#define PWM_M8_IN1_D_Pin GPIO_PIN_13
#define PWM_M8_IN1_D_GPIO_Port GPIOE
#define PWM_M7_IN2_D_Pin GPIO_PIN_14
#define PWM_M7_IN2_D_GPIO_Port GPIOE
#define PWM_M7_IN1_D_Pin GPIO_PIN_10
#define PWM_M7_IN1_D_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_9
#define LED_B_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_10
#define LED_G_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOD
#define PWM_M4_IN2_D_Pin GPIO_PIN_12
#define PWM_M4_IN2_D_GPIO_Port GPIOD
#define PWM_M4_IN1_D_Pin GPIO_PIN_13
#define PWM_M4_IN1_D_GPIO_Port GPIOD
#define PWM_M3_IN2_D_Pin GPIO_PIN_14
#define PWM_M3_IN2_D_GPIO_Port GPIOD
#define PWM_M3_IN1_D_Pin GPIO_PIN_15
#define PWM_M3_IN1_D_GPIO_Port GPIOD
#define PWM_M2_IN2_D_Pin GPIO_PIN_7
#define PWM_M2_IN2_D_GPIO_Port GPIOC
#define PWM_M2_IN1_D_Pin GPIO_PIN_8
#define PWM_M2_IN1_D_GPIO_Port GPIOC
#define PWM_M1_IN2_D_Pin GPIO_PIN_9
#define PWM_M1_IN2_D_GPIO_Port GPIOC
#define PWM_M1_IN1_D_Pin GPIO_PIN_3
#define PWM_M1_IN1_D_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
