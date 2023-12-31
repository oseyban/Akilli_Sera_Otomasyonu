/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define LCD_EN_Pin GPIO_PIN_1
#define LCD_EN_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_2
#define LCD_RS_GPIO_Port GPIOA
#define LCD_POT_Pin GPIO_PIN_3
#define LCD_POT_GPIO_Port GPIOA
#define LCD_D4_Pin GPIO_PIN_4
#define LCD_D4_GPIO_Port GPIOA
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_GPIO_Port GPIOA
#define LCD_D6_Pin GPIO_PIN_6
#define LCD_D6_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOA
#define DHT22_Pin GPIO_PIN_1
#define DHT22_GPIO_Port GPIOB
#define motor1_Pin GPIO_PIN_2
#define motor1_GPIO_Port GPIOB
#define kapak_motor_Pin GPIO_PIN_13
#define kapak_motor_GPIO_Port GPIOA
#define motor2_Pin GPIO_PIN_3
#define motor2_GPIO_Port GPIOB
#define motor3_Pin GPIO_PIN_4
#define motor3_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOB
#define Panel_Motor_Pin GPIO_PIN_6
#define Panel_Motor_GPIO_Port GPIOB
#define motor4_Pin GPIO_PIN_7
#define motor4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
