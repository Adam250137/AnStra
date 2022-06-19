/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Test_LED_Pin GPIO_PIN_13
#define Test_LED_GPIO_Port GPIOC
#define RF_STATUS_Pin GPIO_PIN_14
#define RF_STATUS_GPIO_Port GPIOC
#define RF_RESET_Pin GPIO_PIN_15
#define RF_RESET_GPIO_Port GPIOC
#define RF_CONFIG_Pin GPIO_PIN_0
#define RF_CONFIG_GPIO_Port GPIOA
#define RF_SLEEP_Pin GPIO_PIN_1
#define RF_SLEEP_GPIO_Port GPIOA
#define K1_Pin GPIO_PIN_4
#define K1_GPIO_Port GPIOA
#define K2_Pin GPIO_PIN_5
#define K2_GPIO_Port GPIOA
#define K3_Pin GPIO_PIN_6
#define K3_GPIO_Port GPIOA
#define K4_Pin GPIO_PIN_7
#define K4_GPIO_Port GPIOA
#define S1_Pin GPIO_PIN_0
#define S1_GPIO_Port GPIOB
#define S2_Pin GPIO_PIN_1
#define S2_GPIO_Port GPIOB
#define K8_Pin GPIO_PIN_4
#define K8_GPIO_Port GPIOB
#define K7_Pin GPIO_PIN_5
#define K7_GPIO_Port GPIOB
#define K6_Pin GPIO_PIN_6
#define K6_GPIO_Port GPIOB
#define K5_Pin GPIO_PIN_7
#define K5_GPIO_Port GPIOB
#define LCD_SCL_Pin GPIO_PIN_8
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_9
#define LCD_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/