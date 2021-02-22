/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g0xx_hal.h"

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
uint32_t rand_number(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENABLE_6_Pin GPIO_PIN_2
#define ENABLE_6_GPIO_Port GPIOA
#define ENABLE_5_Pin GPIO_PIN_3
#define ENABLE_5_GPIO_Port GPIOA
#define ENABLE_4_Pin GPIO_PIN_4
#define ENABLE_4_GPIO_Port GPIOA
#define ENABLE_3_Pin GPIO_PIN_5
#define ENABLE_3_GPIO_Port GPIOA
#define ENABLE_2_Pin GPIO_PIN_6
#define ENABLE_2_GPIO_Port GPIOA
#define ENABLE_1_Pin GPIO_PIN_7
#define ENABLE_1_GPIO_Port GPIOA
#define ENABLE_CO_4_Pin GPIO_PIN_0
#define ENABLE_CO_4_GPIO_Port GPIOB
#define ENABLE_CO_3_Pin GPIO_PIN_1
#define ENABLE_CO_3_GPIO_Port GPIOB
#define ENABLE_CO_2_Pin GPIO_PIN_2
#define ENABLE_CO_2_GPIO_Port GPIOB
#define ENABLE_CO_1_Pin GPIO_PIN_10
#define ENABLE_CO_1_GPIO_Port GPIOB
#define RED_LED_Pin GPIO_PIN_10
#define RED_LED_GPIO_Port GPIOA
#define GREEN_LED_Pin GPIO_PIN_11
#define GREEN_LED_GPIO_Port GPIOA
#define YELLOW_LED_Pin GPIO_PIN_12
#define YELLOW_LED_GPIO_Port GPIOA
#define UART_EN_Pin GPIO_PIN_5
#define UART_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
