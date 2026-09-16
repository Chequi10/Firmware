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
#define LIMIT_INF_1_Pin GPIO_PIN_2
#define LIMIT_INF_1_GPIO_Port GPIOE
#define LIMIT_SUP_1_Pin GPIO_PIN_3
#define LIMIT_SUP_1_GPIO_Port GPIOE
#define LIMIT_INF_2_Pin GPIO_PIN_4
#define LIMIT_INF_2_GPIO_Port GPIOE
#define LIMIT_SUP_2_Pin GPIO_PIN_5
#define LIMIT_SUP_2_GPIO_Port GPIOE
#define LIMIT_INF_3_Pin GPIO_PIN_6
#define LIMIT_INF_3_GPIO_Port GPIOE
#define Amarillo_Pin GPIO_PIN_0
#define Amarillo_GPIO_Port GPIOB
#define LIMIT_SUP_3_Pin GPIO_PIN_7
#define LIMIT_SUP_3_GPIO_Port GPIOE
#define LIMIT_INF_4_Pin GPIO_PIN_8
#define LIMIT_INF_4_GPIO_Port GPIOE
#define LIMIT_SUP_4_Pin GPIO_PIN_10
#define LIMIT_SUP_4_GPIO_Port GPIOE
#define LIMIT_INF_5_Pin GPIO_PIN_12
#define LIMIT_INF_5_GPIO_Port GPIOE
#define LIMIT_SUP_5_Pin GPIO_PIN_13
#define LIMIT_SUP_5_GPIO_Port GPIOE
#define LIMIT_INF_6_Pin GPIO_PIN_14
#define LIMIT_INF_6_GPIO_Port GPIOE
#define LIMIT_SUP_6_Pin GPIO_PIN_15
#define LIMIT_SUP_6_GPIO_Port GPIOE
#define Rojo_Pin GPIO_PIN_14
#define Rojo_GPIO_Port GPIOB
#define Azul_Pin GPIO_PIN_7
#define Azul_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
