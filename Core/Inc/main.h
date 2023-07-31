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
#define MAIN_FLAG_RUN	0x01
#define MAIN_FLAG_NEXT	0x02
#define MAIN_FLAG_LOAD	0x04
#define MAIN_FLAG_DISP	0x80
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW_RG_Pin GPIO_PIN_13
#define SW_RG_GPIO_Port GPIOC
#define SW_UP_Pin GPIO_PIN_14
#define SW_UP_GPIO_Port GPIOC
#define LED_SEN_EN_Pin GPIO_PIN_15
#define LED_SEN_EN_GPIO_Port GPIOC
#define MEM_WP_Pin GPIO_PIN_12
#define MEM_WP_GPIO_Port GPIOB
#define SW_HM_Pin GPIO_PIN_13
#define SW_HM_GPIO_Port GPIOB
#define SW_OK_Pin GPIO_PIN_14
#define SW_OK_GPIO_Port GPIOB
#define SW_BK_Pin GPIO_PIN_15
#define SW_BK_GPIO_Port GPIOB
#define LED_BUILTIN_Pin GPIO_PIN_12
#define LED_BUILTIN_GPIO_Port GPIOC
#define SW_DW_Pin GPIO_PIN_4
#define SW_DW_GPIO_Port GPIOB
#define SW_LF_Pin GPIO_PIN_5
#define SW_LF_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern uint8_t main_flag;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
