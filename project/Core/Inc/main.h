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
#include "stm32l4xx_hal.h"

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
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOC
#define SPI1_CS_EXTI_IRQn EXTI4_IRQn
#define INPUT_IT_Pin GPIO_PIN_5
#define INPUT_IT_GPIO_Port GPIOC
#define INPUT_STATE_Pin GPIO_PIN_0
#define INPUT_STATE_GPIO_Port GPIOB
#define INPUT_B0_Pin GPIO_PIN_1
#define INPUT_B0_GPIO_Port GPIOB
#define INPUT_B1_Pin GPIO_PIN_2
#define INPUT_B1_GPIO_Port GPIOB
#define INPUT_B2_Pin GPIO_PIN_10
#define INPUT_B2_GPIO_Port GPIOB
#define INPUT_B3_Pin GPIO_PIN_11
#define INPUT_B3_GPIO_Port GPIOB
#define RIGHT_IND_Pin GPIO_PIN_7
#define RIGHT_IND_GPIO_Port GPIOC
#define LEFT_IND_Pin GPIO_PIN_8
#define LEFT_IND_GPIO_Port GPIOC
#define SPI3_CS_Pin GPIO_PIN_2
#define SPI3_CS_GPIO_Port GPIOD
#define DISPLAY_EXTCOMIN_Pin GPIO_PIN_3
#define DISPLAY_EXTCOMIN_GPIO_Port GPIOB
#define DISPLAY_DISP_Pin GPIO_PIN_4
#define DISPLAY_DISP_GPIO_Port GPIOB
#define FAULTLIGHT_CTRL_Pin GPIO_PIN_6
#define FAULTLIGHT_CTRL_GPIO_Port GPIOB
#define DISPLAY_EXTMODE_Pin GPIO_PIN_7
#define DISPLAY_EXTMODE_GPIO_Port GPIOB
#define READLIGHT_CTRL_Pin GPIO_PIN_8
#define READLIGHT_CTRL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
