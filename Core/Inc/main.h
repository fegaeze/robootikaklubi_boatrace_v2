/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define IR_LEFT_Pin GPIO_PIN_0
#define IR_LEFT_GPIO_Port GPIOA
#define IR_CENTER_Pin GPIO_PIN_1
#define IR_CENTER_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define IR_RIGHT_Pin GPIO_PIN_3
#define IR_RIGHT_GPIO_Port GPIOA
#define RIGHT_DM_ENBL_Pin GPIO_PIN_8
#define RIGHT_DM_ENBL_GPIO_Port GPIOA
#define LEFT_DM_ENBL_Pin GPIO_PIN_9
#define LEFT_DM_ENBL_GPIO_Port GPIOA
#define RIGHT_DM_PHASE_Pin GPIO_PIN_10
#define RIGHT_DM_PHASE_GPIO_Port GPIOA
#define LEFT_DM_PHASE_Pin GPIO_PIN_11
#define LEFT_DM_PHASE_GPIO_Port GPIOA
#define POWER_BTN_Pin GPIO_PIN_12
#define POWER_BTN_GPIO_Port GPIOA
#define POWER_BTN_EXTI_IRQn EXTI15_10_IRQn
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define SERVO_MOTOR_Pin GPIO_PIN_5
#define SERVO_MOTOR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
