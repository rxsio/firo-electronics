/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_RETRACT_Pin GPIO_PIN_3
#define MOTOR_RETRACT_GPIO_Port GPIOA
#define MOTOR_DEPLOY_Pin GPIO_PIN_4
#define MOTOR_DEPLOY_GPIO_Port GPIOA
#define CHARGE_PULSE_Pin GPIO_PIN_5
#define CHARGE_PULSE_GPIO_Port GPIOA
#define BATTERY_CUTOFF_Pin GPIO_PIN_10
#define BATTERY_CUTOFF_GPIO_Port GPIOB
#define CHARGE_SENSE_Pin GPIO_PIN_12
#define CHARGE_SENSE_GPIO_Port GPIOB
#define PROBE_RETRACTED_Pin GPIO_PIN_13
#define PROBE_RETRACTED_GPIO_Port GPIOB
#define PROBE_RETRACTED_EXTI_IRQn EXTI15_10_IRQn
#define PROBE_EXTENDED_Pin GPIO_PIN_14
#define PROBE_EXTENDED_GPIO_Port GPIOB
#define PROBE_EXTENDED_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR_PWM_Pin GPIO_PIN_7
#define MOTOR_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MOTOR_PWM_TIM htim4
#define MOTOR_PWM_Channel TIM_CHANNEL_2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
