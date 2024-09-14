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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define GRIPPER_IN1_Pin GPIO_PIN_1
#define GRIPPER_IN1_GPIO_Port GPIOB
#define GRIPPER_IN4_Pin GPIO_PIN_13
#define GRIPPER_IN4_GPIO_Port GPIOB
#define GRIPPER_IN3_Pin GPIO_PIN_14
#define GRIPPER_IN3_GPIO_Port GPIOB
#define GRIPPER_IN2_Pin GPIO_PIN_15
#define GRIPPER_IN2_GPIO_Port GPIOB
#define ROT_DIR_Pin GPIO_PIN_8
#define ROT_DIR_GPIO_Port GPIOA
#define ENABLE_STEPS_Pin GPIO_PIN_9
#define ENABLE_STEPS_GPIO_Port GPIOA
#define HIGH_STEP_Pin GPIO_PIN_10
#define HIGH_STEP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LOW_DIR_Pin GPIO_PIN_15
#define LOW_DIR_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define HIGH_DIR_Pin GPIO_PIN_4
#define HIGH_DIR_GPIO_Port GPIOB
#define ROT_STEP_Pin GPIO_PIN_5
#define ROT_STEP_GPIO_Port GPIOB
#define LOW_STEP_Pin GPIO_PIN_7
#define LOW_STEP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define HIGH_STEP_HTIM htim1
#define LOW_STEP_HTIM htim4
#define ROT_STEP_HTIM htim3
#define GRIPPER_HTIM htim10
#define HIGH_STEP_TIM_CHANNEL TIM_CHANNEL_3
#define LOW_STEP_TIM_CHANNEL TIM_CHANNEL_2
#define ROT_STEP_TIM_CHANNEL TIM_CHANNEL_2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
