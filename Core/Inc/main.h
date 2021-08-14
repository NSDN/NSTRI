/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define CTL_MODE_Pin GPIO_PIN_0
#define CTL_MODE_GPIO_Port GPIOA
#define ADC_V_Pin GPIO_PIN_1
#define ADC_V_GPIO_Port GPIOA
#define ADC_W_Pin GPIO_PIN_2
#define ADC_W_GPIO_Port GPIOA
#define ADC_PWR_Pin GPIO_PIN_3
#define ADC_PWR_GPIO_Port GPIOA
#define OUT_UN_Pin GPIO_PIN_7
#define OUT_UN_GPIO_Port GPIOA
#define OUT_VN_Pin GPIO_PIN_0
#define OUT_VN_GPIO_Port GPIOB
#define OUT_WN_Pin GPIO_PIN_1
#define OUT_WN_GPIO_Port GPIOB
#define OUT_U_Pin GPIO_PIN_8
#define OUT_U_GPIO_Port GPIOA
#define OUT_V_Pin GPIO_PIN_9
#define OUT_V_GPIO_Port GPIOA
#define OUT_W_Pin GPIO_PIN_10
#define OUT_W_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_11
#define KEY1_GPIO_Port GPIOA
#define KEY1_EXTI_IRQn EXTI15_10_IRQn
#define KEY2_Pin GPIO_PIN_12
#define KEY2_GPIO_Port GPIOA
#define KEY2_EXTI_IRQn EXTI15_10_IRQn
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOA
#define CKP_Pin GPIO_PIN_3
#define CKP_GPIO_Port GPIOB
#define CKP_EXTI_IRQn EXTI3_IRQn
#define CKU_Pin GPIO_PIN_4
#define CKU_GPIO_Port GPIOB
#define CKU_EXTI_IRQn EXTI4_IRQn
#define CKD_Pin GPIO_PIN_5
#define CKD_GPIO_Port GPIOB
#define CKD_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
