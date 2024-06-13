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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "log.h"
#include "m_event_queue.h"
#include "errno.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void JumpBootLoader(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OPLED_Pin GPIO_PIN_13
#define OPLED_GPIO_Port GPIOC
#define STATUS_Pin GPIO_PIN_14
#define STATUS_GPIO_Port GPIOC
#define ETH_NSS_Pin GPIO_PIN_4
#define ETH_NSS_GPIO_Port GPIOA
#define EXT_INT1_Pin GPIO_PIN_0
#define EXT_INT1_GPIO_Port GPIOB
#define EXT_INT1_EXTI_IRQn EXTI0_IRQn
#define EXT_INT2_Pin GPIO_PIN_1
#define EXT_INT2_GPIO_Port GPIOB
#define EXT_INT2_EXTI_IRQn EXTI1_IRQn
#define DIPSW1_Pin GPIO_PIN_12
#define DIPSW1_GPIO_Port GPIOB
#define DIPSW2_Pin GPIO_PIN_13
#define DIPSW2_GPIO_Port GPIOB
#define DIPSW3_Pin GPIO_PIN_14
#define DIPSW3_GPIO_Port GPIOB
#define DIPSW4_Pin GPIO_PIN_15
#define DIPSW4_GPIO_Port GPIOB
#define INPUT_SEL_Pin GPIO_PIN_8
#define INPUT_SEL_GPIO_Port GPIOC
#define EXT_INT4_Pin GPIO_PIN_4
#define EXT_INT4_GPIO_Port GPIOB
#define EXT_INT4_EXTI_IRQn EXTI4_IRQn
#define EXT_INT3_Pin GPIO_PIN_5
#define EXT_INT3_GPIO_Port GPIOB
#define EXT_INT3_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
