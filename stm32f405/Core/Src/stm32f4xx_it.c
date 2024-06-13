/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "m_normal.h"
#include "m_pwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMER1	(&htim7)
#define TIMER2	(&htim12)
#define TIMER3	(&htim13)
#define TIMER4	(&htim14)

enum{
	DIMMING_MODE,
	STROBE_MODE,
	STROBE_TEST
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static int nStrobe = 0;
static int nStrobeTestCount = 25001;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_NORMAL){
		if(Device.nDELAY[0] == 0){
			GPIOA->BSRR = GPIO_PIN_15;
			nPWMTimer[0] = INIT_DELAY_MS;
		}else{
			nPWMTimer[0] = Device.nDELAY[0]+ADD_DELAY_MS;
		}
	}
#if 1
	if(__HAL_GPIO_EXTI_GET_IT(EXT_INT1_Pin) != 0x00u)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(EXT_INT1_Pin);
	}
#else
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXT_INT1_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
#endif
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_NORMAL){
		if(Device.nDELAY[1] == 0){
			GPIOB->BSRR = GPIO_PIN_3;
			nPWMTimer[1] = INIT_DELAY_MS;
		}else{
			nPWMTimer[1] = Device.nDELAY[1]+ADD_DELAY_MS;
		}
	}
#if 1
	if(__HAL_GPIO_EXTI_GET_IT(EXT_INT2_Pin) != 0x00u)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(EXT_INT2_Pin);
	}
#else
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXT_INT2_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
#endif
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_NORMAL){
		if(Device.nDELAY[3] == 0){
			GPIOB->BSRR = GPIO_PIN_11;
			nPWMTimer[3] = INIT_DELAY_MS;
		}else{
			nPWMTimer[3] = Device.nDELAY[3]+ADD_DELAY_MS;
		}
	}
#if 1
	if(__HAL_GPIO_EXTI_GET_IT(EXT_INT4_Pin) != 0x00u)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(EXT_INT4_Pin);
	}
#else
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXT_INT4_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
#endif
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_NORMAL){
		if(Device.nDELAY[2] == 0){
			GPIOB->BSRR = GPIO_PIN_10;
			nPWMTimer[2] = INIT_DELAY_MS;
		}else{
			nPWMTimer[2] = Device.nDELAY[2]+ADD_DELAY_MS;
		}
	}
#if 1
	if(__HAL_GPIO_EXTI_GET_IT(EXT_INT3_Pin) != 0x00u)
	{
		__HAL_GPIO_EXTI_CLEAR_IT(EXT_INT3_Pin);
	}
#else
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXT_INT3_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
#endif
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */
#if 1
	if (nStrobe == STROBE_MODE){
		if(nPWMTimer[1] > 0){
			nPWMTimer[1]-=TIMER_UNIT;
			if(nPWMTimer[1] == ADD_DELAY_MS){
				GPIOB->BSRR = CH2_OUT_Pin;
				nPWMTimer[1] = Device.nPULSE[1];
			}else if(nPWMTimer[1] <= 0){
				GPIOB->BSRR = 0x00080000;//(uint32_t)GPIO_PIN_3 << 16u;
			}
		}
	}
#else
  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */
#endif
  if (__HAL_TIM_GET_FLAG(TIMER2, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(TIMER2, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(TIMER2, TIM_IT_UPDATE);
    }
  }
  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */
#if 1
	if (nStrobe == STROBE_MODE){
		if(nPWMTimer[2] > 0){
			nPWMTimer[2]-=TIMER_UNIT;
			if(nPWMTimer[2] == ADD_DELAY_MS){
				GPIOB->BSRR = CH3_OUT_Pin;
			}else if(nPWMTimer[2] <= 0){
				GPIOB->BSRR = 0x04000000;//(uint32_t)GPIO_PIN_10 << 16u;
			}
		}
	}
#else
  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim13);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */
#endif
  if (__HAL_TIM_GET_FLAG(TIMER3, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(TIMER3, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(TIMER3, TIM_IT_UPDATE);
    }
  }
  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */
#if 1
	if (nStrobe == STROBE_MODE){
		if(nPWMTimer[3] > 0){
			nPWMTimer[3]-=TIMER_UNIT;
			if(nPWMTimer[3] == ADD_DELAY_MS){
				GPIOB->BSRR = CH4_OUT_Pin;
			}else if(nPWMTimer[3] <= TIMER_UNIT){
				GPIOB->BSRR = 0x08000000;//(uint32_t)GPIO_PIN_11 << 16u;
			}
		}
	}
#else
  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */
#endif
  if (__HAL_TIM_GET_FLAG(TIMER4, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(TIMER4, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(TIMER4, TIM_IT_UPDATE);
    }
  }
  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
#if 1
	if(nStrobe == STROBE_MODE){
		if(nPWMTimer[0] > 0){
			nPWMTimer[0]-=TIMER_UNIT;
			if(nPWMTimer[0] == ADD_DELAY_MS){
				GPIOB->BSRR = CH1_OUT_Pin;
				nPWMTimer[0] = Device.nPULSE[0];
			}
		}else if(nPWMTimer[0] <= 0){
			GPIOA->BSRR = 0x80000000;//(uint32_t)GPIO_PIN_15 << 16u;
			nPWMTimer[0] = 0;
		}
	}else if (nStrobe == STROBE_TEST){
		nStrobeTestCount--;
		if (nStrobeTestCount == 100000){
			GPIOA->BSRR = 0x80000000;
			GPIOB->BSRR = 0x00080000;//(uint32_t)GPIO_PIN_3 << 16u;
			GPIOB->BSRR = 0x04000000;//(uint32_t)GPIO_PIN_10 << 16u;
			GPIOB->BSRR = 0x08000000;//(uint32_t)GPIO_PIN_11 << 16u;
		}else if(nStrobeTestCount == 100){
			GPIOA->BSRR = GPIO_PIN_15;
			GPIOB->BSRR = GPIO_PIN_3;
			GPIOB->BSRR = GPIO_PIN_10;
			GPIOB->BSRR = GPIO_PIN_11;
		}else if(nStrobeTestCount <= 0){
			nStrobeTestCount =  100001;
		}
	}
#else
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
#endif
  if (__HAL_TIM_GET_FLAG(TIMER1, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(TIMER1, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(TIMER1, TIM_IT_UPDATE);
    }
  }
  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void setCurDeviceStatus(void)
{
	if ((Device.nDeviceCurrentStatus >= LCD_STATUS_MODE_STROBE_NORMAL) && (Device.nDeviceCurrentStatus <= LCD_STATUS_MODE_STROBE_REMOTE)){
		nStrobe = STROBE_MODE;
	}else if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_TEST){
		nStrobe = STROBE_TEST;
	}else{
		nStrobe = DIMMING_MODE;
	}
}
/* USER CODE END 1 */
