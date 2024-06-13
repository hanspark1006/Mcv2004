/*
 * m_pwm.c
 *
 *  Created on: Mar 8, 2024
 *      Author: catsa
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "m_normal.h"
#include "m_pwm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PWM_TIMER	(&htim2)
#define TIMER_1		(&htim7)
#define	TIMER_2		(&htim12)
#define	TIMER_3		(&htim13)
#define	TIMER_4		(&htim14)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO int nPWMTimer[MAX_PWM_CH] = {0,};
TIM_OC_InitTypeDef sConfigOC_;
/* Private function prototypes -----------------------------------------------*/
void m_pwm_out(int channel, int data)
{
	uint32_t tim_ch[MAX_PWM_CH]={TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

	//LOG_INF("%s ch[%d] data[%d]", __func__, channel, data);

	sConfigOC_.Pulse = (uint32_t)data;
#if 0
	if(HAL_TIM_PWM_Stop(PWM_TIMER, tim_ch[channel])!=HAL_OK){
		LOG_ERR("PWM Stop Error!!\r\n");
	}

	if(HAL_TIM_PWM_ConfigChannel(PWM_TIMER, &sConfigOC_, tim_ch[channel]) != HAL_OK){
		LOG_ERR("PWM Set config Error!!\r\n");
	}

	if(HAL_TIM_PWM_Start(PWM_TIMER, tim_ch[channel]) != HAL_OK){
		LOG_ERR("PWM Start Error!!\r\n");
	}
#else
	if(HAL_TIM_PWM_ConfigChannel(PWM_TIMER, &sConfigOC_, tim_ch[channel]) != HAL_OK){
		LOG_ERR("PWM Set config Error!!\r\n");
	}
#endif
	m_normal_DataSend2(channel, data);
}

void m_pwm_stop(void)
{
	uint32_t tim_ch[MAX_PWM_CH]={TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

	LOG_DBG("PWM Disable!!\r\n");

	for(int i = 0; i < MAX_PWM_CH; i++){
		HAL_TIM_PWM_Stop(PWM_TIMER, tim_ch[i]);
	}
	HAL_TIM_PWM_DeInit(PWM_TIMER);
}

void set_gpio_out_port(void)
{
	GPIO_TypeDef *gpio_port[MAX_PWM_CH] = {CH1_OUT_GPIO_Port, CH2_OUT_GPIO_Port, CH3_OUT_GPIO_Port, CH4_OUT_GPIO_Port};
	uint16_t gpio_pin[MAX_PWM_CH] = {CH1_OUT_Pin, CH2_OUT_Pin, CH3_OUT_Pin, CH4_OUT_Pin};

	for(int i = 0; i < MAX_PWM_CH; i++){
		HAL_GPIO_WritePin(gpio_port[i], gpio_pin[i], GPIO_PIN_RESET);
	}

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = CH1_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CH1_OUT_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = CH2_OUT_Pin | CH3_OUT_Pin | CH4_OUT_Pin | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CH2_OUT_GPIO_Port, &GPIO_InitStruct);
}

void m_pwm_channel_change(void)
{
	set_gpio_out_port();

	HAL_TIM_Base_Start_IT(TIMER_1);
	HAL_TIM_Base_Start_IT(TIMER_2);
	HAL_TIM_Base_Start_IT(TIMER_3);
	HAL_TIM_Base_Start_IT(TIMER_4);
}

void m_pwm_init(void)
{
	uint32_t tim_ch[MAX_PWM_CH]={TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
	int i;

	sConfigOC_.OCMode = TIM_OCMODE_PWM1;
	sConfigOC_.Pulse = 0;
	sConfigOC_.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC_.OCFastMode = TIM_OCFAST_ENABLE;

	for(i = 0; i < MAX_PWM_CH; i++){
		HAL_TIM_PWM_Start(PWM_TIMER, tim_ch[i]);
	}
}
