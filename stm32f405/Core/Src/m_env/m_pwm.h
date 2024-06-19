/*
 * m_pwm.h
 *
 *  Created on: Mar 8, 2024
 *      Author: catsa
 */

#ifndef SRC_M_ENV_M_PWM_H_
#define SRC_M_ENV_M_PWM_H_
/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct _pwm_timer{
//	uint8_t get_evt;
//	uint8_t gpio_set;
	int ncount;
	//int	nDelay;
	//int nPluse;
}PWM_Timer;
/* Exported constants --------------------------------------------------------*/
extern __IO int nPWMTimer[];
extern uint8_t inter[];
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void m_pwm_init(void);
void m_pwm_out(int channel, int data);
void m_pwm_stop(void);
void m_pwm_channel_change(void);
/* Private defines -----------------------------------------------------------*/
#define TIMER_UNIT		1  // 10us
#define INIT_DELAY_MS	10000001
#define ADD_DELAY_MS	10000000

#define CH1_OUT_Pin GPIO_PIN_15
#define CH1_OUT_GPIO_Port GPIOA
#define CH2_OUT_Pin GPIO_PIN_3
#define CH2_OUT_GPIO_Port GPIOB
#define CH3_OUT_Pin GPIO_PIN_10
#define CH3_OUT_GPIO_Port GPIOB
#define CH4_OUT_Pin GPIO_PIN_11
#define CH4_OUT_GPIO_Port GPIOB
#ifdef __cplusplus
}
#endif

#endif /* SRC_M_ENV_M_PWM_H_ */
