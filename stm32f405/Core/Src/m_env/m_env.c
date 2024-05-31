/*
 * m_env.c
 *
 *  Created on: Feb 9, 2024
 *      Author: Robyn
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "m_normal.h"
#include "m_env.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_DIP_SW_CHANNEL	4

#define I2C_TIMEOUT			1000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t bSW[MAX_DIP_SW_CHANNEL] = {1,};
/* Private function prototypes -----------------------------------------------*/
uint8_t m_dipsw_read_Channel(void)
{
	GPIO_TypeDef *dipsw_port[MAX_DIP_SW_CHANNEL] ={DIPSW1_GPIO_Port, DIPSW2_GPIO_Port, DIPSW3_GPIO_Port, DIPSW4_GPIO_Port};
	uint16_t	dipsw_pin[MAX_DIP_SW_CHANNEL] = {DIPSW1_Pin, DIPSW2_Pin, DIPSW3_Pin, DIPSW4_Pin};
	unsigned int status = 0x00;
	int flag = 0, i;

	for(i=0; i < MAX_DIP_SW_CHANNEL; i++){
		status = HAL_GPIO_ReadPin(dipsw_port[i], dipsw_pin[i]);
		if(bSW[i] != status){
			bSW[i] = status;
		}
		if(bSW[i] == GPIO_PIN_RESET){
			if(i==2){
				flag |= 0x02;
			}else if(i == 3){
				flag |= 0x01;
			}
		}
	}

	return flag;
}

int m_env_e2p_write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Mem_Write(hi2c, DevAddress,  MemAddress,  MemAddSize, pData, Size, I2C_TIMEOUT);
	if(ret != HAL_OK)
	{
		LOG_ERR("[%02x]I2c_Write_E2p Error_no [%d]", DevAddress, ret);
		return -1;
	}
	return 0;
}

int m_env_e2p_read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef ret;

	ret = HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, I2C_TIMEOUT);
	if(ret != HAL_OK)
	{
		LOG_ERR("[%02x]I2c_Read_E2p Error_no [%d]", DevAddress, ret);
		return -1;
	}
	return 0;
}

uint8_t m_env_serial_select(void)
{
	return HAL_GPIO_ReadPin(INPUT_SEL_GPIO_Port, INPUT_SEL_Pin);
}

void m_env_check_temp(void)
{
#if 0
	uint8_t detect_temp = 0;

	detect_temp = HAL_GPIO_ReadPin(TEMP_SENS1_GPIO_Port, TEMP_SENS1_Pin);
	detect_temp |= HAL_GPIO_ReadPin(TEMP_SENS1_GPIO_Port, TEMP_SENS1_Pin);
	if(detect_temp)
		HAL_GPIO_WritePin(FAN_CTRL_GPIO_Port, FAN_CTRL_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(FAN_CTRL_GPIO_Port, FAN_CTRL_Pin, GPIO_PIN_RESET);
#endif
}
