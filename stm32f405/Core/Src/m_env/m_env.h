/*
 * m_env.h
 *
 *  Created on: Feb 9, 2024
 *      Author: Robyn
 */

#ifndef SRC_M_ENV_M_ENV_H_
#define SRC_M_ENV_M_ENV_H_
/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void m_env_check_temp(void);
uint8_t m_env_serial_select(void);
uint8_t m_dipsw_read_Channel(void);
uint8_t m_isEnable_Ethernet(void);
int m_env_e2p_write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
int m_env_e2p_read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
/* Private defines -----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* SRC_M_ENV_M_ENV_H_ */
