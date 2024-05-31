/*
 * m_serial.h
 *
 *  Created on: Feb 8, 2024
 *      Author: catsa
 */

#ifndef SRC_M_SERIAL_M_SERIAL_H_
#define SRC_M_SERIAL_M_SERIAL_H_
/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/
enum{
	UART_RECV_EXT,
	UART_RECV_FRONT,
#ifdef _UART485
	UART_RECV_485
#endif
};

enum{
	DIGIT_1,
	DIGIT_3,
	RET_OK,
	RET_IP,
};
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
#ifdef _UART485
void RS485_Enable_Int_Handle(void);
#endif
void m_serial_init(void);
void m_serial_SendFront(int protocol, int data);
void m_serial_SendPC(uint8_t digit, void *data);
void push_ext_buf(uint8_t ch);
/* Private defines -----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* SRC_M_SERIAL_M_SERIAL_H_ */
