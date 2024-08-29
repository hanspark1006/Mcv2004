/*
 * m_event_def.h
 *
 *  Created on: Feb 6, 2024
 *      Author: catsa
 */

#ifndef SRC_M_EVENT_M_EVENT_DEF_H_
#define SRC_M_EVENT_M_EVENT_DEF_H_
/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

/* Private defines -----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#include "macros_common.h"

typedef enum
{
	EVT_none,
	EVT_menu_start,
	EVT_send_pwm,
	EVT_send_status,
	EVT_received_data,
	EVT_measure_start,
	EVT_received_tcp,
	EVT_Set_ip,
	EVT_Get_ip,
	EVT_Disable_ip,
	EVT_eth_enable,
} event_type_t;

const char* event_type_2_str(event_type_t evt);

#endif /* SRC_M_EVENT_M_EVENT_DEF_H_ */
