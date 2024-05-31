/*
 * m_event_def.c
 *
 *  Created on: Feb 6, 2024
 *      Author: catsa
 */
#include "main.h"
#include "m_event_def.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
const char* event_type_2_str(event_type_t evt)
{
    switch (evt)
    {
    	case_str(EVT_none)
		case_str(EVT_menu_start)
    	case_str(EVT_send_pwm)
		case_str(EVT_send_status)
    	case_str(EVT_received_data)
    	case_str(EVT_measure_start)
		case_str(EVT_received_tcp)
		case_str(EVT_Set_ip)
		case_str(EVT_Get_ip)
    	default :
			break;
    }

    return "Unknown event";
}

