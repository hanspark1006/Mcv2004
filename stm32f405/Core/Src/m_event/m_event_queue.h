/*
 * m_event_queue.h
 *
 *  Created on: Feb 6, 2024
 *      Author: catsa
 */

#ifndef SRC_M_EVENT_M_EVENT_QUEUE_H_
#define SRC_M_EVENT_M_EVENT_QUEUE_H_
/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "m_event_def.h"
#include "macros_common.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    event_type_t event;
	union
	{
		struct
		{
			uint32_t     param1;
			uint32_t     param2;
			uint32_t     param3;
		};
		uint32_t param[3];
	};
	void*    p_event_data;
	uint32_t event_data_size;
} event_t;

typedef void (*event_queue_handler_t)(event_t const* e, void* p_context);

typedef struct
{
	event_queue_handler_t handler;
	void*                 p_context;
} event_queue_observer_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
int m_event_init(void);
int m_event_register(event_queue_observer_t *pEvent);
void m_event_queue_clear(void);
int m_event_push(uint32_t evt, uint32_t param1, uint32_t param2, uint32_t param3, void* p_event_data, uint32_t event_data_size);
void m_event_dispatch(void);

inline static uint32_t push_event0(event_type_t event)                                      { return m_event_push(event, 0, 0, 0, NULL, 0); }
inline static uint32_t push_event1(event_type_t event, uint32_t param1)                     { return m_event_push(event, param1, 0, 0, NULL, 0); }
inline static uint32_t push_event2(event_type_t event, uint32_t param1, uint32_t param2)    { return m_event_push(event, param1, param2, 0, NULL, 0); }
inline static uint32_t push_event3(event_type_t event, uint32_t param1, uint32_t param2, uint32_t param3)    { return m_event_push(event, param1, param2, param3, NULL, 0); }

inline static uint32_t push_event0_param(event_type_t event, void* p_event_data, uint32_t event_data_size)                                      					{ return m_event_push(event, 0, 0, 0, p_event_data, event_data_size); }
inline static uint32_t push_event1_param(event_type_t event, uint32_t param1, void* p_event_data, uint32_t event_data_size)                     				{ return m_event_push(event, param1, 0, 0, p_event_data, event_data_size); }
inline static uint32_t push_event2_param(event_type_t event, uint32_t param1, uint32_t param2, void* p_event_data, uint32_t event_data_size)    				{ return m_event_push(event, param1, param2, 0, p_event_data, event_data_size); }
inline static uint32_t push_event3_param(event_type_t event, uint32_t param1, uint32_t param2, uint32_t param3, void* p_event_data, uint32_t event_data_size)   { return m_event_push(event, param1, param2, param3, p_event_data, event_data_size); }
/* Private defines -----------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* SRC_M_EVENT_M_EVENT_QUEUE_H_ */
