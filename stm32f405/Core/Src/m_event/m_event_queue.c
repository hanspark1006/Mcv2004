/*
 * m_event_queue.c
 *
 *  Created on: Feb 6, 2024
 *      Author: catsa
 */
#include "main.h"
#include "errno.h"
#include "cmsis_os.h"
#include "m_event_queue.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#define MAX_EVT	10

osMailQId	EventQ;

event_queue_observer_t g_Event_Handle[MAX_EVT];
int evt_handle_cnt = 0;
uint8_t evt_cnt = 0;
int m_event_register(event_queue_observer_t *pEvent)
{
	if(evt_handle_cnt < MAX_EVT){
		g_Event_Handle[evt_handle_cnt].handler = pEvent->handler;
		g_Event_Handle[evt_handle_cnt].p_context = pEvent->p_context;

		evt_handle_cnt++;
		return 0;
	}

	return 1;
}

int m_event_push(uint32_t evt, uint32_t param1, uint32_t param2, uint32_t param3, void* p_event_data, uint32_t event_data_size)
{
	int err;

    event_t *e;

    e = osMailAlloc(EventQ, osWaitForever);
    if(e == NULL){
    	LOG_ERR("Message Queue allocate Error!!\r\n");
    	return 0;
    }

    memset(e, 0, sizeof(event_t));
    e->event = evt;
    e->param1 = param1;
    e->param2 = param2;
    e->param3 = param3;

    if (event_data_size != 0 && p_event_data != NULL)
    {
        e->p_event_data = pvPortMalloc(event_data_size);
        e->event_data_size = event_data_size;

        if (e->p_event_data == NULL)
        {
            LOG_ERR("Event data allocate error");
            return -ENOMEM;
        }
        memcpy(e->p_event_data, p_event_data, event_data_size);
    }

    //LOG_DBG("Push evt[%d] [%s] e->p_event_data[%p] size[%d]", evt_cnt, event_type_2_str(e->event), e->p_event_data, e->event_data_size);
    do
    {
        err = osMailPut(EventQ, e);
        if (err != osOK)
        {
        	if(e->p_event_data){
        		vPortFree(e->p_event_data);
        	}
            LOG_WRN("event_queue could not be enqueued, error code: %d\r\n", err);

            osMailFree(EventQ, e);
            break;
        }
    } while (0);

    evt_cnt++;
    return err;
}

static void __free_event(event_t* e)
{
    if (e==NULL || e->p_event_data == NULL)
        return;
    vPortFree(e->p_event_data);
    e->p_event_data = NULL;
    e->event_data_size = 0;
}

void m_event_dispatch(void)
{
	osEvent event;
    static event_t *e;
	event_queue_observer_t *p_config;

    do
    {
        event = osMailGet(EventQ, osWaitForever);
        if (event.status == osEventMail){
        	e = event.value.p;

		//LOG_INF("EVT : %s(%d) (%d,%d,%d)\r\n", event_type_2_str(e->event), e->event, e->param1, e->param2, e->param3)

        for(int i = 0; i < evt_handle_cnt; i++){
			//LOG_INF("Call Evt[%d]", i)
        	p_config = &g_Event_Handle[i];
				if(p_config->handler)
					p_config->handler(e, p_config->p_context);
				else
					LOG_ERR("Check Handler!!!");
			}
        }
        // free memory allocated by event data
        __free_event(e);
        osMailFree(EventQ, e);
        evt_cnt--;
    } while (0);
}

int m_event_init(void)
{
	osMailQDef(EVENTQ, 20, event_t);
	EventQ = osMailCreate(osMailQ(EVENTQ), NULL);
	if(EventQ == NULL){
		LOG_ERR("Event Queue Create Error");
		return 1;
	}

	return 0;
}
