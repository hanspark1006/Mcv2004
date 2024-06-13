/*
 * m_normal.c
 *
 *  Created on: Feb 9, 2024
 *      Author: Robyn
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_it.h"

#include "m_normal.h"
#include "m_env.h"
#include "m_serial.h"
#include "m_pwm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PWM_FREQ_140Khz		300-1
#define PWM_FREQ_300Khz		140-1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osTimerId measureTimerHandle;
event_queue_observer_t menu_event;
static int g_Channels;
uint8_t g_RecvData = 0;
int g_FirmwareVersion = 0;

DEVICE Device;

uint8_t bEX_INPUT[MAX_PWM_CH] = {1,};
int bSerialOnOff[4] = {0, 0, 0, 0};
int nChangeFlag = 0;
int g_pwm_freq = PWM_FREQ_140Khz;
/* Private function prototypes -----------------------------------------------*/
static void check_ext_input(void);

void on_measure(void const * argument)
{
	static int old_status = -1;

	//m_env_check_temp();

	if(old_status != Device.nDeviceCurrentStatus){
		LOG_DBG("Status[%d] Channel[%d]", Device.nDeviceCurrentStatus, g_Channels);
		old_status = Device.nDeviceCurrentStatus;
	}

	if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_EXT){
		check_ext_input();
	}
}

static void check_ext_input(void)
{
	unsigned int status = 0x00;
	uint8_t i;
	GPIO_TypeDef *port[]={EXT_INT1_GPIO_Port, EXT_INT2_GPIO_Port, EXT_INT3_GPIO_Port, EXT_INT4_GPIO_Port};
	uint16_t pin[]={EXT_INT1_Pin, EXT_INT2_Pin, EXT_INT3_Pin, EXT_INT4_Pin};

	for(i = 0; i < MAX_PWM_CH; i++){
		status = HAL_GPIO_ReadPin(port[i], pin[i]);
		if (bEX_INPUT[i] != status){
			bEX_INPUT[i] = status;
			if (bEX_INPUT[i] == GPIO_PIN_RESET){
				m_pwm_out(i, Device.nPWM[i]);
			}else{
				m_pwm_out(i, 0);
			}
		}
	}
}

int num_data_modify(uint8_t *pRX_data)
{
	if ((Device.nDeviceNo == 1) && (*pRX_data >= 0x30) && (*pRX_data < 0x34)){
		*pRX_data -= 0x00; //????
	}else if ((Device.nDeviceNo == 2) && (*pRX_data >= 0x34) && (*pRX_data < 0x38)){
		*pRX_data -= 0x04;
	}else if ((Device.nDeviceNo == 3) && (*pRX_data >= 0x38) && (*pRX_data < 0x3C)){
		*pRX_data -= 0x08;
	}else if ((Device.nDeviceNo == 4) && (*pRX_data >= 0x3C) && (*pRX_data < 0x40)){
		*pRX_data -= 0x0C;
	}else{
		return 1;
	}

	return 0;
}

uint8_t parse_remote(uint8_t *nRx1_data)
{
	if ((nRx1_data[4] == 0x02) && (nRx1_data[7] == 0x03)){
		//LOG_HEX_DUMP(nRx1_data, 8, "Remote4");
		if(num_data_modify(&nRx1_data[5])){
			return 1;
		}

		int idx = nRx1_data[5] - 0x30;
		if (nRx1_data[6] == 'o'){
			m_pwm_out(idx, Device.nSerialPWM[idx]);
			bSerialOnOff[idx] = 1;
		}else{
			m_pwm_out(idx, 0);
			bSerialOnOff[idx] = 0;
		}
	}else if ((nRx1_data[0] == 0x02) && (nRx1_data[7] == 0x03)){
		//LOG_HEX_DUMP(nRx1_data, 8, "Remote8");
		if(num_data_modify(&nRx1_data[1])){
			return 1;
		}

		int idx = nRx1_data[1] - 0x30;
		if (nRx1_data[2] == 'w'){
			int adc = 0;
			adc = (nRx1_data[3] - 0x30) * 1000;
			adc += ((nRx1_data[4] - 0x30) * 100);
			adc += ((nRx1_data[5] - 0x30) * 10);
			adc += (nRx1_data[6] - 0x30);
			adc /= 4;
			Device.nSerialPWM[idx] = adc;

			m_pwm_out(idx, Device.nSerialPWM[idx]);
		}
	}else if((nRx1_data[0] == 0xEF) && (nRx1_data[7] == 0xFE)){  // Set IP Address
		LOG_HEX_DUMP(nRx1_data, 8, "Set IP");
		push_event0_param(EVT_Set_ip, &nRx1_data[1], 6);
	}else if((nRx1_data[0] == 0xDF) && (nRx1_data[7] == 0xFD)){	// Read IP Address
		LOG_HEX_DUMP(nRx1_data, 8, "Read IP");
		push_event0(EVT_Get_ip);
	}

	return 0;
}

void find_character(uint8_t RxData, int *arg)
{
	if (RxData == 'A') *arg = 10;
	else if (RxData == 'B') *arg = 11;
	else if (RxData == 'C') *arg = 12;
	else if (RxData == 'D') *arg = 13;
	else if (RxData == 'E') *arg = 14;
	else if (RxData == 'F') *arg = 15;
}

void parse_strobe(uint8_t *nRx1_data)
{
	int i = 0;

	if ((nRx1_data[22] == '\r') && (nRx1_data[23] == '\n')){
		int id = 0;
		int comm = 0;
		int type = 0;
		int channel = 0;
		int data = 0;
		for (i = 21; i >= 0; i--){
			if (nRx1_data[i] == ':') break;
		}

		if (i >= 0){
			id = nRx1_data[i + 1] - 0x30;
			if (id > 9){
				find_character(nRx1_data[i + 1], &id);
			}
			comm = nRx1_data[i + 2];
			type = nRx1_data[i + 3] - 0x30;
			if (type > 9){
				find_character(nRx1_data[i + 3], &type);
			}
			channel = nRx1_data[i + 4] - 0x30;
			if (channel > 9){
				find_character(nRx1_data[i + 4], &channel);
			}

			if (i == 16){
				data = (int)(nRx1_data[i + 5] - 0x30);
			}else if (i == 14){
				data = (int)(nRx1_data[i + 5] - 0x30) * 100;
				data += (int)(nRx1_data[i + 6] - 0x30) * 10;
				data += (int)(nRx1_data[i + 7] - 0x30);
			}

			if (comm == 'W'){
				if (type == 2){
					Device.nPULSE[channel - 1] = data;
					m_serial_SendFront(0xB0 + (channel - 1), data);
				}else if (type == 3){
					Device.nDELAY[channel - 1] = data;
					m_serial_SendFront(0xA0 + (channel - 1), data);
				}else if (type == 8){
					if (data == 1){
						if (Device.nDELAY[channel-1] == 0){
							nPWMTimer[channel-1] = INIT_DELAY_MS;
						}else{
							//nPWMTimer[channel-1].nDelay = Device.nDELAY[channel-1];
							nPWMTimer[channel-1] = Device.nDELAY[channel-1] + ADD_DELAY_MS;
						}
					}
				}else if (type == 10){
					Device.nEDGE[channel - 1] = data;
					m_serial_SendFront(0xC0 + (channel - 1), data);
				}else if ((type == 15) && (channel == 15)){
				// �����ϴ� �ڵ�

				}
			}else if (comm == 'R'){
				if (type == 2){
					m_serial_SendPC(DIGIT_3, &Device.nPULSE[channel-1]);
				}else if (type == 3){
					m_serial_SendPC(DIGIT_3, &Device.nDELAY[channel-1]);
				}else if (type == 10){
					m_serial_SendPC(DIGIT_1, &Device.nEDGE[channel-1]);
				}
			}
		}
		for (i = 0; i < 24; i++){
			nRx1_data[i] = 0;
		}
	}
}

//extern void dump_serial_buffer(void);
void parse_front(uint8_t *nRx2_data)
{
	uint16_t pin[]={EXT_INT1_Pin, EXT_INT2_Pin, EXT_INT3_Pin, EXT_INT4_Pin};
	uint8_t	ch_idx;

	if (nRx2_data[0] >= 0x80){
		char recv_data[10];
		unsigned char protocol = nRx2_data[0];
		unsigned char channel = nRx2_data[1];

		//LOG_HEX_DUMP(nRx2_data, 8, "Front");

		for(int i=0, j=2; i < 6; i++,j++){
			recv_data[i]=nRx2_data[j];
		}
		recv_data[6]=0;

		int data = atoi(recv_data);
		if (protocol == DEVICE_STATUS)
		{
			//LOG_INF("CurStatus:%d, RecvStatus[%d] DeviceNo[%d]", Device.nDeviceCurrentStatus, data, Device.nDeviceNo);
			Device.nDeviceCurrentStatus = data;
			if (Device.nDeviceCurrentStatus != Device.nDeviceCurrentStatus_old){
				Device.nDeviceCurrentStatus_old = Device.nDeviceCurrentStatus;
				setCurDeviceStatus();
				nChangeFlag = 1;
			}

			if (Device.nDeviceNo == 1){
				m_serial_SendFront(DEVICE_ECHO, DEVICE_FIRMWARE_VERSION);
			}

			if ((Device.nDeviceCurrentStatus >= LCD_STATUS_MODE_LOCAL) && (Device.nDeviceCurrentStatus <= LCD_STATUS_MODE_EXT))	{
				if (Device.nDeviceCurrentStatus == LCD_STATUS_MODE_LOCAL){
					for(int ch = 0; ch < MAX_PWM_CH; ch++){
						m_pwm_out(ch, Device.nPWM[ch]);
					}
				}else if (Device.nDeviceCurrentStatus == LCD_STATUS_MODE_REMOTE){
					if (nChangeFlag == 1){
						for(int ch = 0; ch < MAX_PWM_CH; ch++){
							m_pwm_out(ch, 0);
							bSerialOnOff[ch] = 0;
						}
					}
				}else if (Device.nDeviceCurrentStatus == LCD_STATUS_MODE_EXT){
					if (nChangeFlag == 1){
						for(int input = 0; input < MAX_PWM_CH; input++){
							if(bEX_INPUT[input] == 0) m_pwm_out(input, Device.nPWM[input]);
							else m_pwm_out(input, 0);
						}
					}
				}
				nChangeFlag = 0;
			}else{
				for(int cnt = 0; cnt < MAX_PWM_CH; cnt++){
					m_pwm_out(cnt, 0);
				}
				//dump_serial_buffer();
				m_pwm_stop();
				m_pwm_channel_change();
			}
		}else if (protocol == STROBE_DELAY){
			if ((channel >= 0x30) && (channel <= 0x33)) Device.nDELAY[channel-0x30]=data;
		}else if (protocol == STROBE_PULSE){
			if ((channel >= 0x30) && (channel <=0x33)) Device.nPULSE[channel-0x30]=data;
		}else if (protocol == STROBE_RISEFALL){
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			if ((channel >= 0x30) && (channel <= 0x33)){
				ch_idx = channel - 0x30;
				Device.nEDGE[ch_idx]=data;
				GPIO_InitStruct.Pin = pin[ch_idx];
				if (Device.nEDGE[ch_idx] == 0)
				{
					GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
				}
				else if (Device.nEDGE[ch_idx] == 1)
				{
					GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
				}
				else
				{
					GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
				}
				GPIO_InitStruct.Pull = GPIO_PULLUP;
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
			}
		}
		else if (protocol == CV_PWM)
		{
			uint8_t ch_base=0x30;
			data *= 100;
			data = ((data * g_pwm_freq)/100)/100;
			if (Device.nDeviceNo >= 1 && Device.nDeviceNo <= 6 ){
				ch_base += (Device.nDeviceNo-1)*4;
				ch_idx = channel - ch_base;
				//LOG_INF("channel[%d] idx[%d] data[%d]", channel, ch_idx, data);
				if(ch_idx < 4){
					Device.nPWM[ch_idx]=data;
				}
			}
		}
		else if (protocol == DEVICE_UPDATE)
		{
			if (data == 1) JumpBootLoader();
		}
	}
}

void onPushTcpData(uint8_t *pData, int size)
{
	int i;

	for(i = 0; i < size; i++){
		push_ext_buf(pData[i]);
	}
	return ;
}

void m_normal_parse_received_data(uint8_t recv_type, uint8_t *rcv_data, uint32_t len)
{
	if(recv_type == UART_RECV_EXT){
		if (Device.nDeviceCurrentStatus == LCD_STATUS_MODE_REMOTE){
			parse_remote(rcv_data);
		}else if (Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_REMOTE){
			parse_strobe(rcv_data);
		}
	}else if(recv_type == UART_RECV_FRONT){
		parse_front(rcv_data);
	}
}

static void evt_handler(event_t const* evt, void* p_context)
{
	switch (evt->event)
	{
		case EVT_menu_start:
			osTimerStart(measureTimerHandle,200); // 200msec
			break;
		case EVT_received_tcp:
			onPushTcpData(evt->p_event_data, evt->event_data_size);
			break;
		default:
			break;
	}
}

void m_normal_DataSend2(uint8_t channel, int data)
{
	if (channel == 0){
		if ((Device.nDeviceCurrentStatus >= LCD_STATUS_MODE_LOCAL) && (Device.nDeviceCurrentStatus <= LCD_STATUS_MODE_EXT)){
			if (data == 0) m_serial_SendFront(SENSING_LED, 10);
			else m_serial_SendFront(SENSING_LED, 11);
		}
	}
}

int m_menu_normal_init(void)
{
	LOG_INF("Menu Init");
	osTimerDef(measure_timer, on_measure);
	measureTimerHandle = osTimerCreate(osTimer(measure_timer), osTimerPeriodic, NULL);
	if(measureTimerHandle==NULL){
		LOG_ERR("measureTimer Create Error!!");
		return 1;
	}

	menu_event.handler = evt_handler;
	menu_event.p_context = NULL;
	if(m_event_register(&menu_event)){
		return 1;
	}

	Device.nDeviceNo = 0x00;
	Device.nDeviceCurrentStatus = 0;
	Device.nDeviceCurrentStatus_old = -1;
	for (int i = 0; i < MAX_PWM_CH; i++)
	{
		Device.nPWM[i]=0;
		Device.nDELAY[i]=0;
		Device.nPULSE[i]=100;
		Device.nEDGE[i]=1;
		Device.nSerialPWM[i]=0xff;
	}

	g_Channels = m_dipsw_read_Channel();

	if(Device.nDeviceNo == 0){
		g_Channels++;
		Device.nDeviceNo = g_Channels;
	}
	LOG_INF("Total Device num:%d", g_Channels);

	for(int i = 0; i < MAX_PWM_CH; i++){
		m_pwm_out(i, 0);
	}

	return 0;
}
