/*
 * m_serial.c
 *
 *  Created on: Feb 8, 2024
 *      Author: catsa
 */


/*
 * uart_proc.c
 *
 *  Created on: Jan 5, 2024
 *      Author: catsa
 */
/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <stdarg.h>
#include "main.h"

#include "cmsis_os.h"
#include "m_normal.h"
#include "m_serial.h"
#include "m_ether.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define MAX_RECV_LEN			512
#define MAX_FRONT_SEND_LEN		8
#define MAX_PC_SEND_LEN			24

#define EXT_UART			(&huart1)
#define FRONT_UART			(&huart2)

enum{
	eFRONT_UART,
	eEXT_UART
};
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t Ext_rx_buf[1];
static uint8_t Front_rx_buf[1];

static uint8_t recv_buf[MAX_RECV_LEN]={0};
static uint8_t extRxData[24]={0,};
static uint8_t frontRxData[8]={0,};

static int ext_in = 0, ext_out = 0;
static int front_in = 0, front_out = 0;

osThreadId serialTaskHandle;

/* Private function prototypes -----------------------------------------------*/
void push_ext_buf(uint8_t ch)
{
	int index = (ext_in +1) % MAX_RECV_LEN;

	if(index == ext_out) return ;

	ext_in = index;
	recv_buf[ext_in] = ch;
}

void push_front_buf(uint8_t ch)
{
	int index = (front_in +1) % MAX_RECV_LEN;

	if(index == front_out) return ;

	front_in = index;
	recv_buf[front_in] = ch;
}

uint8_t pop_ext_buf(uint8_t *ch)
{
	if(ext_in == ext_out) return 0;
	ext_out = (ext_out + 1) % MAX_RECV_LEN;

	*ch =  recv_buf[ext_out];

	return 1;
}

uint8_t pop_front_buf(uint8_t *ch)
{
	if(front_in == front_out) return 0;
	front_out = (front_out + 1) % MAX_RECV_LEN;

	*ch =  recv_buf[front_out];

	return 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Console */
	if ( huart->Instance == USART1 ){
		push_ext_buf(Ext_rx_buf[0]);
		HAL_UART_Receive_IT(EXT_UART, &Ext_rx_buf[0], 1 );
	}else if ( huart->Instance == USART2 ){
		push_front_buf(Front_rx_buf[0]);
		HAL_UART_Receive_IT(FRONT_UART , &Front_rx_buf[0], 1 );
	}
}

int send_serial_data(uint8_t dest, uint8_t *send_data, uint32_t len)
{
	UART_HandleTypeDef *huart = FRONT_UART;
	int error;

	if(dest == eEXT_UART)
		huart = EXT_UART;
	error  = HAL_UART_Transmit(huart, send_data, len, 1000);
	if (error != HAL_OK){
		LOG_ERR("Modem send Cmd Error!![%x] \r\n", error);
		return -EIO; // Error
	}

	return HAL_OK;
}

void m_serial_SendFront(int protocol, int data)
{
	char nData[MAX_FRONT_SEND_LEN+1];

	sprintf(nData, "%c%07d", protocol,data);

	send_serial_data(eFRONT_UART, (uint8_t *)nData, MAX_FRONT_SEND_LEN);
}

void m_serial_SendPC(uint8_t digit, void *data)
{
	char nData[MAX_PC_SEND_LEN]={0};
	int *ptr, send_len;
	ip_net_t *ipnet;

	if(digit == DIGIT_3){
		ptr = (int *)data;
		sprintf(nData, ":%xR%03d\r\n", Device.nDeviceNo, *ptr);
		send_len = strlen(nData);
	}else if(digit == DIGIT_1){
		ptr = (int *)data;
		sprintf(nData, ":%xR%d\r\n", Device.nDeviceNo, *ptr);
		send_len = strlen(nData);
	}else if(digit == RET_OK){
		ptr = (int *)data;
		if(*ptr){
			sprintf(nData, "ERROR\r\n");
			send_len = 7;
		}else{
			sprintf(nData, "OK\r\n");
			send_len = 4;
		}
	}else if(digit == RET_IP){
		nData[0] = 0xDF;
		ipnet = (ip_net_t *)data;
		memcpy(&nData[1], ipnet->ipaddr, 4);
		memcpy(&nData[5], &ipnet->port, 2);
		LOG_DBG("port[%d][%d][%d]",ipnet->port,nData[5],nData[6]);

		nData[7] = 0xFD;
		send_len = 8;
	}

	LOG_DBG("Send PC [%s] Len[%d]", nData, send_len);
	send_serial_data(eEXT_UART, (uint8_t *)nData, send_len);
}

void get_front_data(void)
{
	int i;
	uint8_t read_ch;

	while(pop_front_buf(&read_ch)){
		for(i = 0; i < 7; i++){
			frontRxData[i] = frontRxData[i+1];
		}
		frontRxData[i] = read_ch;
		//LOG_HEX_DUMP(frontRxData, 8, "Get F");
		m_normal_parse_received_data(UART_RECV_FRONT, frontRxData, 8);
	}
}

//void dump_serial_buffer(void)
//{
//	LOG_DBG("FI[%d] FO[%d]", front_in, front_out);
//	LOG_HEX_DUMP(recv_buf, 512, "Serial Buf");
//	while(1){
//		osDelay(1000);
//	}
//}

void get_ext_data(void)
{
	int i;
	uint8_t read_ch;

	while(pop_ext_buf(&read_ch)){
		if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_REMOTE){
			for(i =0; i < 7; i++){
				extRxData[i] = extRxData[i+1];
			}
			extRxData[i] = read_ch;
			//LOG_HEX_DUMP(extRxData, 8, "erxData");
			m_normal_parse_received_data(UART_RECV_EXT, extRxData, 8);
		}else if(Device.nDeviceCurrentStatus == LCD_STATUS_MODE_STROBE_REMOTE){
			for(int i = 0; i < 23; i++){
				extRxData[i] = extRxData[i + 1];
			}
			extRxData[i] = read_ch;
			m_normal_parse_received_data(UART_RECV_EXT, extRxData, 24);
		}
	}
}

static void SerialTask(void const * argument)
{
	while(1){
		get_ext_data();
		get_front_data();
		osDelay(2);
	}
}

void m_serial_init(void)
{
	LOG_INF("Serial Init");

	HAL_UART_Receive_IT(EXT_UART, &Ext_rx_buf[0], 1);
	HAL_UART_Receive_IT(FRONT_UART, &Front_rx_buf[0], 1);

	osThreadDef(serialTask, SerialTask, osPriorityNormal, 0, 1024);
	serialTaskHandle = osThreadCreate(osThread(serialTask), NULL);
}
