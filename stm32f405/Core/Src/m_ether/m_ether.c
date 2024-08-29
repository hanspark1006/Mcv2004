/*
 * m_ether.c
 *
 *  Created on: Feb 9, 2024
 *      Author: Robyn
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

#include "m_env.h"
#include "m_ether.h"
#include "m_serial.h"
#include "Ethernet.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct _mac_ip_addr
{
	char id[2];
	ip_net_t ip_net;
}mac_ip_addr;

typedef enum{
	eREPLY_OK,
	eREPLY_IP
}eReply_t;
/* Private define ------------------------------------------------------------*/
#define  DEC2BCD(v) (((v/10)<<4) + (v%10));
#define  BCD2DEC(v) ((v>>4)*10 + (v&0x0F));

#define AT24EEP_ADDR		0xA0
#define AT24MAC_ADDR		0xB0
#define EUI48_ADDR			0x9A
#define TCP_DATA_PAYLOAD	0x36

#define MAC_I2C_HANDLE	hi2c1

#define _SET_DEFAULT_IP_TEST	0
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osThreadId ethernetTaskHandle;

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

#ifdef _DHCP_
uint8_t dhcpsvrip[4] = { 168, 124, 101, 2 };
uint8_t dnsip[4] = { 0, 0, 0, 0 };
#endif

static uint8_t mymac[6] = { 0x54, 0x57, 0x51, 0x10, 0x05, 0x25 };
mac_ip_addr g_ip_net;

uint8_t eth_buf[ETH_BUFFER_SIZE+1];
uint8_t recv_buf[50];
int send_reply_len;
uint8_t need_reply = 0;
static event_queue_observer_t ether_event;

static struct {
	uint8_t enable;
}m_cfg = {.enable = 0};

/* Private function prototypes -----------------------------------------------*/
void EthernetTask(void const * argument);

static int read_mac_addr(void)
{
	uint8_t mac_addr[7];

	if(m_env_e2p_read(&MAC_I2C_HANDLE, AT24MAC_ADDR, EUI48_ADDR, 1, mac_addr, 6))
	{
		LOG_ERR("Get MAC Error");
		return 1;
	}

	memcpy(mymac, &mac_addr[1],6);

	LOG_DBG("mac = %x:%x:%x:%x:%x:%x", mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], mac_addr[6]);

	return 0;
}
#if 0
static int make_send_data(uint8_t *pPayLoad, eReply_t type, int error)
#else
static int make_send_data(uint8_t *pPayLoad)
#endif
{
	int offset = 0;
#if 0
	if(type == eREPLY_OK){
		if(error){
			sprintf((char *)&pPayLoad[offset], "ERROR\r\n");
			offset = 7;
		}else{
			sprintf((char *)&pPayLoad[offset], "OK\r\n");
			offset = 4;
		}
	}else if(type == eREPLY_IP){
		pPayLoad[offset] = 0xDF;
		offset+=1;

		memcpy(&pPayLoad[offset], g_ip_net.ip_net.ipaddr, 4);
		offset+=4;
		memcpy(&pPayLoad[offset], &g_ip_net.ip_net.port, 2);
		offset+=2;
		pPayLoad[offset] = 0xFD;
	}
#else
	pPayLoad[offset++] = 0x00;
	pPayLoad[offset++] = 0xFF;
	pPayLoad[offset++] = 0xFE;
#endif

	return offset;
}

#if 0
static void send_reply_data(eReply_t reply_type, int error)
{
	uint16_t offset;

	offset =  make_send_data(&eth_buf[TCP_DATA_PAYLOAD], reply_type, error);
	if(offset > 0){
//			LOG_HEX_DUMP(eth_buf,  dat_p+offset, "Send Data >>");
	}
	else{
		offset = send_reply_len;
	}
	LOG_DBG("Send Tcp Reply Data Len[%d]", offset);
	Ethernet_wb_server_reply(eth_buf, offset);
}
#endif

static int read_packet(void)
{
	int tmp_cnt;
	uint16_t dat_p;
	uint16_t offset;

	tmp_cnt = Ethernet_PacketReceive(ETH_BUFFER_SIZE, eth_buf);
	dat_p = Ethernet_packetloop_icmp_tcp(eth_buf, tmp_cnt);
	if (dat_p > 0){
//		LOG_HEX_DUMP(eth_buf, tmp_cnt, "Received Data >>");

		memcpy(recv_buf, &eth_buf[TCP_DATA_PAYLOAD],tmp_cnt - dat_p);
		push_event0_param(EVT_received_tcp, recv_buf, tmp_cnt - dat_p);
		offset = make_send_data(&eth_buf[TCP_DATA_PAYLOAD]);
		if(offset > 0){

		}else{
			offset = tmp_cnt - dat_p;
		}

		Ethernet_wb_server_reply(eth_buf, offset);
		need_reply = 1;
	}

	return 0;
}

static int ethernet_init(void)
{
	Ethernet_Init(mymac);

	if (Ethernet_Revision() <= 0){
		// Failed to access ENC28J60
		//while (1);    // Just loop here
		LOG_ERR("Ethernet_Revision Error!!");
		return 1;
	}

	LOG_INF("Ethernet_Revision : %d", Ethernet_Revision());
	LOG_INF("mymac : %d:%d:%d:%d:%d:%d", mymac[0], mymac[1], mymac[2], mymac[3], mymac[4], mymac[5] );

#ifdef _DHCP_
	// Get IP Address details
	if (Ethernet_allocate_ip_address(eth_buf, ETH_BUFFER_SIZE, mymac, 80, g_ip_net.ip_net.ipaddr, g_ip_net.ip_net.submask, g_ip_net.ip_net.gateway, dhcpsvrip, dnsip) > 0){
		// Display the results:
		LOG_DBG("Success IP Resource binding");
	}else{
		// Failed to get IP address"
		LOG_ERR("Failed to get IP address");
		return 1;
	}
#else
	Ethernet_init_ip_arp_udp_tcp(mymac, g_ip_net.ip_net.ipaddr, g_ip_net.ip_net.port);
#if 0 // Todo : need to Check
	Ethernet_client_set_gwip(g_ip_net.ip_net.gateway);
#endif
#endif

	LOG_DBG("myip = %d.%d.%d.%d \r\n", g_ip_net.ip_net.ipaddr[0],g_ip_net.ip_net.ipaddr[1],
									g_ip_net.ip_net.ipaddr[2],g_ip_net.ip_net.ipaddr[3]);

	return 0;
}

void ENC28J60_DelayUs(uint32_t delay) {
	do {
		asm volatile (	"MOV R0,%[loops]\n\t"\
				"1: \n\t"\
				"SUB R0, #1\n\t"\
				"CMP R0, #0\n\t"\
				"BNE 1b \n\t" : : [loops] "r" (20*delay) : "memory"\
			      ); // test logic analyzer, target 100us : 20->97us 21->127us
	} while(0);
}

void ENC28J60_DelayMs(uint32_t delay) {
	osDelay(delay);
}

uint32_t ENC28J60_GetMs(void) {
	return HAL_GetTick();
}

void ENC28J60_EnableChip(void) {
	HAL_GPIO_WritePin(ETH_NSS_GPIO_Port, ETH_NSS_Pin, GPIO_PIN_RESET);
}

void ENC28J60_DisableChip(void) {
	HAL_GPIO_WritePin(ETH_NSS_GPIO_Port, ETH_NSS_Pin, GPIO_PIN_SET);
}

uint8_t ENC28J60_TransceiveByte(uint8_t data) {
	uint8_t received;
	if (HAL_SPI_TransmitReceive(&hspi1, &data, &received, 1, 1000) == HAL_OK) {
		return received;
	}
	return 0;
}

int m_eth_write_mac_ipaddr(ip_net_t *ip_addr)
{
	uint8_t buffer[32]={0,}, size;
	uint16_t mem_address;

	mem_address = 0;
	size = sizeof(g_ip_net);
	g_ip_net.id[0] = 'Y';
	g_ip_net.id[1] = 'S';
	memcpy(&g_ip_net.ip_net, ip_addr, sizeof(ip_net_t));
	memcpy(buffer, &g_ip_net, sizeof(mac_ip_addr));
	LOG_HEX_DUMP(buffer, size, "Write E2p");
	if(m_env_e2p_write(&MAC_I2C_HANDLE, AT24EEP_ADDR, mem_address, 1, buffer, size)){
		LOG_ERR("Set IP Address from MacIC Error");
		return 1;
	}

	Ethernet_init_ip_arp_udp_tcp(mymac, g_ip_net.ip_net.ipaddr, g_ip_net.ip_net.port);
	return 0;
}

static int read_ipnet(void)
{
	uint8_t buffer[32]={0,}, size;
	uint16_t mem_address;

	mem_address = 0;
	size = sizeof(mac_ip_addr);
	if(m_env_e2p_read(&MAC_I2C_HANDLE, AT24EEP_ADDR, mem_address, 1, buffer, size)){
		LOG_ERR("Get IP Address from MacIC Error\r\n");
		return 1;
	}
	LOG_HEX_DUMP(buffer, size, "Read E2p");
	memcpy(&g_ip_net, buffer, sizeof(mac_ip_addr));
	if(memcmp(g_ip_net.id, "YS", 2)==0){
		LOG_INF("Read Size [%d] IP Addr [%03d.%03d.%03d.%03d] Port[%d]\r\n", size, g_ip_net.ip_net.ipaddr[0], g_ip_net.ip_net.ipaddr[1],
							g_ip_net.ip_net.ipaddr[2], g_ip_net.ip_net.ipaddr[3], g_ip_net.ip_net.port);
#if _SET_DEFAULT_IP_TEST
		return 1;
#endif
		return 0;
	}else{
		LOG_ERR("Not found IP Address from MacIC!!\r\n");
		return 1;
	}

	return 0;
}

static void onWriteIPAddress(void *pData, uint32_t size)
{
	ip_net_t ipnet;
	int error;
	char *Data = pData;

	ipnet.submask[0] = 255;
	ipnet.submask[1] = 255;
	ipnet.submask[2] = 255;
	ipnet.submask[3] = 255;

	ipnet.gateway[0] = Data[0];
	ipnet.gateway[1] = Data[1];
	ipnet.gateway[2] = 0;
	ipnet.gateway[3] = 1;

	ipnet.ipaddr[0] = Data[0];
	ipnet.ipaddr[1] = Data[1];
	ipnet.ipaddr[2] = Data[2];
	ipnet.ipaddr[3] = Data[3];

	ipnet.port = (Data[4]<<8 | Data[5]);
	LOG_DBG("Port[%d][%d] inet[%d]", Data[4], Data[5], ipnet.port);


	error = m_eth_write_mac_ipaddr(&ipnet);

	m_serial_SendPC(RET_OK, &error);
#if 0
	if(need_reply){
		need_reply = 0;
		send_reply_data(eREPLY_OK, error);
	}
#endif
}

static void onReadIPAddress(void)
{
	int error;

	error = read_ipnet();

	if(error){
		m_serial_SendPC(RET_OK, &error);
	}else{
		m_serial_SendPC(RET_IP, &g_ip_net.ip_net);
	}
#if 0
	if(need_reply){
		if(error){
			send_reply_data(eREPLY_OK, error);
		}else{
			send_reply_data(eREPLY_IP, 0);
		}
	}
#endif
}

int m_eth_read_mac_ipaddr(ip_net_t *ip_addr)
{
	memcpy(ip_addr, &g_ip_net.ip_net, sizeof(ip_net_t));

	return 0;
}

void onEthernetFuncEnable(uint8_t enable)
{
	m_cfg.enable = enable;
}

void EthernetTask(void const * argument)
{
	while(1){
		if(m_cfg.enable){
			read_packet();
		}
		osDelay(10);
	}
}

static void evt_handler(event_t const* evt, void* p_context)
{
	switch (evt->event)
	{
		case EVT_Set_ip:
			onWriteIPAddress(evt->p_event_data, evt->event_data_size);
			break;
		case EVT_Get_ip:
			onReadIPAddress();
			break;
		case EVT_Disable_ip:
			break;
		case EVT_eth_enable:
			onEthernetFuncEnable(evt->param1);
			break;
		default:
			break;
	}
}

void EthernetInit(void)
{
	int time_out = 0;
	ip_net_t default_ip;

	osThreadDef(ethernetTask, EthernetTask, osPriorityNormal, 0, 128);
	ethernetTaskHandle = osThreadCreate(osThread(ethernetTask), NULL);

	while(time_out++ < 10){
		if(read_mac_addr() == 0)
			break;
		HAL_Delay(10);
	}

	if(read_ipnet()){
		// default ip address
		default_ip.ipaddr[0] = 192;
		default_ip.ipaddr[1] = 168;
		default_ip.ipaddr[2] = 0;
		default_ip.ipaddr[3] = 5;
		default_ip.port = 5050;

		default_ip.gateway[0] = 192;
		default_ip.gateway[1] = 168;
		default_ip.gateway[2] = 0;
		default_ip.gateway[3] = 1;

		LOG_INF("Set Default IP Addr [%03d.%03d.%03d.%03d] Port[%d]\r\n", default_ip.ipaddr[0], default_ip.ipaddr[1],
										default_ip.ipaddr[2], g_ip_net.ip_net.ipaddr[3], default_ip.port);
		m_eth_write_mac_ipaddr(&default_ip);
	}

	if(ethernet_init())
		return ;

	ether_event.handler = evt_handler;
	ether_event.p_context = NULL;
	if(m_event_register(&ether_event)){
		return ;
	}
}
