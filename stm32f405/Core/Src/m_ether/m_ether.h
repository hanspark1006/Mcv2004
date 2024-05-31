/*
 * m_ether.h
 *
 *  Created on: Feb 9, 2024
 *      Author: Robyn
 */

#ifndef SRC_M_ETHER_M_ETHER_H_
#define SRC_M_ETHER_M_ETHER_H_
/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct _ip_net_t
{
	uint8_t ipaddr[4];
	uint8_t submask[4];
	uint8_t gateway[4];
	uint16_t port;
}ip_net_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
void EthernetInit(void);
/* Private defines -----------------------------------------------------------*/
#define MYWWWPORT 80
#define ETH_BUFFER_SIZE 550

#ifdef __cplusplus
}
#endif

typedef enum{
	eEtherConneting,
	eEtherConnected,
	eEtherDisconnected
}eEthernetStatus;

#endif /* SRC_M_ETHER_M_ETHER_H_ */
