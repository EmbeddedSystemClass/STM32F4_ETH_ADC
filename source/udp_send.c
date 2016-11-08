/* Includes ------------------------------------------------------------------*/
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/udp.h"
#include "main.h"
#include "memp.h"
#include "adc.h"
#include <stdio.h>
#include <string.h>
#include "adc_dcmi.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


#define UDP_ADC_PACKET_SIZE	1024
#define UDP_PACKET_SEND_DELAY 1000

#define SERVER_IP_ADDR0   192
#define SERVER_IP_ADDR1   168
#define SERVER_IP_ADDR2   109
#define SERVER_IP_ADDR3   140

#define SERVER_PORT		  1000

struct udp_pcb *client_pcb;
struct pbuf *pb;
struct ip_addr DestIPaddr;

extern uint16_t *ADC_buf_pnt;

uint16_t adc_buf_offset=0;

extern SemaphoreHandle_t xAdcBuf_Send_Semaphore;

void UDP_Send_Task( void *pvParameters );

#pragma pack(push,1)
typedef struct
{
	uint8_t 	id;
	uint64_t 	timestamp;
	uint8_t 	data[UDP_ADC_PACKET_SIZE];
}stPacket;
#pragma pack(pop)

stPacket UDPPacket;

void udp_client_init(void)
{
  client_pcb = udp_new();
  IP4_ADDR( &DestIPaddr, SERVER_IP_ADDR0, SERVER_IP_ADDR1, SERVER_IP_ADDR2, SERVER_IP_ADDR3 );
  pb = pbuf_alloc(PBUF_TRANSPORT,sizeof(UDPPacket), PBUF_REF);
  pb->len = pb->tot_len = sizeof(UDPPacket);
  pb->payload = (uint8_t*)&UDPPacket;


  xTaskCreate( UDP_Send_Task, "UDP Task", 512, NULL, 2, NULL );
}

inline void delay(uint32_t time)
{
	while (time)
		time--;
}

void udp_client_send_buf(void)
{
  err_t err;
  adc_buf_offset=0;
  UDPPacket.id=0;
 // UDPPacket.timestamp=Timestamp_GetLastTimestamp();
  if (client_pcb != NULL)
  {
	while(adc_buf_offset!=(ADC_BUF_LEN>>1))
	{
		memcpy(&UDPPacket.data,((uint8_t*)ADC_buf_pnt+adc_buf_offset),UDP_ADC_PACKET_SIZE);
		err=udp_sendto(client_pcb, pb,&DestIPaddr,SERVER_PORT);
		adc_buf_offset+=UDP_ADC_PACKET_SIZE;
		UDPPacket.id++;
		delay(UDP_PACKET_SEND_DELAY);//???
	}
  }
}

void UDP_Send_Task( void *pvParameters )
{
	while(1)
	{
		xSemaphoreTake( xAdcBuf_Send_Semaphore, portMAX_DELAY );
		udp_client_send_buf();
	}
}



