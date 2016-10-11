/* Includes ------------------------------------------------------------------*/
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/udp.h"
#include "main.h"
#include "memp.h"
#include "adc.h"
#include <stdio.h>
#include <string.h>


struct udp_pcb *client_pcb;
struct pbuf *pb;
struct ip_addr DestIPaddr;
uint16_t  DestPort=70;

extern uint16_t *ADC_buf_pnt;

#define UDP_ADC_PACKET_SIZE	1024

uint16_t adc_buf_offset=0;

void udp_client_init(void)
{
  client_pcb = udp_new();
  IP4_ADDR( &DestIPaddr, 192, 168, 109, 140 );
  pb = pbuf_alloc(PBUF_TRANSPORT,UDP_ADC_PACKET_SIZE, PBUF_REF);
  pb->len = pb->tot_len = UDP_ADC_PACKET_SIZE;

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
  if (client_pcb != NULL)
  {
	while(adc_buf_offset!=ADC_BUF_LEN)
	{
		pb->payload = ((uint8_t*)ADC_buf_pnt+adc_buf_offset);
		err=udp_sendto(client_pcb, pb,&DestIPaddr,DestPort);
		adc_buf_offset+=UDP_ADC_PACKET_SIZE;
		delay(1000);
	}
  }
}




