/* Includes ------------------------------------------------------------------*/
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "main.h"
#include "memp.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#if LWIP_TCP


struct tcp_pcb *client_pcb;

extern uint8_t buf_transmit_flag;


/* Private function prototypes -----------------------------------------------*/

static void tcp_client_connection_close(struct tcp_pcb *tpcb);
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_client_send(struct tcp_pcb *tpcb);
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);

/* Private functions ---------------------------------------------------------*/

void tcp_client_connect(void)
{
  struct ip_addr DestIPaddr;
  buf_transmit_flag=1;
  client_pcb = tcp_new();
  if (client_pcb != NULL)
  {
    IP4_ADDR( &DestIPaddr, 192, 168, 109, 140 );
    tcp_connect(client_pcb,&DestIPaddr,8080,tcp_client_connected);
  }
}


static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{

  if (err == ERR_OK)
  {
        /* initialize LwIP tcp_sent callback function */
        tcp_sent(tpcb, tcp_client_sent);

        /* send data */
        tcp_client_send(tpcb);


        return ERR_OK;
  }
  else
  {
    /* close connection */
	  	 tcp_client_connection_close(tpcb);
  }
  return err;
}


extern uint16_t *ADC_buf_pnt;
static void tcp_client_send(struct tcp_pcb *tpcb)
{
	err_t wr_err = ERR_OK;
//	((uint8_t*)ADC_buf_pnt+(cnt*1024))[0]=cnt;
//	wr_err = tcp_write(tpcb, (uint8_t*)ADC_buf_pnt+(cnt*1024),1024,0);
	wr_err = tcp_write(tpcb, (uint8_t*)ADC_buf_pnt,16384,0);
	tcp_output(tpcb);
}

extern uint8_t buf_transmit_flag;
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  LWIP_UNUSED_ARG(len);
//  cnt++;

//  if(cnt!=32)
//  {
//    tcp_client_send(tpcb);
//  }
//  else
//  {
	  tcp_client_connection_close(tpcb);
	  buf_transmit_flag=0;
//	  cnt=0;
//  }

  return ERR_OK;
}


static void tcp_client_connection_close(struct tcp_pcb *tpcb)
{
  tcp_sent(tpcb, NULL);
  tcp_close(tpcb);
}

#endif /* LWIP_TCP */


