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


u8_t   data[100];

__IO u8_t Tcp_flag = 0;

struct tcp_pcb *client_pcb;

uint8_t cnt=0;



/* Private function prototypes -----------------------------------------------*/

static void tcp_client_connection_close(struct tcp_pcb *tpcb);
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_client_send(struct tcp_pcb *tpcb);
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Connects to the TCP echo server
  * @param  None
  * @retval None
  */
void tcp_client_connect(void)
{
  struct ip_addr DestIPaddr;

  /* create new tcp pcb */
  client_pcb = tcp_new();

  if (client_pcb != NULL) {
;
    IP4_ADDR( &DestIPaddr, 192, 168, 109, 140 );

    /* connect to destination address/port */
    tcp_connect(client_pcb,&DestIPaddr,23,tcp_client_connected);
  }
}

/**
  * @brief Function called when TCP connection established
  * @param tpcb: pointer on the connection contol block
  * @param err: when connection correctly established err should be ERR_OK
  * @retval err_t: returned error
  */
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
  if (err == ERR_OK)
  {
        /* initialize LwIP tcp_sent callback function */
        tcp_sent(tpcb, tcp_client_sent);

        /* send data */
        tcp_client_send(tpcb);
        cnt=0;

        return ERR_OK;
  }
  else
  {
    /* close connection */
	  	 tcp_client_connection_close(tpcb);
  }
  return err;
}



/**
  * @brief function used to send data
  * @param  tpcb: tcp control block
  * @param  es: pointer on structure of type client containing info on data
  *             to be sent
  * @retval None
  */
extern uint16_t *ADC_buf_pnt;
static void tcp_client_send(struct tcp_pcb *tpcb)
{
	err_t wr_err = ERR_OK;
	wr_err = tcp_write(tpcb, (uint8_t*)ADC_buf_pnt+(cnt*1024),1024,0);
//  struct pbuf *ptr;
//  err_t wr_err = ERR_OK;
//
//  while ((wr_err == ERR_OK) &&
//         (es->p_tx != NULL) &&
//         (es->p_tx->len <= tcp_sndbuf(tpcb)))
//  {
//
//    /* get pointer on pbuf from es structure */
//    ptr = es->p_tx;
//
//    /* enqueue data for transmission */
//    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);
//
//    if (wr_err == ERR_OK) {
//      /* continue with next pbuf in chain (if any) */
//      es->p_tx = ptr->next;
//
//      if (es->p_tx != NULL) {
//        /* increment reference count for es->p */
//        pbuf_ref(es->p_tx);
//      }
//
//      /* free pbuf: will free pbufs up to es->p (because es->p has a reference count > 0) */
//      pbuf_free(ptr);
//   } else if(wr_err == ERR_MEM) {
//      /* we are low on memory, try later, defer to poll */
//     es->p_tx = ptr;
//   } else {
//     /* other problem ?? */
//   }
//  }
}


/**
  * @brief  This function implements the tcp_sent LwIP callback (called when ACK
  *         is received from remote host for sent data)
  * @param  arg: pointer on argument passed to callback
  * @param  tcp_pcb: tcp connection control block
  * @param  len: length of data sent
  * @retval err_t: returned error code
  */
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  LWIP_UNUSED_ARG(len);



  if(cnt!=(16*sizeof(uint16_t)))
  {
    /* still got pbufs to send */
    tcp_client_send(tpcb);
    cnt++;
  }
  else
  {
	  tcp_client_connection_close(tpcb);
  }

  return ERR_OK;
}

/**
  * @brief This function is used to close the tcp connection with server
  * @param tpcb: tcp connection control block
  * @param es: pointer on client structure
  * @retval None
  */
static void tcp_client_connection_close(struct tcp_pcb *tpcb)
{
  /* remove callbacks */
  tcp_recv(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_poll(tpcb, NULL,0);

  /* close tcp connection */
  tcp_close(tpcb);

}

#endif /* LWIP_TCP */


