#include "tcp_send.h"
#include <stdio.h>
#include <string.h>
#include "err.h"
#include "tcp.h"

static struct tcp_pcb *connected_pcb = NULL;

#define SEND_BUFSIZE (256)
#define TCP_SND_BUFFER 8192

static char send_buf[SEND_BUFSIZE];


err_t sent_callback(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
 return ERR_OK;
}

err_t connected_callback(void *arg, struct tcp_pcb *tpcb, err_t err)
{

	/* store state */
	//connected_pcb = tpcb;

	/* set callback values & functions */
 	tcp_arg(tpcb, NULL);
 	tcp_sent(tpcb, sent_callback);

	/* initiate data transfer */
	return ERR_OK;
}

int8_t TCP_Send_Init(void)
{
 	struct tcp_pcb *pcb;
 	struct ip_addr ipaddr;
 	err_t err = ERR_CONN;
 	u16_t port;
 	int i;

	/* create new TCP PCB structure */
 	pcb = tcp_new();

	if (!pcb)
	{
  		return -1;
 	}

	/* connect to iperf server */
 	IP4_ADDR(&ipaddr,  192, 168, 109, 140);  /* iperf server address */
 	port = 10000;     /* iperf server port */
 	err = tcp_connect(pcb, &ipaddr, port, connected_callback);
	if (err != ERR_OK)
	{
  		return err;
 	}

	connected_pcb = pcb;

	/* initialize data buffer being sent */
	for (i = 0; i < SEND_BUFSIZE; i++)
	{
		send_buf[i] = (i % 10) + '0';
 	}
	return 0;
}



int8_t TCP_Send_Buf(void)
{
	int copy = 1;
 	err_t err;
 	struct tcp_pcb *tpcb = connected_pcb;
   	static packet_cnt = 0;
 	int buflen;

	 if (!connected_pcb)
 	 	return ERR_OK;

	if ((buflen = tcp_sndbuf(tpcb)) > SEND_BUFSIZE)
	{
  		packet_cnt++;
  		err = tcp_write(tpcb, send_buf, SEND_BUFSIZE, copy);

		if (err != ERR_OK)
		{
   			return -1;
  		}
 	}

 	if ((buflen = tcp_sndbuf(tpcb)) >= (TCP_SND_BUFFER - 3 * SEND_BUFSIZE))
	{
  		packet_cnt++;

  		err = tcp_write(tpcb, send_buf, SEND_BUFSIZE, copy);

		if (err != ERR_OK)
		{
   			return -1;
  		}
 	}

 	if ((buflen = tcp_sndbuf(tpcb)) >= (TCP_SND_BUFFER - 3 * SEND_BUFSIZE))
	{
  		packet_cnt++;
  		err = tcp_write(tpcb, send_buf, SEND_BUFSIZE, copy);

		if (err != ERR_OK)
		{
   			return -1;
  		}
 	}

	return 0;
}

