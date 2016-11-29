#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include <stdio.h>
#include <string.h>
#include "http_server.h"

#define LEN 1024
char data[LEN];

const char webif_200_header[] =
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: text/html; charset=windows-1251\r\n"
    "Server: ATmega16\r\n"
    "\r\n";

const char webif_form[] =
		"<pre>"
		"<form action='/' method='GET'>\r\n"
		"Enter device IP address :\r\n"
		"<input type='text' name='ip0' size='4' value='192'>"
		"<input type='text' name='ip1' size='4' value='168'>"
		"<input type='text' name='ip2' size='4' value='109'>"
		"<input type='text' name='ip3' size='4' value='150'>"
		"<input type='submit' value='OK'>\r\n"
		"</form>"
		"</pre>";


void http_server_serve(struct netconn *conn);
void http_server_netconn_thread();


void http_server_init(void)
{
	 xTaskCreate(http_server_netconn_thread, "HTTP", 512, NULL, tskIDLE_PRIORITY,  NULL);
}

void http_server_serve(struct netconn *conn) 
{
    struct netbuf *inbuf;
    err_t res;
    char* buf;
    u16_t buflen;
    size_t len;
    static u16_t call_times = 0;
        
    res = netconn_recv(conn, &inbuf);
    
    if (res == ERR_OK)
    {
        netbuf_data(inbuf, (void**)&buf, &buflen);
        if ((buflen >=5) && (strncmp(buf, "GET /", 5) == 0))
        {
//            sprintf(data, "Hello %d times!", call_times++);
//            len = strlen(data);
//            netconn_write(conn, (const unsigned char*)(data), (size_t)len, NETCONN_NOCOPY);

            len = strlen(webif_200_header);
            netconn_write(conn, (const unsigned char*)(webif_200_header), (size_t)len, NETCONN_NOCOPY);

            len = strlen(webif_form);
            netconn_write(conn, (const unsigned char*)(webif_form), (size_t)len, NETCONN_NOCOPY);
        }
    }
    
    /* Close the connection (server closes in HTTP) */
    netconn_close(conn);

    /* Delete the buffer (netconn_recv gives us ownership,
    so we have to make sure to deallocate the buffer) */
    netbuf_delete(inbuf);
}

void http_server_netconn_thread()
{ 
  struct netconn *conn, *newconn;
  err_t err;
  
  /* Create a new TCP connection handle */
  conn = netconn_new(NETCONN_TCP);
  
  if (conn!= NULL)
  {
    /* Bind to port 80 (HTTP) with default IP address */
    err = netconn_bind(conn, NULL, 80);
    
    if (err == ERR_OK)
    {
      /* Put the connection into LISTEN state */
      netconn_listen(conn);
  
      while(1) 
      {
        /* accept any icoming connection */
        err = netconn_accept(conn, &newconn);

        /* serve connection */
        http_server_serve(newconn);

        /* delete connection */
        netconn_delete(newconn);
      }
    }
    else
    {
      printf("can not bind netconn");
    }
  }
  else
  {
    printf("can not create netconn");
  }
}

void http_param_handler(uint8_t *param_string)
{
	const uint8_t *str_param[]={"ip0","ip1","ip2","ip3"};
	uint8_t *str_pnt;
	uint8_t ip[4];
	uint8_t i=0;

	for(i=0;i<4;i++)
	{
		if(strcmp(param_string,str_param[i]))
		{
			if(str_pnt = strchr(param_string, '='))
			{
				str_pnt++;
				//atoi();
			}

		}
	}



}
