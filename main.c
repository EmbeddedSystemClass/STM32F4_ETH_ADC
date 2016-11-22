#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4x7_eth_bsp.h"



#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/dhcp.h"
#include "ethernetif.h"
#include "main.h"
#include "tcpip.h"
#include "netif.h"
#include "http_server.h"
#include <stdio.h>
#include "mbinit.h"
#include "udp_send.h"
#include "tcp_send.h"
#include "adc_dcmi.h"
#include "cfg_info.h"

struct netif xnetif;
extern sConfigInfo configInfo;

void Init_Task( void *pvParameters );


void LwIP_Init(sIPAddress IPAddress)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;

  tcpip_init( NULL, NULL );	

#ifdef USE_DHCP
  ipaddr.addr = 0;
  netmask.addr = 0;
  gw.addr = 0;
#else
  IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
  //IP4_ADDR(&ipaddr, IPAddress.ip_addr_0, IPAddress.ip_addr_1, IPAddress.ip_addr_2, IPAddress.ip_addr_3);
  IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
  IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
#endif

  netif_add(&xnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
  netif_set_default(&xnetif);
  netif_set_up(&xnetif); 
}


void Init_Task( void *pvParameters )
{
	ConfigInfoRead();
    ETH_BSP_Config();

    /* Initilaize the LwIP stack */
    LwIP_Init(configInfo.IPAdress_Client);
    MB_TCP_Init();

    ADC_Ext_Init();
    udp_client_init();
	vTaskDelete(NULL);
}


int main()
{
	//  SystemInit();
	//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

//	ConfigInfoRead();
//    ETH_BSP_Config();
//
//    /* Initilaize the LwIP stack */
//    LwIP_Init(configInfo.IPAdress_Client);
//    MB_TCP_Init();
//
//    ADC_Ext_Init();
    //udp_client_init();

    xTaskCreate( Init_Task, "Init Task", 256, NULL, tskIDLE_PRIORITY, NULL );

    vTaskStartScheduler();
    return 0;
}
