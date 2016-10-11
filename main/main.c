/* Includes ------------------------------------------------------------------*/
#include "stm32f4x7_eth.h"
#include "netconf.h"
#include "main.h"
#include "misc.h"
#include "adc.h"
//#include "tcp_send.h"
#include "udp_send.h"
#include "mb.h"
#include "mbtcp.h"

#define SYSTEMTICK_PERIOD_MS  10
static uint8_t Vendor[] = "GEOS";


volatile uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;


extern uint8_t  ADC_buf_full_flag;


int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  ADC1_Init();
  ETH_BSP_Config();
  LwIP_Init();
  udp_client_init();

  eMBTCPInit(0);
  eMBEnable();
  eMBSetSlaveID( MB_TCP_PSEUDO_ADDRESS, TRUE, Vendor, sizeof(Vendor) );

  while (1)
  {  
	eMBPoll();

    if (ETH_CheckFrameReceived())
    { 
      LwIP_Pkt_Handle();
    }

    LwIP_Periodic_Handle(LocalTime);

    if(ADC_buf_full_flag)
    {
    	udp_client_send_buf();
    	ADC_buf_full_flag=0;
    }
  } 
}


void Delay(uint32_t nCount)
{
  timingdelay = LocalTime + nCount;  

  while(timingdelay > LocalTime)
  {     
  }
}

void Time_Update(void)
{
  LocalTime += SYSTEMTICK_PERIOD_MS;
}


