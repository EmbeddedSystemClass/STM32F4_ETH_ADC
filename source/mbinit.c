#include "mbinit.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mb.h"
#include "mbtcp.h"

#define MB_TCP_TASK_STACK_SIZE	1280
static uint8_t Vendor[] = "GEOS";

void MB_TCP_Task(void *pvParameters);

void MB_TCP_Init(void)
{
	eMBTCPInit(0);
	eMBEnable();
	eMBSetSlaveID( MB_TCP_PSEUDO_ADDRESS, TRUE, Vendor, sizeof(Vendor) );
	xTaskCreate(MB_TCP_Task, "MB TCP Task", MB_TCP_TASK_STACK_SIZE, NULL, 2, ( TaskHandle_t * ) NULL);
}


void MB_TCP_Task(void *pvParameters)
{
   while(1)
    {
	   eMBPoll();
        //vTaskDelay(1);
    }
}
