#include "main.h"


#include "stm32f4xx_spi.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dcmi.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "string.h"


#define RX_BUFF_SIZE	ADC_BUF_LEN
__IO uint8_t DCMIAdcRxBuff[RX_BUFF_SIZE];


uint8_t *ADC_buf_pnt;
uint16_t ADC_last_data[ADC_CHN_NUM];
uint64_t timestamp=0;

SemaphoreHandle_t xAdcBuf_Send_Semaphore=NULL;
QueueHandle_t xADC_MB_Queue;


void ADC_DCMI_Tim_Init(void);
void ADC_DCMI_Core_Init(void);
void Timestamp_Init(void);

void ADC_Ext_Init(void)
{
	vSemaphoreCreateBinary( xAdcBuf_Send_Semaphore );
	xADC_MB_Queue = xQueueCreate( 1, sizeof(uint16_t)*ADC_CHN_NUM);
	Timestamp_Init();
	ADC_DCMI_Core_Init();
	ADC_DCMI_Tim_Init();
}



void Timestamp_Init(void)
{
	  TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;

	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);


      TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
      TIM_TimeBaseStructure.TIM_Prescaler =(SystemCoreClock>>1) / 100000 - 1;//10us
      TIM_TimeBaseStructure.TIM_ClockDivision = 0;
      TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
      TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

      TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
      TIM_TimeBaseStructure.TIM_Prescaler = 0;
      TIM_TimeBaseStructure.TIM_ClockDivision = 0;
      TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
      TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);


	  TIM_SelectInputTrigger(TIM5, TIM_TS_ITR2);

	  TIM_SelectMasterSlaveMode(TIM5, TIM_MasterSlaveMode_Enable);
	  TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_External1);

	  TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);


	  TIM_SetCounter(TIM4, 0);
	  TIM_SetCounter(TIM5, 0);

	  TIM_Cmd(TIM4, ENABLE);
	  TIM_Cmd(TIM5, ENABLE);
}

uint64_t ADC_GetLastTimestamp(void)
{
	return timestamp;
}

void ADC_GetLastVal(void)
{
	uint8_t temp[16];
	uint16_t result[8]={0,0,0,0,0,0,0,0};

	uint8_t bit_count=0, channel_count=0;

	memcpy(temp,&ADC_buf_pnt[(RX_BUFF_SIZE>>1)-16],16);

	for(bit_count=0;bit_count<16;bit_count++)
	{
		for(channel_count=0;channel_count<ADC_CHN_NUM;channel_count++)
		{
			result[channel_count]|=temp[]
		}
	}

	xQueueSend( xADC_MB_Queue, ( void * ) &result, ( TickType_t ) 0 );
}

void ADC_DCMI_Tim_Init(void)
{
	TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef           TIM_OCInitStructure;
	GPIO_InitTypeDef 			GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

	 //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE, ENABLE);


	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO );


	  RCC_MCO1Config(RCC_MCO1Source_PLLCLK,RCC_MCO1Div_4);
	  //RCC_MCO1Config(RCC_MCO1Source_HSE,RCC_MCO1Div_1);


	 //-------------ETR_CONFIG-------------------
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_TIM3);
	  //------------------------------------------
	 GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 ;
	 GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	 GPIO_Init(GPIOB, &GPIO_InitStructure);
	 GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);


	 /* Time base configuration */
	 TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	 TIM_TimeBaseStructure.TIM_Prescaler =  1-1;
	 TIM_TimeBaseStructure.TIM_Period = 420;//1680;
	 TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	 TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_Inverted, 0);

	 /* PWM1 Mode configuration: Channel1 */
	 TIM_OCStructInit(&TIM_OCInitStructure);
	 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	 TIM_OCInitStructure.TIM_Pulse = 16;//1350;//1515;
	 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	 TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;



	 TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	 TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

//	 TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	 TIM_ARRPreloadConfig(TIM3, ENABLE);

     GPIO_SetBits(GPIOA,GPIO_Pin_3);//start dcmi capture

	 TIM_Cmd(TIM3, ENABLE);
	 TIM_CtrlPWMOutputs(TIM3, ENABLE);


	 //-------------------------------------

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void ADC_DCMI_Core_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  DMA_InitTypeDef DMA_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	  DCMI_InitTypeDef DCMI_InitStructure;

	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE, ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);

	  //---------------------------------------------------

	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI );
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI );

	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI );
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI );

	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI );
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI );
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_DCMI );
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_DCMI );

	  GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI );
	  GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI );
	  GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI );

	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;


	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_6;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	  GPIO_Init(GPIOE, &GPIO_InitStructure);

		DCMI_DeInit();
		DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_Continuous;
		DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
		DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
		DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
		DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
		DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_Low;
		DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
		DCMI_Init(&DCMI_InitStructure);
		DCMI_CaptureCmd(ENABLE);
		DCMI_JPEGCmd (ENABLE);
		DCMI_Cmd(ENABLE);

	  //--------------DMA RX---------------------

	    DMA_InitStructure.DMA_BufferSize =RX_BUFF_SIZE/4;
	    DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	    DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)DCMIAdcRxBuff;
	    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&DCMI->DR);
	    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	    DMA_InitStructure.DMA_Mode =  DMA_Mode_Circular;
	    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	    DMA_Init(DMA2_Stream1, &DMA_InitStructure);

	    /* Inialize NVIC for the DMA interrupt */
	    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&NVIC_InitStructure);

	    DMA_ITConfig(DMA2_Stream1, DMA_IT_TC | DMA_IT_HT, ENABLE);

	    DMA_Cmd(DMA2_Stream1, ENABLE);
}

void DMA2_Stream1_IRQHandler(void)
{

	 static BaseType_t xHigherPriorityTaskWoken;


	if (DMA_GetITStatus(DMA2_Stream1,DMA_IT_HTIF1))
	{
		DMA_ClearITPendingBit ( DMA2_Stream1, DMA_IT_HTIF1);
		ADC_buf_pnt=&DCMIAdcRxBuff[0];
		xSemaphoreGiveFromISR( xAdcBuf_Send_Semaphore, &xHigherPriorityTaskWoken);
		GPIO_ToggleBits(GPIOE,GPIO_Pin_5);
	}
	else if (DMA_GetITStatus(DMA2_Stream1,DMA_IT_TCIF1))
	{
		DMA_ClearITPendingBit ( DMA2_Stream1, DMA_IT_TCIF1);
		ADC_buf_pnt=&DCMIAdcRxBuff[ADC_BUF_LEN>>1];
		xSemaphoreGiveFromISR( xAdcBuf_Send_Semaphore, &xHigherPriorityTaskWoken);

	}

	timestamp=((((uint64_t)(TIM5->CNT))<<16)|TIM4->CNT);

	if( xHigherPriorityTaskWoken != pdFALSE )
	{
		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	}
}




