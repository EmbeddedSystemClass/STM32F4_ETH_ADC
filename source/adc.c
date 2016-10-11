#include "adc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"



uint16_t ADC_Buf[ADC_BUF_LEN];

uint16_t *ADC_buf_pnt;
uint8_t  ADC_buf_full_flag=0;

uint8_t wait_for_mb_flag=0;


void ADC1_Init(void)
{
       NVIC_InitTypeDef  		NVIC_InitStructure;
       DMA_InitTypeDef 			DMA_InitStructure;
       ADC_InitTypeDef 			ADC_InitStructure;
       ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
       TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
       GPIO_InitTypeDef  		GPIO_InitStructure;


       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);

       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3;
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
       GPIO_Init(GPIOC, &GPIO_InitStructure);

       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
       GPIO_Init(GPIOA, &GPIO_InitStructure);


       DMA_DeInit(DMA2_Stream0);
       DMA_InitStructure.DMA_Channel = DMA_Channel_0;
       DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
       DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Buf[0];
       DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
       DMA_InitStructure.DMA_BufferSize = ADC_BUF_LEN;
       DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
       DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
       DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
       DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
       DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
       DMA_InitStructure.DMA_Priority = DMA_Priority_High;
       DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
       DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
       DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
       DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
       DMA_Init(DMA2_Stream0, &DMA_InitStructure);
       DMA_Cmd(DMA2_Stream0, ENABLE);

       /* Enable the DMA Stream IRQ Channel */
       NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
       NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
       NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
       NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
       NVIC_Init(&NVIC_InitStructure);

       DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_HTIF0 | DMA_FLAG_TCIF0);
       DMA_ITConfig(DMA2_Stream0, DMA_IT_HT | DMA_IT_TC, ENABLE);


       	// базовая нстйрока
       TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
       TIM_TimeBaseStructure.TIM_Period = 105-1;//(84000000 / 200000) - 1; ;
       TIM_TimeBaseStructure.TIM_Prescaler = 0;
       TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
       TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
       TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
       	// выход синхронизации
       TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
       	// запуск таймера
       TIM_Cmd(TIM2, ENABLE);


       // общие параметры работы АЦП
       ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
       ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
       ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
       ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
       ADC_CommonInit(&ADC_CommonInitStructure);

       ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
       ADC_InitStructure.ADC_ScanConvMode = DISABLE;
       ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
       ADC_InitStructure.ADC_ExternalTrigConvEdge =  ADC_ExternalTrigConvEdge_Rising;
       ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
       ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
       ADC_InitStructure.ADC_NbrOfConversion = 1;
       ADC_Init(ADC1, &ADC_InitStructure);



		  //Порядок оцифровки

		#define ADC_SampleTime ADC_SampleTime_3Cycles
		ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, ADC_SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 3, ADC_SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_0 , 4, ADC_SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_3 , 5, ADC_SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_4 , 6, ADC_SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_5 , 7, ADC_SampleTime);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_6 , 8, ADC_SampleTime);

		ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

		ADC_DMACmd(ADC1, ENABLE); //Enable ADC1 DMA
		ADC_Cmd(ADC1, ENABLE); //Enable ADC1

}

void  DMA2_Stream0_IRQHandler (void)
{
    if (DMA_GetITStatus(DMA2_Stream0,DMA_IT_HTIF0))
    {
    	DMA_ClearITPendingBit ( DMA2_Stream0, DMA_IT_HTIF0);
    	ADC_buf_pnt=&ADC_Buf[0];
    	ADC_buf_full_flag=1;
    }

    if (DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0))
    {
    	DMA_ClearITPendingBit ( DMA2_Stream0, DMA_IT_TCIF0);
    	ADC_buf_pnt=&ADC_Buf[ADC_BUF_LEN>>1];
    	ADC_buf_full_flag=1;
    }

}

