#include "main.h"


#include "stm32f4xx_spi.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dcmi.h"

#define RX_BUFF_SIZE	ADC_BUF_LEN
//#define SPI1_TX_BUFF_SIZE	4
__IO uint8_t DCMIAdcRxBuff[RX_BUFF_SIZE];
//__IO uint16_t DCMIAdcTxBuff[TX_BUFF_SIZE];

uint16_t *ADC_buf_pnt;
uint8_t  ADC_buf_full_flag=0;
uint16_t ADC_last_data[ADC_CHN_NUM];

void ADC_DCMI_Tim_Init(void);
void ADC_DCMI_Core_Init(void);

void ADC_Ext_Init(void)
{
	ADC_DCMI_Tim_Init();
	ADC_DCMI_Core_Init();

}

void ADC_DCMI_Tim_Init(void)
{
	TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef           TIM_OCInitStructure;
	GPIO_InitTypeDef 			GPIO_InitStructure;

	 /*    TIM1 Configuration
	 */
	 //RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

	 //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE, ENABLE);

	 //-------------ETR_CONFIG-------------------
	  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  //GPIO_Init(GPIOE, &GPIO_InitStructure);
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  //GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_TIM1);
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_TIM3);

	  //------------------------------------------
	// GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 ;
	 GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 ;
	 GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//	 GPIO_Init(GPIOE, &GPIO_InitStructure);
//	 GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	 GPIO_Init(GPIOB, &GPIO_InitStructure);
	 GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);


	 /* Time base configuration */
	 TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	 TIM_TimeBaseStructure.TIM_Prescaler =  1-1;
	 TIM_TimeBaseStructure.TIM_Period = 420;//1680;
	 TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 //TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	 TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

//	 TIM_ETRClockMode2Config(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
	 TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_Inverted, 0);

	 /* PWM1 Mode configuration: Channel1 */
	 TIM_OCStructInit(&TIM_OCInitStructure);
	 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	 TIM_OCInitStructure.TIM_Pulse = 24;//1350;//1515;
	 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	 TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

//	 TIM_OC1Init(TIM1, &TIM_OCInitStructure);
//	 TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
//
//	 TIM_ARRPreloadConfig(TIM1, ENABLE);
//
//	 TIM_Cmd(TIM1, ENABLE);
//	 TIM_CtrlPWMOutputs(TIM1, ENABLE);

	 TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);


	 TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	 TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	 TIM_ARRPreloadConfig(TIM3, ENABLE);

	 TIM_Cmd(TIM3, ENABLE);
	 TIM_CtrlPWMOutputs(TIM3, ENABLE);


	 //-----------------TIM9----------------------

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOE, &GPIO_InitStructure);

	  GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);


	 /* Time base configuration */
	 TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	 TIM_TimeBaseStructure.TIM_Prescaler =  1-1;
	 TIM_TimeBaseStructure.TIM_Period = 1024;//1680;
	 TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);


	  TIM_SelectInputTrigger(TIM9, TIM_TS_ITR1);
	  TIM_SelectMasterSlaveMode(TIM9, TIM_MasterSlaveMode_Enable);
	  TIM_SelectSlaveMode(TIM9, TIM_SlaveMode_External1);


	 /* PWM1 Mode configuration: Channel1 */
	 TIM_OCStructInit(&TIM_OCInitStructure);
	 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	 TIM_OCInitStructure.TIM_Pulse = 1024;//1350;//1515;
	 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	 TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;



	 TIM_OC1Init(TIM9, &TIM_OCInitStructure);
	 TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

	 TIM_ARRPreloadConfig(TIM9, ENABLE);

	 TIM_Cmd(TIM9, ENABLE);
	 TIM_CtrlPWMOutputs(TIM9, ENABLE);

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



	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO );


		//RCC_MCO1Config(RCC_MCO1Source_PLLCLK,RCC_MCO1Div_4);
		RCC_MCO1Config(RCC_MCO1Source_HSE,RCC_MCO1Div_1);


	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4|GPIO_PinSource6, GPIO_AF_DCMI );
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6|GPIO_PinSource7, GPIO_AF_DCMI );
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6|GPIO_PinSource7|GPIO_PinSource8|GPIO_PinSource9, GPIO_AF_DCMI );
	  GPIO_PinAFConfig(GPIOE, GPIO_PinSource4|GPIO_PinSource5|GPIO_PinSource6, GPIO_AF_DCMI );

	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;


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
		DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_High;
		DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
		DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
		DCMI_Init(&DCMI_InitStructure);
		DCMI_CaptureCmd(ENABLE);
		DCMI_Cmd(ENABLE);


		///--------------]
	    // прерывание DCMI
	    DCMI_ITConfig(DCMI_IT_VSYNC, ENABLE);
	    DCMI_ITConfig(DCMI_IT_LINE, ENABLE);
	    DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
	    DCMI_ITConfig(DCMI_IT_OVF, ENABLE);
	    DCMI_ITConfig(DCMI_IT_ERR, ENABLE);

	    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	    NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&NVIC_InitStructure);
		///--------------]



	  //--------------DMA RX---------------------

	    DMA_InitStructure.DMA_BufferSize = RX_BUFF_SIZE;
	    DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	    DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)DCMIAdcRxBuff;
	    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&DCMI->DR);
	    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	    DMA_InitStructure.DMA_Mode =  DMA_Mode_Circular; //DMA_Mode_Normal;
	    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	    DMA_Init(DMA2_Stream1, &DMA_InitStructure);

	    /* Inialize NVIC for the DMA interrupt */
	    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&NVIC_InitStructure);

	    DMA_ITConfig(DMA2_Stream1, DMA_IT_TC | DMA_IT_HT, ENABLE);

	    DMA_Cmd(DMA2_Stream1, ENABLE);
}

void DMA2_Stream1_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA2_Stream1,DMA_IT_HTIF1))
	{
		DMA_ClearITPendingBit ( DMA2_Stream1, DMA_IT_HTIF1);
		ADC_buf_pnt=&DCMIAdcRxBuff[0];
		ADC_buf_full_flag=1;
	}

	if (DMA_GetITStatus(DMA2_Stream1,DMA_IT_TCIF1))
	{
		DMA_ClearITPendingBit ( DMA2_Stream1, DMA_IT_TCIF1);
		ADC_buf_pnt=&DCMIAdcRxBuff[ADC_BUF_LEN>>1];
		ADC_buf_full_flag=1;
	}
}

void DCMI_IRQHandler(void)
{
    if(DCMI_GetFlagStatus(DCMI_FLAG_VSYNCRI) == SET)
    {
    	DCMI_ClearFlag(DCMI_FLAG_VSYNCRI);
    }
    else if(DCMI_GetFlagStatus(DCMI_FLAG_LINERI) == SET)
    {
    	DCMI_ClearFlag(DCMI_FLAG_LINERI);
    }
    else if(DCMI_GetFlagStatus(DCMI_FLAG_FRAMERI) == SET)
    {
    	DCMI_ClearFlag(DCMI_FLAG_FRAMERI);
    }
    else if(DCMI_GetFlagStatus(DCMI_FLAG_ERRRI) == SET)
    {
        DCMI_ClearFlag(DCMI_FLAG_ERRRI);
    }
    else if(DCMI_GetFlagStatus(DCMI_FLAG_OVFRI) == SET)
    {
        DCMI_ClearFlag(DCMI_FLAG_OVFRI);
    }
}
