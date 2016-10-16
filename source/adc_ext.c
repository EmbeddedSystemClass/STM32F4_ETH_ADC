#include "adc_ext.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"

#define SPI1_RX_BUFF_SIZE	4
__IO uint16_t ExtAdcRxBuff[SPI1_RX_BUFF_SIZE];

void ADC_Ext_TimInit(void);
void ADC_Ext_SPIInit(void);

void ADC_ExtInit(void)
{
	ADC_Ext_TimInit();
	ADC_Ext_SPIInit();

}

void ADC_Ext_TimInit(void)
{
	TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef           TIM_OCInitStructure;
	GPIO_InitTypeDef 			GPIO_InitStructure;

	 /*    TIM1 Configuration
	 */
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	 //
	 GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_11;
	 GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	 GPIO_Init(GPIOE, &GPIO_InitStructure);
	 GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	 GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

	 /* Time base configuration */
	 TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	 TIM_TimeBaseStructure.TIM_Prescaler =  168 - 1;
	 TIM_TimeBaseStructure.TIM_Period = 2500 - 1;
	 TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);



	 /* PWM1 Mode configuration: Channel1 */
	 TIM_OCStructInit(&TIM_OCInitStructure);
	 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	 TIM_OCInitStructure.TIM_Pulse = 2400;
	 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	 TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

	 TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	 TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	 /* PWM1 Mode configuration: Channel2 */
	 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	 TIM_OCInitStructure.TIM_Pulse = 2350;
	 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	 TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

	 TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	 TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
	 TIM_ARRPreloadConfig(TIM1, ENABLE);
	 TIM_Cmd(TIM1, ENABLE);
	 TIM_CtrlPWMOutputs(TIM1, ENABLE);

	 TIM_DMACmd(TIM1, TIM_DMA_CC2, ENABLE);
}

void ADC_Ext_SPIInit(void)
{
	  SPI_InitTypeDef  SPI_InitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;

	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

		/*!< Enable GPIO clocks */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	  /*!< SPI pins configuration *************************************************/
	  /*!< Connect SPI pins to AF5 */
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1 );
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1 );
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1 );


	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

		/*!< SPI SCK pin configuration */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);


	  /*!< SPI MOSI pin configuration */
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);


	  /*!< SPI MISO pin configuration */
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

		/*!< Configure CS pin in output pushpull mode ********************/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

		/*!< SPI configuration */
	 SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	 SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	 SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	 SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	 SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // APB2 / 8 = 10.5 MHz
	 SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	 SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	 SPI_InitStructure.SPI_NSS = SPI_NSS_Soft ;
	 SPI_Init(SPI1, &SPI_InitStructure);


	  /*!< Enable the SPI  */
	  SPI_Cmd(SPI1, ENABLE);

	  //--------------DMA---------------------


	  DMA_InitTypeDef DMA_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;

	    /* Enable RCC clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	    /* Disable stream if it was in use */
	    DMA_DeInit(DMA2_Stream0);
	    while(RESET != DMA_GetCmdStatus(DMA2_Stream0));

	    /* Initiliaze DMA2 Channel 3 Stream 0 */
//	    DMA_InitStructure.DMA_Channel = DMA_Channel_3;
//	    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	    DMA_InitStructure.DMA_PeripheralBaseAddr =  (uint32_t) &(SPI1->DR);
//	    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) (ExtAdcRxBuff);
//	    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//	    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//	    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	    DMA_InitStructure.DMA_BufferSize = SPI1_RX_BUFF_SIZE;
//	    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//	    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	    DMA_InitStructure.DMA_BufferSize = SPI1_RX_BUFF_SIZE;
	    DMA_InitStructure.DMA_Channel = DMA_Channel_3;
	    DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)ExtAdcRxBuff;
	    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPI1->DR);
	    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	    DMA_InitStructure.DMA_Mode =  DMA_Mode_Circular; //DMA_Mode_Normal;
	    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	    /* Inialize NVIC for the DMA interrupt */
	    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&NVIC_InitStructure);

	    /* Connect SPI and DMA2 */
	    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);


			/* Enable DMA2 Channel 3 Stream 0 for the SPI1 RX */
	    DMA_Cmd(DMA2_Stream0, ENABLE);
}

void DMA2_Stream0_IRQHandler(void)
{
  /* Test on DMA Stream Transfer Complete interrupt */
  if (DMA_GetITStatus(DMA2_Stream0, SPI_I2S_DMAReq_Rx))
  {
    /* Clear DMA Stream Transfer Complete interrupt pending bit */
    DMA_ClearITPendingBit(DMA2_Stream0, SPI_I2S_DMAReq_Rx);


  }
}
