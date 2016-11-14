#include "settings.h"
#include "eeprom.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

stSettings Settings;

stSettings Settings_Default={192,168,109,150,1000,192,168,109,140};

void Settings_Init(stSettings *Settings)
{
	//--jumper pin init
	GPIO_InitTypeDef 			GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//----------------

	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_10)/*jumper off*/)
	{
		Settings_Load(&Settings);
	}
	else
	{
		Settings=Settings_Default;
	}
}

void Settings_Load(stSettings *Settings)
{

}

void Settings_Save(stSettings *Settings)
{

}
