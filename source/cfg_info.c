/*
Модуль работы с переменными и константами
*/

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
//#include "config.h"
#include "cfg_info.h"

//****************************************************************************
// именованные константы

//****************************************************************************
// внешние переменные
extern const sConfigInfo configInfoHard;

//****************************************************************************
// глобальные переменные доступные извне
sConfigInfo configInfo;			// структура для хранения конфигурации прибора в ОЗУ

//****************************************************************************
// внутренние переменные

// указатели на хранимые во FLASH буферы конфигурационной информации
// одновременно активен только один буфер
static sConfigInfo *pMyInfoActive;

// для хранения и возможности переписывания информации используется 2 банка данной информации
// работающие по очереди.
static sConfigInfo MyInfo __attribute__((section(".flash_cfg1")));
static sConfigInfo MyInfo1 __attribute__((section(".flash_cfg2")));


#define VOLTAGE_RANGE           (uint8_t)VoltageRange_3

#define PAGE0_ID               FLASH_Sector_1
#define PAGE1_ID               FLASH_Sector_2

//****************************************************************************

static int FLASH_ErasePage (uint32_t  *pageAddr)
{

	uint32_t sector;

	if(pageAddr==&MyInfo)
	{
		sector=PAGE0_ID;
	}
	else
	{
		sector=PAGE1_ID;
	}

	FLASH_Unlock();

	if (FLASH_EraseSector(sector,VOLTAGE_RANGE)!=FLASH_COMPLETE)
	{
		FLASH_Lock();
		return -1;
	}
	FLASH_Lock();
	return 0;
}

// процедура записи во FLASH блока данных 
// return value: 0 - OK, -1 - ERROR

static int FlashWrite_32(uint32_t  *from, uint32_t  *to, uint16_t cnt)
{


	if(FLASH_ErasePage(to)<0)
	{
		FLASH_Lock();
		return -1;
	}
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
				FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
				FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	FLASH_Unlock();
	for (int i=0; i<cnt; i++, from++, to++)
	{ // запишем на 1 слово больше, чтобы исключить "обрезание" при делении на 4
		if (FLASH_ProgramWord( (uint32_t)to, *from) != FLASH_COMPLETE)
		{
			FLASH_Lock();
			return -1;
		}
	}
	FLASH_Lock();
	return 0;
}

//****************************************************************************
// процедура считывания конфигурационной информации прибора

int ConfigInfoRead (void)
{
// определим активный буфер конфигурационной информации
  if(strcmp(MyInfo.Label, LABEL_CFG_SECTOR) == 0)
  {			//
    pMyInfoActive = &MyInfo;			// активный буфер - 1
//	printf("MyInfo\r\n");
  }
  else
  {
    if(strcmp(MyInfo1.Label, LABEL_CFG_SECTOR) == 0)
    {	//
      pMyInfoActive = &MyInfo1;		// активный буфер - 2
//	  printf("MyInfo1\r\n");
    }
    else
    {	// странно, но ни в одном буфере нет нужной информации - восстановим ее! (либо первый запуск прибора)
      if(FlashWrite_32((uint32_t*)&configInfoHard, (uint32_t*)&MyInfo, sizeof(MyInfo)/4 +1 ) < 0)
      {
//				memcpy(&configInfo, &configInfoHard, sizeof(MyInfo));
        return -1;
      }
      // сотрем второй буфер FLASH
      if(FLASH_ErasePage((uint32_t*)&MyInfo1) < 0)
      {
        return -1;
      }
      pMyInfoActive = &MyInfo;			// активный буфер - 1
//	  printf("configInfoHard\r\n");
    }
  }
  memcpy(&configInfo, pMyInfoActive, sizeof(MyInfo));	
  return 0;
}

//****************************************************************************
// Запись новой информации о приборе (в виде структуры) во FLASH из configInfo
// return 0 - OK, -1 - Error

int ConfigInfoWrite(void)
{
	sConfigInfo *pMyInfoNoActive;
   if(pMyInfoActive == &MyInfo)
   {
		pMyInfoNoActive = &MyInfo1;
   }
   else
   {
		pMyInfoNoActive = &MyInfo;
   }

	if(FlashWrite_32((uint32_t*)&configInfo, (uint32_t*)pMyInfoNoActive, sizeof(MyInfo)/4 +1 ) < 0)
	{ // запишем обновление во FLASH
		return -1;
	}
  // сотрем второй буфер FLASH
	if(FLASH_ErasePage((uint32_t*)&pMyInfoActive) < 0)
	{
		return -1;
	}
  pMyInfoActive = pMyInfoNoActive;
  return 0;
}

