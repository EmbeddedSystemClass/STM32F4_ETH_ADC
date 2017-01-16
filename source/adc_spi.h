#ifndef ADC_SPI_H
#define ADC_SPI_H

#include "stm32f4xx.h"

void ADC_Ext_Init(void);
uint64_t ADC_GetLastTimestamp(void);
void ADC_GetLastVal(void);

#endif
