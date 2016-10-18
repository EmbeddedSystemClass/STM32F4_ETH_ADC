#ifndef ADC_H
#define ADC_H
#include "stm32f4xx.h"



void 	 ADC1_Init(void);
uint64_t Timestamp_GetLastTimestamp(void);
void 	 ADC_GetLastData(void);

#endif
