#ifndef ADC_H
#define ADC_H
#include "stm32f4xx.h"

#define ADC_BUF_LEN 32768
#define ADC_CHN_NUM	8

void 	 ADC1_Init(void);
uint64_t Timestamp_GetLastTimestamp(void);
void 	 ADC_GetLastData(void);

#endif
