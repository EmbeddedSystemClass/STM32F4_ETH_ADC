#ifndef ADC_H
#define ADC_H
#include "stm32f4xx.h"

#define ADC_BUF_LEN 32768
void ADC1_Init(void);
uint64_t Get_LastTimestamp(void);

#endif
