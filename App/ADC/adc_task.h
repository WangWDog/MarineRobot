#ifndef __ADC_H
#define __ADC_H
#include "main.h"
#define NUM_ADC_CHANNEL 2 //采集板通道数

typedef struct
{
	float vol_output;
	float current_output;
}adc_data;


void adc_init(void);
void ADC_Get_Value(uint32_t adc_value[]);//获取信号采集板J1和J2中ADC通道的值
void adc_task(void const * argument);
#endif
