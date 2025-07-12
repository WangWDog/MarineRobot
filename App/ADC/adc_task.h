#ifndef __ADC_H
#define __ADC_H
#include "main.h"
#define NUM_ADC_CHANNEL 2 //�ɼ���ͨ����

typedef struct
{
	float vol_output;
	float current_output;
}adc_data;


void adc_init(void);
void ADC_Get_Value(uint32_t adc_value[]);//��ȡ�źŲɼ���J1��J2��ADCͨ����ֵ
void adc_task(void const * argument);
#endif
