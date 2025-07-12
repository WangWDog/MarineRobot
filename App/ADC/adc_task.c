#include "adc_task.h"

#include "adc.h"
#include "cmsis_os.h"
#define ADC_BUFFER_LENGTH 32*2
#define ADC_REF_VOLTAGE 3.3f
#define ADC_RESOLUTION 4096.0f
#define ADC_GAIN 0.5f    // ����ϵ���������뱻˥��Ϊһ��


adc_data adc_info;//ADC���ݽӿ�

uint32_t adc_value[NUM_ADC_CHANNEL] = {0};

void ADC_Get_Value(uint32_t adc_value[])
{
	int i = 0;
	for(i = 0;i < NUM_ADC_CHANNEL;i++)
	{
		HAL_ADC_Start(&hadc2);//����ADC1
		HAL_ADC_PollForConversion(&hadc2,10);//�ȴ�ת�����
		adc_value[i] = HAL_ADC_GetValue(&hadc2);//��ȡADC��ֵ
	}
	HAL_ADC_Stop(&hadc2);
	return;
}

void adc_task(void const * argument)
{
	for(;;)
	{
		ADC_Get_Value(adc_value);
		adc_info.vol_output =adc_value[0]/65535.0f*3.3f*2.0f*2.5f;
		adc_info.current_output =adc_value[1]/65535.0f*3.3f*2.0f*5.0f;
		osDelay(500);
	}
}
