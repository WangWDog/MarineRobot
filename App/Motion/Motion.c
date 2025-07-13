//
// Created by 大口大口喝拿铁 on 25-7-12.
//

#include "Motion.h"
#include "tim.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
 /*
	电机1：转向1对应TIM2_CH2,转向2对应TIM3_CH4
	电机2：转向1对应TIM3_CH3,转向2对应TIM3_CH2
	电机3：转向1对应TIM4_CH4,转向2对应TIM4_CH3
	电机4：转向1对应TIM4_CH2,转向2对应TIM4_CH1
	电机5：转向1对应TIM1_CH1,转向2对应TIM3_CH1
	电机6：转向1对应TIM2_CH4,转向2对应TIM2_CH1
	电机7：转向1对应TIM2_CH3,转向2对应TIM1_CH4
	电机8：转向1对应TIM1_CH3,转向2对应TIM1_CH2
 */
void motor_pwm_init(void);
void motor_driver(uint8_t id,float rate);

void motion_task(void const * argument)
{
	motor_pwm_init();
	for(;;)
	{
		//此处放控制逻辑
		osDelay(10);
	}
}


void motor_pwm_init(void)//电机初始化，除了打开pwm之外，还要输出置0防止抽风
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
}
//电机驱动任务,电机使用AT8236模块输出，pwm频率为0-2000,输入值为电机id和对应的速度比率，如比率为正则转向1pwm输出，转向2pwm归0，反之则反过来

void motor_driver(uint8_t id,float rate)
{
	float speed_rate = rate/100.0f;
	switch(id)
	{
		case 1:
			if(speed_rate>0)
			{
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,speed_rate*2000);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
			}
			else
			{
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,-speed_rate*2000);
			}
			break;
		case 2:
			if(speed_rate>0)
			{
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,speed_rate*2000);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
			}
			else
			{
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,-speed_rate*2000);
			}
			break;
		case 3:
			if(speed_rate>0)
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,speed_rate*2000);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
			}
			else
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,-speed_rate*2000);
			}
			break;
		case 4:
			if(speed_rate>0)
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,speed_rate*2000);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
			}
			else
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,-speed_rate*2000);
			}
			break;
		case 5:
			if(speed_rate>0)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,speed_rate*2000);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
			}
			else
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,-speed_rate*2000);
			}
			break;
		case 6:
			if(speed_rate>0)
			{
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,speed_rate*2000);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
			}
			else
			{
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,-speed_rate*2000);
			}
			break;
		case 7:
			if(speed_rate>0)
			{
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,speed_rate*2000);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
			}
			else
			{
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,-speed_rate*2000);
			}
			break;
		case 8:
			if(speed_rate>0)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,speed_rate*2000);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
			}
			else
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,-speed_rate*2000);
			}
			break;
		}
}


// 统一电机控制函数：根据结构体输入的 8 路占空比统一设置电机
void motor_pwm_output(const MotorPWMCommand_t* pwm_cmd)
{
	if (pwm_cmd == NULL) return;

	for (uint8_t i = 0; i < 8; i++) {
		motor_driver(i + 1, pwm_cmd->motor_pwm[i]);  // 电机编号从1开始
	}
}
