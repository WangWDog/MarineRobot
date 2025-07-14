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
void clear_all_pwm(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}

// 电机初始化：开启所有PWM并默认输出0
void motor_pwm_init(void)
{
	// 启动 TIM1 通道
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	// 启动 TIM2 通道
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	// 启动 TIM3 通道
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	// 启动 TIM4 通道
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	// 所有通道清零
	clear_all_pwm();
}

// 电机控制基础函数：根据转向分别设置两个通道
static void set_motor_pwm(TIM_HandleTypeDef *htim_pos, uint32_t ch_pos,
                          TIM_HandleTypeDef *htim_neg, uint32_t ch_neg,
                          float speed_rate)
{
	if (speed_rate > 0.0f) {
		__HAL_TIM_SET_COMPARE(htim_pos, ch_pos, speed_rate * MAX_PWM);
		__HAL_TIM_SET_COMPARE(htim_neg, ch_neg, 0);
	} else if (speed_rate < 0.0f) {
		__HAL_TIM_SET_COMPARE(htim_pos, ch_pos, 0);
		__HAL_TIM_SET_COMPARE(htim_neg, ch_neg, -speed_rate * MAX_PWM);
	} else {
		__HAL_TIM_SET_COMPARE(htim_pos, ch_pos, 0);
		__HAL_TIM_SET_COMPARE(htim_neg, ch_neg, 0);
	}
}

// 电机驱动函数（输入为uint8_t，支持死区）
void motor_driver(uint8_t id, uint8_t input)
{
	float speed_rate = 0.0f;

	// 死区处理：125~129之间视为0
	if (input < DEADZONE_MIN || input > DEADZONE_MAX) {
		speed_rate = (input - 127.0f) / 128.0f;
	}

	switch (id)
	{
		case 1:
			set_motor_pwm(&htim2, TIM_CHANNEL_2, &htim3, TIM_CHANNEL_4, speed_rate);
			break;
		case 2:
			set_motor_pwm(&htim3, TIM_CHANNEL_3, &htim3, TIM_CHANNEL_2, speed_rate);
			break;
		case 3:
			set_motor_pwm(&htim4, TIM_CHANNEL_4, &htim4, TIM_CHANNEL_3, speed_rate);
			break;
		case 4:
			set_motor_pwm(&htim4, TIM_CHANNEL_2, &htim4, TIM_CHANNEL_1, speed_rate);
			break;
		case 5:
			set_motor_pwm(&htim1, TIM_CHANNEL_1, &htim3, TIM_CHANNEL_1, speed_rate);
			break;
		case 6:
			set_motor_pwm(&htim2, TIM_CHANNEL_4, &htim2, TIM_CHANNEL_1, speed_rate);
			break;
		case 7:
			set_motor_pwm(&htim2, TIM_CHANNEL_3, &htim1, TIM_CHANNEL_4, speed_rate);
			break;
		case 8:
			set_motor_pwm(&htim1, TIM_CHANNEL_3, &htim1, TIM_CHANNEL_2, speed_rate);
			break;
		default:
			// 非法ID忽略或可加入报警
			break;
	}
}

// 一次性控制8路电机的占空比输入（结构体接口）
void motor_pwm_output(const MotorPWMCommand_t* pwm_cmd)
{
	if (pwm_cmd == NULL) return;

	for (uint8_t i = 0; i < 8; i++) {
		motor_driver(i + 1, pwm_cmd->motor_pwm[i]);  // 电机编号从1开始
	}
}

// RTOS中的电机任务（初始化+主循环）
void motion_task(void const * argument)
{
	motor_pwm_init();

	for (;;)
	{
		// TODO: 这里可以添加运动控制逻辑
		osDelay(10);  // 10ms控制周期
	}
}