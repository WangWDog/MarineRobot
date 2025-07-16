//
// Created by 大口大口喝拿铁 on 25-7-12.
//

#include "motor.h"
#include "tim.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

// motor_bias[0] 对应 Front_Left（id=1）
float motor_bias[7] = {
    0.0f, // [0] Front_Left
    0.0f, // [1] Front_Right
    0.0f, // [2] Back_Left
    0.0f, // [3] Back_Right
    0.0f, // [4] Middle_Left
    0.0f, // [5] Middle_Right
    0.0f // [6] Back_Middle
};
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

static void set_motor_pwm(TIM_HandleTypeDef* htim_pos, uint32_t ch_pos,
                          TIM_HandleTypeDef* htim_neg, uint32_t ch_neg,
                          float speed_rate, float bias)
{
    if (fabsf(speed_rate) < 0.01f)
    {
        // 静止状态，两个通道都关
        __HAL_TIM_SET_COMPARE(htim_pos, ch_pos, 0);
        __HAL_TIM_SET_COMPARE(htim_neg, ch_neg, 0);
        return;
    }

    // 映射到有效PWM范围（8800 ~ 12000）
    float compensated = speed_rate * (1.0f + bias);
    float abs_rate = fabsf(compensated);
    if (abs_rate > 1.0f) abs_rate = 1.0f; // 限幅
    if (abs_rate < 0.0f) abs_rate = 0.0f; // 限幅
    float duty = MIN_ACTIVE_DUTY + (1.0f - MIN_ACTIVE_DUTY) * abs_rate;
    if (duty > 1.0f) duty = 1.0f;

    uint32_t compare_val = (uint32_t)(duty * MAX_PWM);
    if (compare_val > MAX_PWM) compare_val = MAX_PWM;

    if (speed_rate > 0.0f)
    {
        // 正转：正通道输出，负通道为0
        __HAL_TIM_SET_COMPARE(htim_pos, ch_pos, compare_val);
        __HAL_TIM_SET_COMPARE(htim_neg, ch_neg, 0);
    }
    else
    {
        // 反转：负通道输出，正通道为0
        __HAL_TIM_SET_COMPARE(htim_pos, ch_pos, 0);
        __HAL_TIM_SET_COMPARE(htim_neg, ch_neg, compare_val);
    }
}

static void test_motor_pwm(TIM_HandleTypeDef* htim_pos, uint32_t ch_pos,
                           TIM_HandleTypeDef* htim_neg, uint32_t ch_neg,
                           int pwm)
{
    __HAL_TIM_SET_COMPARE(htim_pos, ch_pos, pwm);
    __HAL_TIM_SET_COMPARE(htim_neg, ch_neg, 0);
}

// 电机驱动函数（输入为uint8_t，支持死区）
void motor_driver(MotorMatrix id, float speed_rate)
{
    if (id < Front_Left || id > Back_Middle) return; // 合法性保护

    float bias = motor_bias[id - 1]; // 正确索引偏差表

    switch (id)
    {
    case Front_Left:
        set_motor_pwm(&htim4, TIM_CHANNEL_4, &htim4, TIM_CHANNEL_3, speed_rate, bias);
        break;
    case Front_Right:
        set_motor_pwm(&htim2, TIM_CHANNEL_2, &htim3, TIM_CHANNEL_4, speed_rate, bias);
        break;
    case Back_Left:
        set_motor_pwm(&htim4, TIM_CHANNEL_2, &htim4, TIM_CHANNEL_1, speed_rate, bias);
        break;
    case Back_Right:
        set_motor_pwm(&htim3, TIM_CHANNEL_3, &htim3, TIM_CHANNEL_2, speed_rate, bias);
        break;
    case Middle_Left:
        set_motor_pwm(&htim2, TIM_CHANNEL_4, &htim2, TIM_CHANNEL_1, speed_rate, bias);
        break;
    case Middle_Right:
        set_motor_pwm(&htim1, TIM_CHANNEL_2, &htim1, TIM_CHANNEL_3, speed_rate, bias);
        break;
    case Back_Middle:
        set_motor_pwm(&htim1, TIM_CHANNEL_4, &htim2, TIM_CHANNEL_3, speed_rate, bias);
        break;
    default:
        break;
    }
}

void test_driver(MotorMatrix id, int input)
{
    switch (id)
    {
    case Front_Right:
        test_motor_pwm(&htim2, TIM_CHANNEL_2, &htim3, TIM_CHANNEL_4, input);
        break;
    case Back_Right:
        test_motor_pwm(&htim3, TIM_CHANNEL_3, &htim3, TIM_CHANNEL_2, input);
        break;
    case Front_Left:
        test_motor_pwm(&htim4, TIM_CHANNEL_4, &htim4, TIM_CHANNEL_3, input);
        break;
    case Back_Left:
        test_motor_pwm(&htim4, TIM_CHANNEL_2, &htim4, TIM_CHANNEL_1, input);
        break;
    case Middle_Right:
        test_motor_pwm(&htim1, TIM_CHANNEL_2, &htim1, TIM_CHANNEL_3, input);
        break;
    case Middle_Left:
        test_motor_pwm(&htim1, TIM_CHANNEL_4, &htim2, TIM_CHANNEL_3, input);
        break;
    case Back_Middle:
        test_motor_pwm(&htim2, TIM_CHANNEL_4, &htim2, TIM_CHANNEL_1, input);
    default:
        // 非法ID忽略或可加入报警
        //	test_motor_pwm(&htim1, TIM_CHANNEL_1, &htim3, TIM_CHANNEL_1, input);

        break;
    }
}

void motor_driver_float(MotorMatrix id, float thrust)
{
    if (thrust > 1.0f) thrust = 1.0f;
    if (thrust < -1.0f) thrust = -1.0f;

    if (fabsf(thrust) < 0.01f)
    {
        motor_driver(id, 0.0f); // 安全停转
        return;
    }

    motor_driver(id, thrust);
}

void motor_output_all(const float motor_output[8])
{
    if (motor_output == NULL) return;

    for (uint8_t i = 0; i < 8; i++)
    {
        motor_driver_float((MotorMatrix)(i + 1), motor_output[i]);
    }
}
