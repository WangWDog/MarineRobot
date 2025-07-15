//
// Created by 大口大口喝拿铁 on 25-7-12.
//

#ifndef MOTION_H
#define MOTION_H
#include <stdint.h>
#define MAX_PWM  12000     // PWM自动重装值
#define MIN_ACTIVE_DUTY 0.733f  // 8800 / 12000
void motor_pwm_init(void);
void hover(void);
// void motor_driver(uint8_t id, float rate);
void motion_task(void const * argument);

typedef struct {
    float motor_pwm[8];
} MotorPWMCommand_t;
typedef enum  {
    Front_Left = 1,
    Front_Right = 2,
    Back_Left = 3,
    Back_Right = 4,
    Middle_Left = 5,
    Middle_Right = 6,
    Back_Middle = 7,
} MotorMatrix;
void test_driver(MotorMatrix id, int input);
void motor_pwm_init(void);
void motor_driver(MotorMatrix id, float speed_rate);
void motor_pwm_output(const MotorPWMCommand_t* pwm_cmd); // 新加：电机整体输出控制函数
void motor_output_all(const float motor_output[8]);
void motion_task(void const * argument);
#endif //MOTION_H
