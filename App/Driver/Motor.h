//
// Created by 大口大口喝拿铁 on 25-7-12.
//

#ifndef MOTION_H
#define MOTION_H
#include <stdint.h>
#define MAX_PWM 12000
#define DEADZONE_MIN 125
#define DEADZONE_MAX 129
void motor_pwm_init(void);
// void motor_driver(uint8_t id, float rate);
void motion_task(void const * argument);

typedef struct {
    float motor_pwm[8];
} MotorPWMCommand_t;
typedef enum  {
    Back_Right = 1,
    Front_Right = 2,
    Front_Left = 3,
    Back_Left = 4,
    Middle_Right = 5,
    Middle_Left = 6,
} MotorMatrix;
void motor_driver(MotorMatrix id, uint8_t input);
void motor_pwm_init(void);
// void motor_driver(uint8_t id, float rate);
void motor_pwm_output(const MotorPWMCommand_t* pwm_cmd); // 新加：电机整体输出控制函数
void motion_task(void const * argument);
#endif //MOTION_H
