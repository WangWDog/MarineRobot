//
// Created by 大口大口喝拿铁 on 25-7-12.
//

#ifndef MOTION_H
#define MOTION_H
#include <stdint.h>

void motor_pwm_init(void);
void motor_driver(uint8_t id, float rate);
void motion_task(void const * argument);

#endif //MOTION_H
