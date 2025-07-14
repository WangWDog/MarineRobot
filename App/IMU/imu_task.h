//
// Created by 13033 on 2025/7/12.
//

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdbool.h>

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2
typedef struct
{
    float gyro[3];
    float acc[3];
    float imuAngle[3];
    float temp;

}imu_struct;
// extern imu_struct imu_data;//陀螺仪角度接口
void imu_task(void const * argument);
bool imu_get_accel(float* ax, float* ay, float* az);
bool imu_get_euler(float* yaw, float* pitch, float* roll);
#endif //IMU_TASK_H
