//
// Created by 13033 on 2025/7/12.
//

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "cmsis_os.h"

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2
void ImuTask_Entry(void const * argument);
typedef struct
{
    float gyro[3];
    float acc[3];
    float imuAngle[3];
    float temp;

}imu_struct;
extern imu_struct imu_data;//陀螺仪角度接口
void imu_task(void const * argument);
#endif //IMU_TASK_H
