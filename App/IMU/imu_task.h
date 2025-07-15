//
// Created by 13033 on 2025/7/12.
//

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

    // 姿态角数组偏移量
#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

    // IMU数据结构体
    typedef struct
    {
        float gyro[3];       // 角速度 XYZ (单位: rad/s)
        float acc[3];        // 加速度 XYZ (单位: m/s^2)
        float imuAngle[3];   // 欧拉角 yaw/pitch/roll (单位: deg)
        float temp;          // 温度 (单位: °C)
    } imu_struct;

    // 全局IMU数据声明（定义在 imu_task.c 中）
    extern volatile imu_struct imu_data;

    // 任务创建接口
    void imu_task(void const * argument);

    // 获取当前欧拉角（线程安全）
    bool imu_get_euler(float* yaw, float* pitch, float* roll);

    // 获取当前加速度（线程安全）
    bool imu_get_accel(float* ax, float* ay, float* az);

#ifdef __cplusplus
}
#endif

#endif // IMU_TASK_H
