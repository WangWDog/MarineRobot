//
// Created by 13033 on 2025/7/12.
//

#include "imu_task.h"

#include <stdbool.h>
#include <math.h>

#include "BMI088driver.h"
#include "cmsis_os.h"
#include "MahonyAHRS.h"
#include "../Driver/Motor.h"

// 姿态角结构体
volatile imu_struct imu_data;

// 四元数初值
static float imuQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};

// 弧度转角度系数
#define rad2deg 57.2957795f

// 互斥锁
osMutexId imu_mutexHandle;
osMutexDef(imu_mutex);

// 初始化四元数
void AHRS_init(float quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

// 姿态更新函数（Mahony滤波）
void AHRS_update(float quat[4], float gyro[3], float accel[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

// 四元数转欧拉角
void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw   = rad2deg * atan2f(2.0f * (q[0]*q[3] + q[1]*q[2]), 2.0f * (q[0]*q[0] + q[1]*q[1]) - 1.0f);
    *pitch = rad2deg * asinf(-2.0f * (q[1]*q[3] - q[0]*q[2]));
    *roll  = rad2deg * atan2f(2.0f * (q[0]*q[1] + q[2]*q[3]), 2.0f * (q[0]*q[0] + q[3]*q[3]) - 1.0f);
}

// 姿态任务
void imu_task(void const * argument)
{
    osDelay(10); // 系统稳定后初始化
    while (BMI088_init())
    {
        osDelay(100);
    }

    // 创建互斥锁
    imu_mutexHandle = osMutexCreate(osMutex(imu_mutex));
    AHRS_init(imuQuat);

    for (;;)
    {
        // 读取原始IMU数据
        BMI088_read(imu_data.gyro, imu_data.acc, &imu_data.temp);

        osMutexWait(imu_mutexHandle, osWaitForever); // 🔒加锁

        // 姿态解算
        AHRS_update(imuQuat, imu_data.gyro, imu_data.acc);
        GetAngle(imuQuat,
                 imu_data.imuAngle + INS_YAW_ADDRESS_OFFSET,
                 imu_data.imuAngle + INS_PITCH_ADDRESS_OFFSET,
                 imu_data.imuAngle + INS_ROLL_ADDRESS_OFFSET);

        osMutexRelease(imu_mutexHandle); // 🔓解锁

        osDelay(1); // 1ms周期
    }
}

// 获取欧拉角（线程安全）
bool imu_get_euler(float* yaw, float* pitch, float* roll)
{
    if (!yaw || !pitch || !roll) return false;

    osMutexWait(imu_mutexHandle, osWaitForever);
    *yaw   = imu_data.imuAngle[INS_YAW_ADDRESS_OFFSET];
    *pitch = imu_data.imuAngle[INS_PITCH_ADDRESS_OFFSET];
    *roll  = imu_data.imuAngle[INS_ROLL_ADDRESS_OFFSET];
    osMutexRelease(imu_mutexHandle);

    return true;
}

// 获取加速度（线程安全）
bool imu_get_accel(float* ax, float* ay, float* az)
{
    if (!ax || !ay || !az) return false;

    osMutexWait(imu_mutexHandle, osWaitForever);
    *ax = imu_data.acc[0];
    *ay = imu_data.acc[1];
    *az = imu_data.acc[2];
    osMutexRelease(imu_mutexHandle);

    return true;
}
