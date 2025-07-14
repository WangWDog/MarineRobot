//
// Created by 13033 on 2025/7/12.
//

#include "imu_task.h"

#include <stdbool.h>

#include "BMI088driver.h"
#include "cmsis_os.h"
#include "MahonyAHRS.h"
#include "Motion.h"
volatile imu_struct imu_data;//陀螺仪角度接口

#define DES_TEMP    40.0f
#define KP          100.0f
#define KI          50.0f
#define KD          10.0f
#define MAX_OUT     500
#define rad2deg 57.2957795f

osMutexId imu_mutexHandle;         // 互斥锁句柄
osMutexDef(imu_mutex);            // 静态互斥锁定义（CMSIS-RTOS风格）

float imuQuat[4] = {0.0f};
float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;

void AHRS_init(float quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(float quat[4], float gyro[3], float accel[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}
void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = rad2deg *atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = rad2deg *asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = rad2deg *atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}
void imu_task(void const * argument)
{
    /* USER CODE BEGIN ImuTask_Entry */
    osDelay(10); // 系统稳定后再初始化
    while(BMI088_init())
    {
        osDelay(100);
    }
    imu_mutexHandle = osMutexCreate(osMutex(imu_mutex)); // 创建互斥锁

    AHRS_init(imuQuat);
    /* Infinite loop */
    for(;;)
    {
        osMutexWait(imu_mutexHandle, osWaitForever); // 加锁
        BMI088_read(imu_data.gyro, imu_data.acc, &imu_data.temp);
        AHRS_update(imuQuat,imu_data.gyro,imu_data.acc);
        GetAngle(imuQuat, imu_data.imuAngle + INS_YAW_ADDRESS_OFFSET, imu_data.imuAngle + INS_PITCH_ADDRESS_OFFSET, imu_data.imuAngle + INS_ROLL_ADDRESS_OFFSET);
        osMutexRelease(imu_mutexHandle);  // 🔓解锁（其他任务现在可以安全读取）
        osDelay(1);
    }

}
bool imu_get_euler(float* yaw, float* pitch, float* roll)
{
    if (!yaw || !pitch || !roll) return false;  // 判空防止非法指针

    osMutexWait(imu_mutexHandle, osWaitForever);  // 🔒上锁

    *yaw   = imu_data.imuAngle[INS_YAW_ADDRESS_OFFSET];
    *pitch = imu_data.imuAngle[INS_PITCH_ADDRESS_OFFSET];
    *roll  = imu_data.imuAngle[INS_ROLL_ADDRESS_OFFSET];

    osMutexRelease(imu_mutexHandle);  // 🔓解锁
    return true;  // 返回成功
}
bool imu_get_accel(float* ax, float* ay, float* az)
{
    if (!ax || !ay || !az) return false;

    osMutexWait(imu_mutexHandle, osWaitForever);  // 🔒
    *ax = imu_data.acc[0];
    *ay = imu_data.acc[1];
    *az = imu_data.acc[2];
    osMutexRelease(imu_mutexHandle);              // 🔓
    return true;
}