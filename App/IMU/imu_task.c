//
// Created by 13033 on 2025/7/12.
//

#include "imu_task.h"

#include <stdbool.h>

#include "BMI088driver.h"
#include "cmsis_os.h"
#include "MahonyAHRS.h"
#include "../Driver/motor.h"
volatile imu_struct imu_data; //陀螺仪角度接口

#define DES_TEMP    40.0f
#define KP          100.0f
#define KI          50.0f
#define KD          10.0f
#define MAX_OUT     500
#define rad2deg 57.2957795f
#define YAW_DRIFT_THRESHOLD 2.0f  // 漂移判定阈值（单位：度）

// osMutexId imu_mutexHandle;         // 互斥锁句柄
// osMutexDef(imu_mutex);            // 静态互斥锁定义（CMSIS-RTOS风格）

float imuQuat[4] = {0.0f};
float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;
static float drift_rate = 0.0f;
static uint32_t drift_estimate_start_time = 0;
static float drift_estimate_start_yaw = 0.0f;
static uint32_t calibration_time_ms = 0;
static float yaw_offset = 0.0f;
bool drift_estimated = false;

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

void GetAngle(float q[4], float* yaw, float* pitch, float* roll)
{
    *yaw = rad2deg * atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = rad2deg * asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = rad2deg * atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

bool estimate_drift_rate(void)
{
    float yaw, pitch, roll;

    // 第一次进入，初始化：记录当前 yaw 和当前时间为起始参考
    if (drift_estimate_start_time == 0)
    {
        if (imu_get_euler(&yaw, &pitch, &roll))  // 获取当前 yaw
        {
            drift_estimate_start_yaw = yaw;                  // 记录起始 yaw
            drift_estimate_start_time = osKernelSysTick();   // 记录起始时间（单位：ms）
        }
        return false;  // 本次未完成估算，等待下一次进入
    }

    // 获取当前时间并计算已过去的时间（单位：秒）
    uint32_t now = osKernelSysTick();
    float dt = (now - drift_estimate_start_time) / 1000.0f;

    // 如果已过 5 秒，准备计算漂移率
    if (dt >= 10.0f)
    {
        if (imu_get_euler(&yaw, &pitch, &roll))  // 再次读取当前 yaw
        {
            // 计算 yaw 变化量（处理角度跳变）
            float dyaw = yaw - drift_estimate_start_yaw;
            if (dyaw > 180.0f) dyaw -= 360.0f;
            if (dyaw < -180.0f) dyaw += 360.0f;

            if (fabsf(dyaw) > YAW_DRIFT_THRESHOLD)
            {
                // 放弃本次估算，重置初始时间，准备重新采集
                drift_estimate_start_time = 0;
                drift_rate = 0.0f;
                return false;
            }
            yaw_offset = yaw;  // ⭐️ 关键点：将当前航向视作0度
            // === 计算并保存漂移速率 ===
            drift_rate = dyaw / dt;   // 角度 / 时间 = °/s

            // 设置校准参考时间（用于后续漂移补偿）
            calibration_time_ms = osKernelSysTick();

            // 标记为“已完成估算”状态（后续不再重复）
            drift_estimate_start_time = 1;  // 任意非零值防止重复测

            return true;  // ✅ 漂移率估算完成
        }
    }

    return false;  // 尚未完成估算，等待满足 5 秒
}

void imu_task(void const* argument)
{
    /* USER CODE BEGIN ImuTask_Entry */
    // imu_mutexHandle = osMutexCreate(osMutex(imu_mutex)); // 创建互斥锁
    osDelay(10); // 系统稳定后再初始化
    while (BMI088_init())
    {
        osDelay(100);
    }
    AHRS_init(imuQuat);
    /* Infinite loop */
    for (;;)
    {
        if (!drift_estimated)
        {
            drift_estimated = estimate_drift_rate();  // 一旦估算完成，就不再进入
        }
        // osMutexWait(imu_mutexHandle, osWaitForever); // 加锁
        BMI088_read(imu_data.gyro, imu_data.acc, &imu_data.temp);
        AHRS_update(imuQuat, imu_data.gyro, imu_data.acc);
        GetAngle(imuQuat, imu_data.imuAngle + INS_YAW_ADDRESS_OFFSET, imu_data.imuAngle + INS_PITCH_ADDRESS_OFFSET,
                 imu_data.imuAngle + INS_ROLL_ADDRESS_OFFSET);
        // osMutexRelease(imu_mutexHandle);  // 🔓解锁（其他任务现在可以安全读取）
        osDelay(1);
    }
}

bool imu_get_accel(float* ax, float* ay, float* az)
{
    if (!ax || !ay || !az) return false;

    // osMutexWait(imu_mutexHandle, osWaitForever);  // 🔒
    *ax = imu_data.acc[0];
    *ay = imu_data.acc[1];
    *az = imu_data.acc[2];
    // osMutexRelease(imu_mutexHandle);              // 🔓
    return true;
}
bool imu_get_euler(float* yaw, float* pitch, float* roll)
{
    if (!yaw || !pitch || !roll) return false;  // 判空防止非法指针

    float raw_yaw   = imu_data.imuAngle[INS_YAW_ADDRESS_OFFSET];
    float raw_pitch = imu_data.imuAngle[INS_PITCH_ADDRESS_OFFSET];
    float raw_roll  = imu_data.imuAngle[INS_ROLL_ADDRESS_OFFSET];

    // === 加入漂移补偿 ===


    uint32_t now_ms = osKernelSysTick();
    float delta_time = (now_ms - calibration_time_ms) / 1000.0f;  // 秒
    float yaw_drift = drift_rate * delta_time;

    float corrected_yaw = raw_yaw - yaw_drift - yaw_offset;

    // 保持在 [-180, 180]
    if (corrected_yaw > 180.0f)  corrected_yaw -= 360.0f;
    if (corrected_yaw < -180.0f) corrected_yaw += 360.0f;

    *yaw = corrected_yaw;
    *pitch = raw_pitch;
    *roll  = raw_roll;

    return true;
}