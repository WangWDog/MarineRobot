//
// Created by 13033 on 2025/7/12.
//

#include "imu_task.h"

#include "BMI088driver.h"
#include "cmsis_os.h"
#include "MahonyAHRS.h"
#include "Motion.h"
imu_struct imu_data;//陀螺仪角度接口

#define DES_TEMP    40.0f
#define KP          100.0f
#define KI          50.0f
#define KD          10.0f
#define MAX_OUT     500
#define rad2deg 57.2957795f


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
    osDelay(10);
    while(BMI088_init())
    {
        osDelay(100);
    }
    AHRS_init(imuQuat);
    /* Infinite loop */
    for(;;)
    {
        BMI088_read(imu_data.gyro, imu_data.acc, &imu_data.temp);
        AHRS_update(imuQuat,imu_data.gyro,imu_data.acc);
        GetAngle(imuQuat, imu_data.imuAngle + INS_YAW_ADDRESS_OFFSET, imu_data.imuAngle + INS_PITCH_ADDRESS_OFFSET, imu_data.imuAngle + INS_ROLL_ADDRESS_OFFSET);
        osDelay(1);
    }

}
// 统一电机控制函数：根据结构体输入的 8 路占空比统一设置电机
void motor_pwm_output(const MotorPWMCommand_t* pwm_cmd)
{
    if (pwm_cmd == NULL) return;

    for (uint8_t i = 0; i < 8; i++) {
        motor_driver(i + 1, pwm_cmd->motor_pwm[i]);  // 电机编号从1开始
    }
}


