//
// Created by Lenovo on 25-7-13.
//
#include "cmsis_os.h"
#include "PID_task.h"
#include "pid.h"
#include "BMI088driver.h"
#include "MahonyAHRS.h"

// PID 结构体定义（只控制 pitch）
pid_type_def pid_pitch;

// 控制结果导出，其他模块可以读取这个变量
float pitch_control_output = 0.0f;

// PID 参数（你可以修改这里进行调参）
const float PID_PITCH_PARAMS[3] = {20.0f, 0.8f, 1.2f};  // kp, ki, kd
#define PID_MAX_OUTPUT 100.0f
#define PID_MAX_IOUT   30.0f

void PID_Task_Init(void)
{
    // 初始化 PID 参数（位置式 PID）
    PID_init(&pid_pitch, PID_POSITION, PID_PITCH_PARAMS, PID_MAX_OUTPUT, PID_MAX_IOUT);
}

// 主控制任务，每 10ms 执行一次
void PID_Control_Task(void *argument)
{
    float gyro[3], accel[3], temp;
    float pitch, roll, yaw;

    // 初始化 BMI088，Mahony 等（可选）
    BMI088_init();  // 确保你主程序里没重复初始化

    while (1)
    {
        // 1. 读取 BMI088 数据
        BMI088_read(gyro, accel, &temp);

        // 2. 调用 Mahony 算法进行姿态融合（你必须在工程中添加 MahonyAHRS.c/h）
        // 使用融合并保存四元数
        MahonyAHRSupdateIMU_Save(gyro[0], gyro[1], gyro[2],
                                 accel[0], accel[1], accel[2]);

        // 获取欧拉角
        MahonyAHRSGetEuler(&pitch, &roll, &yaw);


        // 3. PID 控制：目标 pitch = 0（水平）
        pitch_control_output = PID_calc(&pid_pitch, pitch, 0.0f);

        // 4. 控制结果输出由其他模块读取 pitch_control_output 即可

        osDelay(10);  // 100Hz 控制周期
    }
}
void PID_task(void *argument) {

}