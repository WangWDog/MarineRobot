// motion.c - 主控制逻辑
#include "motion.h"
#include "pid.h"
#include "imu_task.h"
#include "motor.h"
#include "cmsis_os.h"
#include <math.h>

// === 可调灵敏度参数（原宏定义替换为变量） ===
float PITCH_SENSI = 0.03f;
float YAW_SENSI = 5.0f;

float FAST_SENSI_YAW = 1.2f;
float FAST_SENSI_PITCH = 1.2f;
float FAST_SENSI_X = 1.2f;
float FAST_SENSI_Y = 1.2f;
float FAST_SENSI_DEEP = 1.2f;

float SLOW_SENSI_YAW = 0.5f;
float SLOW_SENSI_PITCH = 0.5f;
float SLOW_SENSI_X = 0.5f;
float SLOW_SENSI_Y = 0.5f;
float SLOW_SENSI_DEEP = 0.5f;


bool is_roll_pid = true;
bool is_pitch_pid = true;
bool is_yaw_pid = true;

enum {
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_YAW = 2,
    AXIS_ROLL = 3,
    AXIS_PITCH = 4,
    AXIS_Z = 5
};

// 7 个电机 × 6 个通道（x, y, yaw, roll, pitch, z）
float motor_mix_gain[7][6] = {
    // x,     y,     yaw,   roll,  pitch,  z
    { -1.0f, -1.0f, -1.0f,  0.0f,   0.0f,  0.0f }, // M1 - Back_Right
    { -1.0f, -1.0f,  1.0f,  0.0f,   0.0f,  0.0f }, // M2 - Front_Right
    {  1.0f,  1.0f, -1.0f,  0.0f,   0.0f,  0.0f }, // M3 - Front_Left
    { -1.0f,  1.0f,  1.0f,  0.0f,   0.0f,  0.0f }, // M4 - Back_Left
    {  0.0f,  0.0f,  0.0f, -1.0f,   1.0f,  1.0f }, // M5 - Middle_Right
    {  0.0f,  0.0f,  0.0f,  1.0f,   1.0f,  1.0f }, // M6 - Middle_Left
    {  0.0f,  0.0f,  0.0f,  0.0f,   2.0f, -1.0f }  // M7 - BackMiddle
};

// === 控制目标变量（静态封装） ===
static float target_x = 0.0f; // 前进 / 后退
static float target_y = 0.0f; // 横移
float target_yaw = 0.0f; // 偏航差速（开环）
float target_pitch = 0.0f; // 俯仰角目标（度）
float target_roll = 0.0f; // 横滚角目标（暂设为0）
static float z_thrust = 0.0f; // 上浮 / 下潜推力（-1.0~+1.0）

// === PID 控制器 ===
static pid_type_def pid_yaw;
static pid_type_def pid_roll;
static pid_type_def pid_pitch;

float roll_param[3] = {0.1f, 0.0f, 0.01f};
float pitch_param[3] = {0.1f, 0.0f, 0.01f};
float yaw_param[3] = {0.01f, 0.0f, 0.01f};

// === 控制指令封装接口（Protocol调用） ===
void apply_motion_command(const MotionCommand* mc)
{
    if (!mc) return;

    if (mc->mode == SLOW)
    {
        target_x = mc->x_thrust * SLOW_SENSI_X;
        target_y = mc->y_thrust * SLOW_SENSI_Y;
        target_yaw -= mc->yaw_thrust * YAW_SENSI * SLOW_SENSI_YAW;
        target_pitch += mc->pitch_thrust * PITCH_SENSI * SLOW_SENSI_PITCH;
        z_thrust = mc->z_thrust * SLOW_SENSI_DEEP;
    }
    if (mc->mode == FAST)
    {
        target_x = mc->x_thrust * FAST_SENSI_X;
        target_y = mc->y_thrust * FAST_SENSI_Y;
        target_yaw -= mc->yaw_thrust * YAW_SENSI * FAST_SENSI_YAW;
        target_pitch += mc->pitch_thrust * PITCH_SENSI * FAST_SENSI_PITCH;
        z_thrust = mc->z_thrust * FAST_SENSI_DEEP;
    }
    if (mc->mode == NORMAL)
    {
        target_x = mc->x_thrust;
        target_y = mc->y_thrust;
        target_yaw -= mc->yaw_thrust * YAW_SENSI;
        target_pitch += mc->pitch_thrust * PITCH_SENSI;
        z_thrust = mc->z_thrust;
    }

    if (target_pitch > 30)
    {
        target_pitch = 30;
    }
    if (target_pitch < -30)
    {
        target_pitch = -30;
    }

    if (target_yaw > 175)
    {
        target_yaw = 175;
    }
    if (target_yaw < -175)
    {
        target_yaw = -175;
    }
}

// 推力合成函数（带混控参数）
void compute_motor_output(float x, float y, float yaw,
                          float roll_out, float pitch_out, float z)
{
    float m[8] = {0}; // M1 ~ M8

    // 控制输入组成
    float input[6] = { x, y, yaw, roll_out, pitch_out, z };

    // 对每个电机执行混合计算
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 6; j++) {
            m[i] += input[j] * motor_mix_gain[i][j];
        }
    }

    // 限幅 [-1.0, 1.0]
    for (int i = 0; i < 7; i++) {
        if (m[i] > 1.0f) m[i] = 1.0f;
        if (m[i] < -1.0f) m[i] = -1.0f;
    }

    // M8 保持为 0（未使用）
    m[7] = 0.0f;

    // 输出到电机
    motor_output_all(m);
}

// // === 推力合成逻辑 ===
// void compute_motor_output(float x, float y, float yaw,
//                           float roll_out, float pitch_out, float z)
// {
//     float m[8] = {0}; // M1 ~ M8
//
//     // 水平电机（M1~M4）
//     m[0] = -x - y - yaw; // M1 - Back_Right
//     m[1] = -x - y + yaw; // M2 - Front_Right
//     m[2] = x + y - yaw;  // M3 - Front_Left
//     m[3] = -x + y + yaw; // M4 - Back_Left
//
//     // 垂直电机（M5~M7）
//     m[4] = z + pitch_out - roll_out;       // M5 - Middle_Right
//     m[5] = z + pitch_out + roll_out;       // M6 - Middle_Left
//     m[6] = -z + 2.0f * pitch_out;          // M7 - BackMiddle
//
//     // M8（未使用）默认为 0
//
//     // 限幅处理
//     for (int i = 0; i < 8; i++)
//     {
//         if (m[i] > 1.0f) m[i] = 1.0f;
//         if (m[i] < -1.0f) m[i] = -1.0f;
//     }
//
//     // 批量输出
//     motor_output_all(m);
// }

// === 控制任务 ===
void motion_task(void const* argument)
{
    motor_pwm_init();
    osDelay(100);
    PID_init(&pid_yaw, PID_POSITION, yaw_param, 0.5f, 0.2f);
    PID_init(&pid_roll, PID_POSITION, roll_param, 0.5f, 0.2f);
    PID_init(&pid_pitch, PID_POSITION, pitch_param, 0.5f, 0.2f);
    osDelay(200); // 等待IMU稳定

    while (1)
    {
        float yaw, pitch, roll;
        float roll_out = 0.0f, yaw_out = 0.0f, pitch_out = 0.0f;

        if (imu_get_euler(&yaw, &pitch, &roll))
        {
            roll_out = PID_calc(&pid_roll, roll, target_roll);
            pitch_out = PID_calc(&pid_pitch, pitch, target_pitch);
            yaw_out = PID_calc(&pid_yaw, yaw, target_yaw);
        }
        if (!is_yaw_pid)   yaw_out = 0.0f;
        if (!is_pitch_pid) pitch_out = 0.0f;
        if (!is_roll_pid)  roll_out = 0.0f;
        compute_motor_output(target_x, target_y, yaw_out,
                             roll_out, pitch_out, z_thrust);

        osDelay(10); // 100Hz 控制周期
    }
}
