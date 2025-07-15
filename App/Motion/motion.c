// motion.c - 主控制逻辑
#include "motion.h"
#include "pid.h"
#include "imu_task.h"
#include "motor.h"
#include "cmsis_os.h"
#include <math.h>

#include "protocol.h"

// === 控制目标变量（静态封装） ===
static float target_x = 0.0f; // 前进 / 后退
static float target_y = 0.0f; // 横移
static float yaw_torque = 0.0f; // 偏航差速（开环）
static float target_pitch = 0.0f; // 俯仰角目标（度）
static float target_roll = 0.0f; // 横滚角目标（暂设为0）
static float z_thrust = 0.0f; // 上浮 / 下潜推力（-1.0~+1.0）

// === PID 控制器 ===
static pid_type_def pid_roll;
static pid_type_def pid_pitch;

static const float roll_param[3] = {0.1f, 0.0f, 0.01f};
static const float pitch_param[3] = {0.1f, 0.0f, 0.01f};

// === 控制指令封装接口（Protocol调用） ===
void apply_motion_command(const MotionCommand* mc)
{
    if (!mc) return;

    // target_x = mc->x_thrust * mc->thrust_scale;
    // target_y = mc->y_thrust * mc->thrust_scale;
    // yaw_torque = mc->yaw_thrust * mc->thrust_scale;

    target_x = mc->x_thrust;
    target_y = mc->y_thrust;
    yaw_torque = mc->yaw_thrust;
    target_pitch = mc->pitch_angle;
    z_thrust = mc->z_thrust * mc->thrust_scale;
}

// === 推力合成逻辑 ===
void compute_motor_output(float x, float y, float yaw,
                          float roll_out, float pitch_out, float z)
{
    float m[8] = {0}; // M1 ~ M8

    // 水平电机（M1~M4）
    m[0] = x - y - yaw; // M1 - Back_Right
    m[1] = x + y + yaw; // M2 - Front_Right
    m[2] = x + y - yaw; // M3 - Front_Left
    m[3] = x - y + yaw; // M4 - Back_Left

    // 垂直电机（M5~M7）
    m[4] = z + pitch_out - roll_out; // M5 - Middle_Right
    m[5] = z + pitch_out + roll_out; // M6 - Middle_Left
    m[6] = z - 2.0f * pitch_out; // M7 - BackMiddle

    // M8（未使用）默认为 0

    // 限幅处理
    for (int i = 0; i < 8; i++)
    {
        if (m[i] > 1.0f) m[i] = 1.0f;
        if (m[i] < -1.0f) m[i] = -1.0f;
    }

    // 批量输出
    motor_output_all(m);
}

// === 控制任务 ===
void motion_task(void const* argument)
{
    motor_pwm_init();
    osDelay(100);
    PID_init(&pid_roll, PID_POSITION, roll_param, 0.5f, 0.2f);
    PID_init(&pid_pitch, PID_POSITION, pitch_param, 0.5f, 0.2f);
    osDelay(200); // 等待IMU稳定

    while (1)
    {
        float yaw, pitch, roll;
        float roll_out = 0.0f, pitch_out = 0.0f;

        if (imu_get_euler(&yaw, &pitch, &roll))
        {
            roll_out = PID_calc(&pid_roll, roll, target_roll);
            pitch_out = PID_calc(&pid_pitch, pitch, target_pitch);
        }

        compute_motor_output(target_x, target_y, yaw_torque,
                             roll_out, pitch_out, z_thrust);

        osDelay(10); // 100Hz 控制周期
    }
}
