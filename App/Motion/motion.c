#include "motion.h"
#include "pid.h"
#include "imu_task.h"
#include "Motor.h"
#include "cmsis_os.h"
#include <string.h>
#include <math.h>

// ========== 控制目标 ==========

static float target_x = 0.0f;
static float target_y = 0.0f;
static float target_pitch = 0.0f;
static float target_yaw = 0.0f;
static float z_thrust = 0.0f;             // 上下推力（中间推进器用）
static ControlMode control_mode = NORMAL;

// ========== PID 控制器 ==========
pid_type_def pid_roll;
pid_type_def pid_pitch;
pid_type_def pid_yaw;

static const float roll_param[3]  = {1.2f, 0.0f, 0.08f};
static const float pitch_param[3] = {1.2f, 0.0f, 0.08f};
static const float yaw_param[3]   = {3.0f, 0.0f, 0.1f};

// ========== 控制接口 ==========
void set_motion_xy(float x, float y)
{
    if (x < -1.0f) x = -1.0f;
    if (x >  1.0f) x =  1.0f;
    if (y < -1.0f) y = -1.0f;
    if (y >  1.0f) y =  1.0f;
    target_x = x;
    target_y = y;
}

void set_attitude_yaw_pitch(float yaw, float pitch)
{
    if (yaw   < -180.0f) yaw   = -180.0f;
    if (yaw   >  180.0f) yaw   =  180.0f;
    if (pitch < -30.0f)  pitch = -30.0f;
    if (pitch >  30.0f)  pitch =  30.0f;
    target_yaw = yaw;
    target_pitch = pitch;
}

void set_mode(ControlMode mode)
{
    control_mode = mode;
}

void dive(void)
{
    z_thrust = 0.3f;  // 正值向下推
}

void rise(void)
{
    z_thrust = -0.3f; // 负值向上推
}

// ========== 电机输出计算 ==========
void compute_motor_output(MotorPWMCommand_t* cmd, float x, float y,
                          float roll_out, float pitch_out, float yaw_out)
{
    float motor_output[8] = {0};

    // 混控模型（可根据结构调整）：
    // 前后电机：影响 pitch
    // 左右电机：影响 roll
    // yaw：左右差速
    // 平移：前后/左右电机线性叠加
    //
    // 电机编号：
    // 1: Back_Right
    // 2: Front_Right
    // 3: Front_Left
    // 4: Back_Left
    // 5: Middle_Right (垂直)
    // 6: Middle_Left  (垂直)

    float roll_k  = 1.0f;
    float pitch_k = 1.0f;
    float yaw_k   = 1.0f;
    float move_k  = 50.0f;  // 平移增益

    motor_output[Back_Right - 1]  = -roll_k * roll_out - pitch_k * pitch_out - yaw_k * yaw_out + x * move_k - y * move_k;
    motor_output[Front_Right - 1] = -roll_k * roll_out + pitch_k * pitch_out - yaw_k * yaw_out + x * move_k + y * move_k;
    motor_output[Front_Left - 1]  =  roll_k * roll_out + pitch_k * pitch_out + yaw_k * yaw_out + x * move_k - y * move_k;
    motor_output[Back_Left - 1]   =  roll_k * roll_out - pitch_k * pitch_out + yaw_k * yaw_out + x * move_k + y * move_k;

    motor_output[Middle_Right - 1] = z_thrust * 100.0f;
    motor_output[Middle_Left  - 1] = z_thrust * 100.0f;

    // 映射并限幅 → PWM范围 0~255 中心127
    for (int i = 0; i < 8; i++)
    {
        int pwm = 127 + (int)motor_output[i];
        if (pwm > 255) pwm = 255;
        if (pwm < 0)   pwm = 0;
        cmd->motor_pwm[i] = pwm;
    }
}

// ========== RTOS控制主任务 ==========
void motion_task(void const * argument)
{
    motor_pwm_init();
    // PID_init(&pid_roll, PID_POSITION, roll_param, 30.0f, 5.0f);
    // PID_init(&pid_pitch, PID_POSITION, pitch_param, 30.0f, 5.0f);
    // PID_init(&pid_yaw, PID_POSITION, yaw_param, 50.0f, 10.0f);
    //
    while (1)
    {
    //     float yaw, pitch, roll;
    //     if (imu_get_euler(&yaw, &pitch, &roll))
    //     {
    //         float roll_out  = PID_calc(&pid_roll, roll, 0.0f);
    //         float pitch_out = PID_calc(&pid_pitch, pitch, target_pitch);
    //         float yaw_out   = Boat_Angle_PID_calc(&pid_yaw, yaw, target_yaw);
    //
    //         MotorPWMCommand_t pwm_cmd;
    //         compute_motor_output(&pwm_cmd, target_x, target_y, roll_out, pitch_out, yaw_out);
    //         motor_pwm_output(&pwm_cmd);
    //
    //         z_thrust = 0.0f;  // 每轮复位浮沉指令，防止持续下沉或上浮
    //     }
    //+
    // motor_driver(Middle_Right,20);
    // motor_driver(Middle_Left,20);
    // motor_driver(Back_Left,20);
    // motor_driver(Back_Right,20);
    // motor_driver(Front_Right,20);
    // motor_driver(Front_Left,20);
    // motor_driver(100,20);



    osDelay(10);  // 100Hz 控制周期
    }
}
