#ifndef __MOTION_H__
#define __MOTION_H__

typedef enum
{
    FAST = 1,
    NORMAL = 2,
    SLOW = 3
} Mode ;
typedef struct
{
    float x_thrust; // -1.0 ~ +1.0
    float y_thrust; // -1.0 ~ +1.0
    float yaw_thrust; // -1.0 ~ +1.0
    float pitch_thrust; // -1.0 ~ +1.0 degrees
    float z_thrust; // -1.0 ~ +1.0 （直接推力）
    Mode mode; // 模式增益（1.0 正常，1.5 加速，0.5 慢速）
} MotionCommand;
// RTOS任务入口
void motion_task(void const* argument);
// 暴露接口 修改外部的函数
void apply_motion_command(const MotionCommand* mc);

// 控制指令接口（由 protocol 调用）
// void set_motion_xy(float x, float y);                      // 平移目标
// void set_attitude_yaw_pitch(float yaw, float pitch);       // 姿态目标
// void set_mode(ControlMode mode);                           // 模式切换
// void dive(void);                                            // 下潜
// void rise(void);                                            // 上浮

#endif
