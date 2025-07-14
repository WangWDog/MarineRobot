#ifndef __MOTION_H__
#define __MOTION_H__

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

    typedef enum {
        NORMAL = 0,
        FAST,
        SLOW
    } ControlMode;

    // RTOS任务入口
    void motion_task(void const * argument);

    // 控制指令接口（由 protocol 调用）
    void set_motion_xy(float x, float y);                      // 平移目标
    void set_attitude_yaw_pitch(float yaw, float pitch);       // 姿态目标
    void set_mode(ControlMode mode);                           // 模式切换
    void dive(void);                                            // 下潜
    void rise(void);                                            // 上浮

#ifdef __cplusplus
}
#endif

#endif
