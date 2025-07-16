#include "protocol.h"
#include "motor.h"
#include <string.h>
#include "motion.h"


void protocol_parse(uint8_t* buf, uint16_t len)
{
    for (uint16_t offset = 0; offset + CONTROL_FRAME_LEN <= len; offset++)
    {
        if (buf[offset] != FRAME_HEAD || buf[offset + CONTROL_FRAME_LEN - 1] != FRAME_TAIL)
            continue;

        // 校验和
        uint8_t sum = 0;
        for (int j = 1; j <= 6; j++) sum ^= buf[offset + j];
        if (sum != buf[offset + 7]) continue;

        // === 构建控制帧（已按标准 0~255 协议处理） ===
        ControlFrame cmd;
        cmd.x_move = buf[offset + 1]; // 原始值 0~255，后面转换为 -1 ~ +1 推力
        cmd.y_move = buf[offset + 2];
        cmd.yaw = buf[offset + 3];
        cmd.pitch = buf[offset + 4]; // 原始值 0~255，后面映射为角度
        cmd.btn = buf[offset + 5]; // 按钮的值

        handle_control_command(&cmd); // 更新目标变量

        offset += CONTROL_FRAME_LEN - 1;
    }
}

void handle_control_command(ControlFrame* cmd)
{
    MotionCommand mc;
    // === 加入死区控制 ===
    if (cmd->x_move>122&&cmd->x_move<132)
    {
        cmd->x_move = 127;
    }
    if (cmd->y_move>122&&cmd->y_move<132)
    {
        cmd->y_move = 127;
    }
    if (cmd->yaw>122&&cmd->yaw<132)
    {
        cmd->yaw = 127;
    }
    if (cmd->pitch>122&&cmd->pitch<132)
    {
        cmd->pitch = 127;
    }
    // === 解析推力方向 ===
    mc.x_thrust = (cmd->x_move - 127.0f) / 128.0f; // 前后 [-127,128]
    // 死区控制 就是防止推杆未回正导致的控制响应
    mc.y_thrust = (cmd->y_move - 127.0f) / 128.0f; // 左右 [-127,128]
    mc.yaw_thrust = (cmd->yaw - 127.0f) / 128.0f; // 偏航  [-127,128]
    mc.pitch_thrust = (cmd->pitch - 127.0f) * (60.0f / 255.0f); // 映射到 -30~+30°
    mc.z_thrust = 0.0f;
    mc.mode = NORMAL;

    // === 按钮功能解析 ===
    if (cmd->btn & 0x01) mc.z_thrust = -0.5f; // 上浮
    if (cmd->btn & 0x02) mc.z_thrust = +0.5f; // 下潜
    if (cmd->btn & 0x04) mc.mode = FAST; // 加速
    if (cmd->btn & 0x08) mc.mode = SLOW; // 慢速

    // === 提交给 motion 控制系统 ===
    apply_motion_command(&mc);
}