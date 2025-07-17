//
// Created by 13033 on 2025/7/16.
//

#include "parse_frame.h"

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "uart4_rx_task.h"

// TLV常量定义
#define MOTION_SWITCH_DEBOUNCE_MS 500

static uint32_t last_motion_toggle_time = 0;  // 上次切换时间（单位：ms）

// 外部参数变量声明（你已有的变量）
extern float motor_bias[7];

extern float motor_mix_gain[7][6];
extern bool is_motion_enabled;

extern float pitch_param[3];
extern float roll_param[3];
extern float yaw_param[3];

extern float PITCH_SENSI, YAW_SENSI;
extern float FAST_SENSI_X, FAST_SENSI_Y, FAST_SENSI_YAW, FAST_SENSI_PITCH, FAST_SENSI_DEEP;
extern float SLOW_SENSI_X, SLOW_SENSI_Y, SLOW_SENSI_YAW, SLOW_SENSI_PITCH, SLOW_SENSI_DEEP;

extern bool is_roll_pid;
extern bool is_pitch_pid;
extern bool is_yaw_pid;

extern float target_pitch;
extern float target_roll;
/**
 * @brief 解析一帧 TLV 协议数据（含头尾），并根据 Type 路由处理
 * @param frame    指向完整帧的指针（包含 HEAD、TYPE、LEN、DATA、TAIL）
 * @param len      帧总长度（包含帧头和帧尾）
 */
void parse_tlv_frame(const uint8_t* frame, uint8_t len)
{
    if (!frame || len < 5) return;

    if (frame[0] != TLV_FRAME_HEAD || frame[len - 1] != TLV_FRAME_TAIL) return;

    uint8_t type = frame[1];
    uint8_t length = frame[2];

    if (length != len - 4) return; // HEAD + TYPE + LEN + TAIL = 4 字节

    const uint8_t* payload = &frame[3];

    switch (type)
    {
    case 0x01: // PID 参数（pitch/roll/yaw 各3个 float）
        if (length == 36)
        {
            memcpy(pitch_param, &payload[0], 3 * sizeof(float));
            memcpy(roll_param, &payload[12], 3 * sizeof(float));
            memcpy(yaw_param, &payload[24], 3 * sizeof(float));
        }
        break;

    case 0x02: // 控制灵敏度参数（12个 float）
        if (length == 48)
        {
            const float* p = (const float*)payload;
            PITCH_SENSI = p[0];
            YAW_SENSI = p[1];
            FAST_SENSI_X = p[2];
            FAST_SENSI_Y = p[3];
            FAST_SENSI_YAW = p[4];
            FAST_SENSI_PITCH = p[5];
            FAST_SENSI_DEEP = p[6];
            SLOW_SENSI_X = p[7];
            SLOW_SENSI_Y = p[8];
            SLOW_SENSI_YAW = p[9];
            SLOW_SENSI_PITCH = p[10];
            SLOW_SENSI_DEEP = p[11];
        }
        break;

    case 0x03: // motor_bias（7个 float）
        if (length == 28)
        {
            memcpy(motor_bias, payload, 7 * sizeof(float));
        }
        break;

    case 0x04: // motor_mix_gain 全部矩阵（7x6 float）
        if (length == 168)
        {
            memcpy(motor_mix_gain, payload, 7 * 6 * sizeof(float));
        }
        break;

    case 0x06: // 单个电机的 motor_mix_gain（1字节 motor_id + 6个 float）
        if (length == 25)
        {
            uint8_t motor_id = payload[0];
            if (motor_id < 7)
            {
                memcpy(motor_mix_gain[motor_id], &payload[1], 6 * sizeof(float));
            }
        }
        break;

    case 0x07: // 设置姿态目标和 PID 开关
        if (length == 9)
        {
            // 2 floats + 1 uint8_t
            const float* p = (const float*)payload;
            target_pitch = p[0];
            target_roll = p[1];

            uint8_t flags = payload[8];
            is_roll_pid = (flags & 0x01) != 0;
            is_pitch_pid = (flags & 0x02) != 0;
            is_yaw_pid = (flags & 0x04) != 0;
        }
    case 0x08: // 系统启停控制（串口按钮模拟：按一下切换）
        if (length == 1 && payload[0] == 1)
        {
            uint32_t now = osKernelSysTick(); // 当前时间
            if (now - last_motion_toggle_time > MOTION_SWITCH_DEBOUNCE_MS)
            {
                is_motion_enabled = !is_motion_enabled; // 状态翻转
                last_motion_toggle_time = now; // 记录切换时间
            }
        }
        break;
    default:
        // 其他类型扩展
        break;
    }
}
