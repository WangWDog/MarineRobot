#include "protocol.h"
#include "../Driver/Motor.h"

#include <string.h>

#include "motion.h"
#include "usart.h"



void protocol_parse(uint8_t *buf, uint16_t len) {
    // 调试信息
    char a = 'd';  // 用于调试
    HAL_UART_Transmit(&huart4, (uint8_t*)&a, sizeof(a), HAL_MAX_DELAY); // 发送调试字符

    // 解析所有可能的帧（支持粘包/多帧）
    for (uint16_t offset = 0; offset + CONTROL_FRAME_LEN <= len; offset++) {
        // 帧头和帧尾校验
        if (buf[offset] != FRAME_HEAD || buf[offset + CONTROL_FRAME_LEN - 1] != FRAME_TAIL) continue;

        // 校验和验证
        uint8_t sum = 0;
        for (int j = 1; j <= 6; j++) sum ^= buf[offset + j]; // 校验和计算
        if (sum != buf[offset + 7]) continue;

        // 有效帧 → 构建结构体
        ControlFrame cmd;
        cmd.x_move = buf[offset + 1] / 255.0f;
        cmd.y_move = buf[offset + 2] / 255.0f;
        cmd.yaw    = buf[offset + 3] / 255.0f;
        cmd.pitch  = buf[offset + 4] / 255.0f;
        cmd.btn    = buf[offset + 5];

        // 调用控制命令处理函数
        handle_control_command(&cmd);

        // 继续检查后续帧（支持多帧连续）
        offset += CONTROL_FRAME_LEN - 1;
    }
}

void handle_control_command(ControlFrame *cmd) {
    // // 平移控制
    // set_motion_xy(cmd->x_move, cmd->y_move);
    //
    // // 姿态控制
    // set_attitude_yaw_pitch(cmd->yaw, cmd->pitch);
    //
    // 上浮/下潜控制
    if (cmd->btn & 0x01) dive();   // btn1
    if (cmd->btn & 0x02) rise();   // btn2
    //
    // // 模式切换
    // if (cmd->btn & 0x04) {
    //     set_mode(FAST);            // btn3
    // } else if (cmd->btn & 0x08) {
    //     set_mode(SLOW);            // btn4
    // } else {
    //     set_mode(NORMAL);          // 默认模式
    // }
}
