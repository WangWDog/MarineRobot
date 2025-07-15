#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>

// 假设 CONTROL_FRAME_LEN 和其他宏定义如下
#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0x55
#define CONTROL_FRAME_LEN 8  // 每个数据包的长度
#define MAX_RETRIES 5  // 最大重试次数

// 控制数据结构体
typedef struct {
    float x_move;   // 摇杆0 - 平动控制 X (0~1)
    float y_move;   // 摇杆0 - 平动控制 Y
    float yaw;      // 摇杆1 - 姿态控制 yaw (0~1)
    float pitch;    // 摇杆1 - 姿态控制 pitch (0~1)
    uint8_t btn;    // 按键状态位（bit0~bit3）
} ControlFrame;

void protocol_parse(uint8_t *buf, uint16_t len);        // 解析原始数据流
void handle_control_command(ControlFrame *cmd);         // 响应控制帧

#endif
