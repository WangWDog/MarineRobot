//
// Created by 13033 on 2025/7/16.
//

#include "SysInfoUsart.h"

#include <stdbool.h>

#include "usart.h"
#include "imu_task.h"  // imu_get_euler()
#include "cmsis_os.h"
#include <string.h>
extern uint16_t  config_packet_count;
extern float target_yaw;
extern float target_pitch;
extern float target_roll;

bool uart_tx_busy = false;

void system_info_task(void *argument)
{
    static uint8_t tx_buf[32];  // 至少 30 字节

    for (;;)
    {
        if (!uart_tx_busy)
        {
            float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
            imu_get_euler(&yaw, &pitch, &roll);

            float payload[6] = {
                target_yaw,
                target_pitch,
                target_roll,
                yaw,
                pitch,
                roll
            };

            tx_buf[0] = 0xA5;
            tx_buf[1] = 0x08;      // TYPE
            tx_buf[2] = 26;        // ✅ Payload长度 = 6*float + 2字节 count

            memcpy(&tx_buf[3], payload, sizeof(payload));  // 24 字节 float

            uint16_t count = config_packet_count;
            tx_buf[27] = (uint8_t)(count & 0xFF);           // 低字节
            tx_buf[28] = (uint8_t)((count >> 8) & 0xFF);    // 高字节

            tx_buf[29] = 0x5A;  // TAIL

            uart_tx_busy = true;
            HAL_UART_Transmit_IT(&huart4, tx_buf, 30);  // ✅ 总长度 = 30
        }

        osDelay(500);  // 每 500ms 上报一次
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4)
    {
        uart_tx_busy = false;
    }
}
