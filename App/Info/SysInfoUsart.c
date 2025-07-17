//
// Created by 13033 on 2025/7/16.
//

#include "SysInfoUsart.h"

#include <stdbool.h>

#include "usart.h"
#include "imu_task.h"  // imu_get_euler()
#include "cmsis_os.h"
#include <string.h>

extern float target_yaw;
extern float target_pitch;
extern float target_roll;

bool uart_tx_busy = false;


void system_info_task(void *argument)
{
    static uint8_t tx_buf[32];  // 最大 32 字节足够

    for (;;)
    {
        // 若上一次发送未完成则跳过本轮（防止重入）
        if (uart_tx_busy == false)
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

            tx_buf[0] = 0xA5;              // HEAD
            tx_buf[1] = 0x08;              // TYPE
            tx_buf[2] = 24;                // LENGTH
            memcpy(&tx_buf[3], payload, sizeof(payload));
            tx_buf[27] = 0x5A;             // TAIL

            uart_tx_busy = true;
            HAL_UART_Transmit_IT(&huart4, tx_buf, 28);
        }

        osDelay(500);  // 每 200ms 尝试发送一次
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4)
    {
        uart_tx_busy = false;
    }
}
