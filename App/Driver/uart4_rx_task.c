#include <string.h>
#include "uart4_rx_task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "parse_frame.h"
#include "protocol.h"  // 你定义 ControlFrame 的地方

#define UART_RX_BUF_SIZE 128
#define FRAME_LEN 8
#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0x55
#define TLV_MAX_LEN 80  // 允许最大TLV帧长度


static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
static uint8_t uart_rx_index = 0;
static uint8_t uart_rx_byte = 0;

TaskHandle_t UartRxTaskHandle = NULL;  // 通知用句柄



void uart_rx_task(void *argument)
{
    HAL_UART_Receive_IT(&huart4, &uart_rx_byte, 1);

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (uart_rx_index >= FRAME_LEN)
        {
            // === 原控制帧识别（固定8字节）
            if (uart_rx_buf[0] == FRAME_HEAD && uart_rx_buf[7] == FRAME_TAIL)
            {
                uint8_t sum = 0;
                for (int i = 1; i <= 5; i++) sum ^= uart_rx_buf[i];
                if (sum == uart_rx_buf[6])
                {
                    ControlFrame cmd_raw;
                    cmd_raw.x_move = uart_rx_buf[1];
                    cmd_raw.y_move = uart_rx_buf[2];
                    cmd_raw.yaw    = uart_rx_buf[3];
                    cmd_raw.pitch  = uart_rx_buf[4];
                    cmd_raw.btn    = uart_rx_buf[5];
                    handle_control_command(&cmd_raw);
                }
                memmove(uart_rx_buf, uart_rx_buf + FRAME_LEN, uart_rx_index - FRAME_LEN);
                uart_rx_index -= FRAME_LEN;
                continue;
            }

            // === TLV 参数帧识别（可变长，至少5字节）
            if (uart_rx_buf[0] == TLV_FRAME_HEAD && uart_rx_index >= 5)
            {
                uint8_t type = uart_rx_buf[1];
                uint8_t len  = uart_rx_buf[2];
                uint8_t full_len = len + 4; // HEAD + TYPE + LEN + TAIL

                // ✅ 加入边界保护：防止越界、帧伪造
                if (full_len <= uart_rx_index &&
                    full_len <= TLV_MAX_LEN &&
                    uart_rx_buf[full_len - 1] == TLV_FRAME_TAIL)
                {
                    parse_tlv_frame(uart_rx_buf, full_len);
                    memmove(uart_rx_buf, uart_rx_buf + full_len, uart_rx_index - full_len);
                    uart_rx_index -= full_len;
                    continue;
                }
            }

            // 否则滑窗继续移动
            memmove(uart_rx_buf, uart_rx_buf + 1, --uart_rx_index);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4)
    {
        if (uart_rx_index < UART_RX_BUF_SIZE)
            uart_rx_buf[uart_rx_index++] = uart_rx_byte;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(UartRxTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        HAL_UART_Receive_IT(&huart4, &uart_rx_byte, 1);
    }
}
