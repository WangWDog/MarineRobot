#include <string.h>
#include "uart4_rx_task.h"

#include <stdbool.h>

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

        while (uart_rx_index >= 5) // 保留最小入口判断
        {
            bool matched = false;

            // TLV 帧头检测，仅在足够长才处理
            if (uart_rx_buf[0] == TLV_FRAME_HEAD)
            {
                // TLV 最少5字节才能提取 len
                if (uart_rx_index >= 3) {
                    uint8_t len = uart_rx_buf[2];
                    uint8_t full_len = len + 4;

                    // ⚠️ 如果长度不足，等待更多数据（不滑动）
                    if (uart_rx_index < full_len)
                        break;  // ❗️暂时退出，等待后续数据

                    // 长度满足再解析
                    if (full_len <= TLV_MAX_LEN &&
                        uart_rx_buf[full_len - 1] == TLV_FRAME_TAIL)
                    {
                        parse_tlv_frame(uart_rx_buf, full_len);
                        memmove(uart_rx_buf, uart_rx_buf + full_len, uart_rx_index - full_len);
                        uart_rx_index -= full_len;
                        matched = true;
                    }
                }
            }

            // === 匹配固定控制帧（8字节） ===
            else if (uart_rx_buf[0] == FRAME_HEAD)
            {
                // 确保有足够字节才能判断控制帧
                if (uart_rx_index < 8)
                    break;  // ⛔ 暂停处理，等待更多数据

                if (uart_rx_buf[7] == FRAME_TAIL)
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

                    memmove(uart_rx_buf, uart_rx_buf + 8, uart_rx_index - 8);
                    uart_rx_index -= 8;
                    matched = true;
                }
            }


            // === 无匹配，滑动窗口 ===
            if (!matched)
            {
                memmove(uart_rx_buf, uart_rx_buf + 1, uart_rx_index - 1);
                uart_rx_index -= 1;
            }
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
