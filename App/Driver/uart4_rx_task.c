#include <string.h>
#include "uart4_rx_task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "protocol.h"  // 你定义 ControlFrame 的地方

#define UART_RX_BUF_SIZE 64
#define FRAME_LEN 8
#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0x55

static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
static uint8_t uart_rx_index = 0;
static uint8_t uart_rx_byte = 0;

TaskHandle_t UartRxTaskHandle = NULL;  // 通知用句柄


void uart_rx_task(void *argument)
{
    // 启动串口接收（1 字节）
    HAL_UART_Receive_IT(&huart4, &uart_rx_byte, 1);

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 解包滑窗
        while (uart_rx_index >= FRAME_LEN)
        {
            if (uart_rx_buf[0] == FRAME_HEAD && uart_rx_buf[7] == FRAME_TAIL)
            {
                uint8_t sum = 0;
                for (int i = 1; i <= 5; i++) sum ^= uart_rx_buf[i];

                if (sum == uart_rx_buf[6])
                {
                    ControlFrame cmd;
                    cmd.x_move = uart_rx_buf[1] / 255.0f;
                    cmd.y_move = uart_rx_buf[2] / 255.0f;
                    cmd.yaw    = uart_rx_buf[3] / 255.0f;
                    cmd.pitch  = uart_rx_buf[4] / 255.0f;
                    cmd.btn    = uart_rx_buf[5];

                    handle_control_command(&cmd);
                }

                // 移除一帧
                memmove(uart_rx_buf, uart_rx_buf + FRAME_LEN, uart_rx_index - FRAME_LEN);
                uart_rx_index -= FRAME_LEN;
            }
            else {
                // 移动1字节继续滑动
                memmove(uart_rx_buf, uart_rx_buf + 1, --uart_rx_index);
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
