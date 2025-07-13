#include "usart.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "uart4_rx_task.h"
#define UART4_RX_BUFFER_SIZE 128

static uint8_t uart4_dma_buf[UART4_RX_BUFFER_SIZE];   // DMA接收区
static uint8_t uart4_proc_buf[UART4_RX_BUFFER_SIZE];  // 处理区
static uint16_t uart4_last_pos = 0;

TaskHandle_t UartRxTaskHandle = NULL;

void uart_rx_task(void *argument)
{
    // 启动DMA接收
    HAL_UART_Receive_DMA(&huart4, uart4_dma_buf, UART4_RX_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);  // 开启空闲中断

    for (;;) {
        // 等待接收中断通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint16_t curr_pos = UART4_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx);
        uint16_t data_len = (curr_pos >= uart4_last_pos)
                            ? (curr_pos - uart4_last_pos)
                            : (UART4_RX_BUFFER_SIZE - uart4_last_pos + curr_pos);

        // 拷贝新接收数据
        for (uint16_t i = 0; i < data_len; i++) {
            uart4_proc_buf[i] = uart4_dma_buf[(uart4_last_pos + i) % UART4_RX_BUFFER_SIZE];
        }

        uart4_last_pos = curr_pos;

        // 调用协议解析函数
        protocol_parse(uart4_proc_buf, data_len);
    }
}
