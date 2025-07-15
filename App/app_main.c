//
// Created by 13,033 on 2025/7/12.
//

#include "app_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "imu_task.h"
#include "Driver/motor.h"
#include "adc_task.h"
#include "sht30.h"
#include "uart4_rx_task.h"
#include "usart.h"

void app_main_init()
{
    // HAL_UART_Receive_DMA(&huart4, uart4_dma_buf, UART4_RX_BUFFER_SIZE);
    // __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
}

void app_main_start()
{
     xTaskCreate(imu_task, "ImuTask", 128, NULL, 3, NULL);
        // xTaskCreate(adc_task, "ADCTask", 128, NULL, 3, NULL);
        // xTaskCreate(sht30_task, "SHT30Task", 128, NULL, 3, NULL);
     xTaskCreate(motion_task, "MotionTask", 256, NULL, 3, NULL);
     xTaskCreate(uart_rx_task, "UartRxTask",  512, NULL, 5, &UartRxTaskHandle);
}
