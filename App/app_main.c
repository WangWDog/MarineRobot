//
// Created by 13,033 on 2025/7/12.
//

#include "app_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "imu_task.h"
#include "Motion.h"
#include "adc_task.h"
#include "sht30.h"
#include "uart4_rx_task.h"

void app_main_init()
{
}

void app_main_start()
{
    // xTaskCreate(imu_task, "ImuTask", 128, NULL, 3, NULL);
    // xTaskCreate(adc_task, "ADCTask", 128, NULL, 3, NULL);
    xTaskCreate(sht30_task, "SHT30Task", 128, NULL, 3, NULL);
    // xTaskCreate(motion_task, "MotionTask", 128, NULL, 3, NULL);
    // xTaskCreate(uart_rx_task, "UartRxTask",  256, NULL, 4, &UartRxTaskHandle);
}
