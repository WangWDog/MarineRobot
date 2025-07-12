//
// Created by 13033 on 2025/7/12.
//

#include "app_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "imu_task.h"
#include "Motion.h"

void app_main_init()
{
}

void app_main_start()
{
    xTaskCreate(imu_task, "ImuTask", 128, NULL, 3, NULL);
    xTaskCreate(motion_task, "MotionTask", 128, NULL, 3, NULL);
}
