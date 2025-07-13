//
// Created by 13033 on 2025/7/13.
//
#ifndef __UART4_RX_TASK_H__
#define __UART4_RX_TASK_H__

#include "cmsis_os.h"

extern TaskHandle_t UartRxTaskHandle;
void uart_rx_task(void *argument);

#endif