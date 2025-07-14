//
// Created by Lenovo on 25-7-13.
//

#ifndef PID_TASK_H
#define PID_TASK_H

void PID_Task_Init(void);         // 任务初始化函数
void PID_Control_Task(void *argument);  // FreeRTOS 任务主体
void PID_task(void *argument);
#endif
