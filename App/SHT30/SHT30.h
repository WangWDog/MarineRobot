//
// Created by 13033 on 2025/7/12.
//

#ifndef SHT30_H
#define SHT30_H
#include "stm32h7xx_hal.h"

typedef struct
{
    float
 humidity;
} SHT30_Data;

extern
 SHT30_Data sht30_data;

void SHT30_Init(I2C_HandleTypeDef *hi2c)
;
uint8_t SHT30_Read(SHT30_Data *data)
;
#endif //SHT30_H
