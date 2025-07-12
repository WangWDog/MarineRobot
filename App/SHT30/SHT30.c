//
// Created by 13033 on 2025/7/12.
//

#include "SHT30.h"
#include "main.h"

#define SHT30_ADDRESS          (0x44 << 1)  // I2C地址左移一位
#define SHT30_CMD_MEASURE      0x2C06       // 高重复性测量命令

I2C_HandleTypeDef *sht30_i2c;
SHT30_Data sht30_data;

void SHT30_Init(I2C_HandleTypeDef *hi2c)
{
    sht30_i2c = hi2c;
}

uint8_t SHT30_Read(SHT30_Data *data)
{
    uint8_t cmd[2] = {0x2C, 0x06
};
    uint8_t recv[6
];

    if (HAL_I2C_Master_Transmit(sht30_i2c, SHT30_ADDRESS, cmd, 2
, HAL_MAX_DELAY) != HAL_OK)
        return 0
;

    HAL_Delay(
15);  // 等待测量完成

    if (HAL_I2C_Master_Receive(sht30_i2c, SHT30_ADDRESS, recv, 6
, HAL_MAX_DELAY) != HAL_OK)
        return 0
;

    uint16_t raw_hum  = (recv[3] << 8) | recv[4
];
    data->humidity =
100 * ((float)raw_hum / 65535.0
);

    return 1
;
}
