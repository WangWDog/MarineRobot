#ifndef __REV_HOST_H
#define __REV_HOST_H
#include "main.h"
#pragma pack (1)//字节对齐，以防接收时数据错位
typedef struct
{
	uint8_t direction;
	uint8_t function;
	uint8_t select_start;
}gamepad_info;
#pragma pack ()
extern gamepad_info joystick;

void rev_gampad_info(gamepad_info *joystick,uint8_t *data,uint8_t length);

#endif
