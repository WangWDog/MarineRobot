#ifndef __REV_HOST_H
#define __REV_HOST_H
#include "main.h"
#pragma pack (1)//�ֽڶ��룬�Է�����ʱ���ݴ�λ
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
