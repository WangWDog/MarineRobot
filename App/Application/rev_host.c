#include "rev_host.h"
#include "string.h"
gamepad_info joystick;

void rev_gampad_info(gamepad_info *joystick,uint8_t *data,uint8_t length)
{
	if (data[0]==0xFA&&data[1]==0xAF)
	{
		if(data[length-1]==0xBF&&data[length-2]==0xFB)
		{
			memcpy(joystick,data+2,length);
		}
	}
}
