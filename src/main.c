/*
===============================================================================
 Name        : main.c
 Author      : Konstantin
 Version     : 0.1
 Description : main definition
===============================================================================
*/

#include "env.h"
#include "type.h"

#include "A3984_module.h"
#include "gpio_module.h"
#include "can_module.h"

#include "usb.h"
#include "usbcfg.h"
#include "usbhw.h"
#include "usbcore.h"

void CAN_Message_Received(can_message_ msg);

int main(void)
{
	can_message_ msg;

	GPIO_init();

	CAN_init(125000, 0, 0, 8, &CAN_Message_Received);

	USB_Init();
	USB_Connect(TRUE);

	A3984_init();
	A3984_set_state(A3984_STATE_WAKEUP);
	A3984_set_state(A3984_STATE_ACTIVE);
	A3984_set_stepping(A3984_STEP_FULL);
	A3984_set_direction(A3984_DIR_CW);
	A3984_set_idle_current(100);

	GPIO_clock_out(TRUE);

	CAN_Message_(&msg, 0x0123, 0, 5, 'R', 'E', 'A', 'D', 'Y');
	CAN_send(1, msg);


	while (1)
	{
#if A3984_CURRENT_ISR == 1
		A3984_set_current_handler();
#endif
	}
	return 0 ;
}

void CAN_Message_Received(can_message_ msg)
{

}
