/*
 * LD_CAN.c
 *
 *  Created on: 2025Äê9ÔÂ2ÈÕ
 *      Author: mengfa3
 */

#include "LD_CAN.h"
#include "pit_timer.h"

Can_PduType CPM_RemoteDiag_Message;
Can_PduType CPM_Version_Message;
Can_PduType CPM_State_Message;
struct CPM_RemoteDiag_S CPM_RemoteDiag;
struct CPM_Version_S CPM_Version;
struct CPM_State_S CPM_State;

void Sys_CAN_Send_Message(void)
{
	if(cnt_500ms > 500)
	{
		CPM_RemoteDiag_Message.id=0x6F3;
		CPM_RemoteDiag_Message.length=32;
		CPM_RemoteDiag_Message.sdu=(uint8_t *)&CPM_RemoteDiag;
		CAN0_SendMsg(0x6F3, (uint8_t *)&CPM_RemoteDiag, 32);
//		Can_Write_Queue(&CPM_RemoteDiag_Message);
		cnt_500ms = 0;
	}

	if(cnt_1000ms > 1000)
	{
		CPM_Version_Message.id=0x6F2;
		CPM_Version_Message.length=32;
		CPM_Version_Message.sdu=(uint8_t *)&CPM_Version;
		CAN0_SendMsg(0x6F2, (uint8_t *)&CPM_Version, 32);
//		Can_Write_Queue(&CPM_Version_Message);
		cnt_1000ms = 0;
	}

	if(cnt_50ms > 50)
	{
		CPM_State_Message.id=0x3FB;
		CPM_State_Message.length=32;
		CPM_State_Message.sdu=(uint8_t *)&CPM_State;
		CAN0_SendMsg(0x3FB, (uint8_t *)&CPM_State, 32);
//		Can_Write_Queue(&CPM_State_Message);
		cnt_50ms = 0;
	}
}
