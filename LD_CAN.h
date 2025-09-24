/*
 * LD_CAN.h
 *
 *  Created on: 2025Äê9ÔÂ2ÈÕ
 *      Author: mengfa3
 */

#ifndef SERVICE_LD_CAN_H_
#define SERVICE_LD_CAN_H_

#include "CanService.h"

struct CPM_RemoteDiag_S{
	uint8_t CPM_TotalFaultNum;
	struct{
		uint8_t CPM_FrameIndex:5;
		uint8_t CPM_SupplierNum:3;
	}X1;
	uint8_t CPM_FaultNum1H;
	struct{
		uint8_t CPM_FaultNum2H:4;
		uint8_t CPM_FaultNum1L:4;
	}X3;
	uint8_t CPM_FaultNum2L;
	uint8_t CPM_FaultNum3H;
	struct{
		uint8_t CPM_FaultNum4H:4;
		uint8_t CPM_FaultNum3L:4;
	}X6;
	uint8_t CPM_FaultNum4L;
	uint8_t CPM_FaultNum5H;
	struct{
		uint8_t CPM_FaultNum6H:4;
		uint8_t CPM_FaultNum5L:4;
	}X9;
	uint8_t CPM_FaultNum6L;
	uint8_t CPM_FaultNum7H;
	struct{
		uint8_t CPM_FaultNum8H:4;
		uint8_t CPM_FaultNum7L:4;
	}X12;
	uint8_t CPM_FaultNum8L;
	uint8_t CPM_FaultNum9H;
	struct{
		uint8_t CPM_FaultNum10H:4;
		uint8_t CPM_FaultNum9L:4;
	}X15;
	uint8_t CPM_FaultNum10L;
	uint8_t CPM_FaultNum11H;
	struct{
		uint8_t CPM_FaultNum12H:4;
		uint8_t CPM_FaultNum11L:4;
	}X18;
	uint8_t CPM_FaultNum12L;
	uint8_t CPM_FaultNum13H;
	struct{
		uint8_t CPM_FaultNum14H:4;
		uint8_t CPM_FaultNum13L:4;
	}X21;
	uint8_t CPM_FaultNum14L;
	uint8_t CPM_FaultNum15H;
	struct{
		uint8_t CPM_FaultNum16H:4;
		uint8_t CPM_FaultNum15L:4;
	}X24;
	uint8_t CPM_FaultNum16L;
	uint8_t CPM_FaultNum17H;
	struct{
		uint8_t CPM_FaultNum18H:4;
		uint8_t CPM_FaultNum17L:4;
	}X27;
	uint8_t CPM_FaultNum18L;
	uint8_t CPM_FaultNum19H;
	struct{
		uint8_t CPM_FaultNum20H:4;
		uint8_t CPM_FaultNum19L:4;
	}X30;
	uint8_t CPM_FaultNum20L;
};

struct CPM_Version_S{
	uint8_t CPM_SupplierCodeH;
	uint8_t CPM_SupplierCodeL;
	uint8_t HwVersion;
	uint8_t SwMajorVersion;
	uint8_t SwMinorVersion;
	uint8_t SwPatchVersion;
	uint8_t SwVersionYear;
	uint8_t SwVersionMonth;
	uint8_t SwVersionDay;
};

struct CPM_State_S{
	uint8_t CPM_Cell1Vol;
	uint8_t CPM_Cell2Vol;
	uint8_t CPM_Cell3Vol;
	uint8_t CPM_Cell4Vol;
	uint8_t CPM_Cell5Vol;
	uint8_t CPM_KL30Vol;
	uint8_t CPM_BoardTemp;
	uint8_t CPM_Current;
	struct{
		uint8_t CPM_RRUnlockFlag:2;
		uint8_t CPM_FRUnlockFlag:2;
		uint8_t CPM_RLUnlockFlag:2;
		uint8_t CPM_FLUnlockFlag:2;
	}X8;
	struct{
		uint8_t CPM_RLHandleFlag:2;
		uint8_t CPM_FLHandleFlag:2;
		uint8_t CPM_RRChdUnlockFlag:2;
		uint8_t CPM_RLChdUnlockFlag:2;
	}X9;
	struct{
		uint8_t CPM_Chg2St:1;
		uint8_t CPM_PrechgSt:1;
		uint8_t CPM_ChgSt:1;
		uint8_t CPM_ChargSt:1;
		uint8_t CPM_RRHandleFlag:2;
		uint8_t CPM_FRHandleFlag:2;
	}X10;
	struct{
		uint8_t Byte11:1;
		uint8_t CPM_ElecapacitorFault:2;
		uint8_t CPM_WorkCount:5;
	}X11;
	uint8_t CPM_ElecapacitorVol;
	uint8_t CPM_PWMVol;
	struct{
		uint8_t CPM_PWMSts:2;
		uint8_t CPM_MainLoopFlag:1;
		uint8_t CPM_BalCmd1:1;
		uint8_t CPM_BalCmd2:1;
		uint8_t CPM_BalCmd3:1;
		uint8_t CPM_BalCmd4:1;
		uint8_t CPM_BalCmd5:1;
	}X14;
	uint8_t CPM_WakeupSource;
	struct{
		uint8_t Byte16:4;
		uint8_t CPM_LockIndex:4;
	}X16;
	uint8_t CPM_OddNumCurr1;
	uint8_t CPM_OddNumCurr2;
	uint8_t CPM_OddNumCurr3;
	uint8_t CPM_OddNumCurr4;
	uint8_t CPM_OddNumCurr5;
	uint8_t CPM_EvenNumCurr1;
	uint8_t CPM_EvenNumCurr2;
	uint8_t CPM_EvenNumCurr3;
	uint8_t CPM_EvenNumCurr4;
	uint8_t CPM_EvenNumCurr5;
	struct{
		uint8_t SBC_LDOSts:6;
		uint8_t SBC_InitSts:1;
		uint8_t Byte27:1;
	}X27;
	uint8_t SBC_WdgErrCounter;
	uint8_t CPM_MCUSMSts;
};

extern struct CPM_RemoteDiag_S CPM_RemoteDiag;
extern struct CPM_Version_S CPM_Version;
extern struct CPM_State_S CPM_State;

void Sys_CAN_Send_Message(void);

#endif /* SERVICE_LD_CAN_H_ */
