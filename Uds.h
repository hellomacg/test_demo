/*
 * Uds.h
 *
 *  Created on: 2025年9月1日
 *      Author: maode1
 */

#ifndef UDS_H_
#define UDS_H_

#include "stdlib.h"
#include "CanService.h"

#define TX_MB_INDEX (7)
#define PROGRAM_TIMER1 (0x0032)
#define PROGRAM_TIMER2 (0x01F4)
#define SF_MEM_BASE 0x10140000

typedef struct {
    uint32 cs;       /*!< Code and Status*/
    uint32 id;       /*!< ID of the message */
    uint8 data[8];  /*!< Data bytes of the CAN message*/
    uint8 length;    /*!< Length of payload in bytes */
} udscan_message_t;

typedef struct
{
    uint8 SingleOrMore;
    uint8 Number;
    uint16 Length;
    uint8 Service;
    uint8 data[150];
} CAN_Data;


extern struct spi_nor flash;
extern udscan_message_t recvMsg_CAN1;
extern uint8 IRQ_CAN0_RX;
extern uint16 usLPTCount;
extern uint8 gucMispassCount;/*密钥错误次数*/
extern uint8 FlowLost_time;
extern uint8 Time_Out_Flag;

extern uint8 handle1;

void dealwith10(uint8 *stic,uint8 *sticlock,uint8 AddressMode,CAN_Data pCAN_Data);
void dealwith11(uint8 stic,uint8 AddressMode,CAN_Data pCAN_Data);
void dealwith22(uint8 stic,uint8 AddressMode,CAN_Data pCAN_Data);
void flowadd(uint8 BS, uint8 STmin);
void dealwith27(uint8 stic,uint8 *sticlock,CAN_Data pCAN_Data);
void dealwith2E(uint8 stic,uint8 sticlock,CAN_Data pCAN_Data);
void dealwith31(uint8 stic,uint8 sticlock,CAN_Data pCAN_Data);
void dealwith34(uint8 stic,uint8 sticlock,CAN_Data pCAN_Data);
void dealwith36(uint8 stic,CAN_Data pCAN_Data);
void dealwith37(uint8 stic,CAN_Data pCAN_Data);
void dealwith3E(uint8 stic,uint8 AddressMode,CAN_Data pCAN_Data);
void dealwith3E_80(uint8 *hold_buf);
uint16 calcKey(uint16 seed);
void Delay_5s(uint8 mark);
//void APP2toAPP1();
unsigned short int CalcCRC16(uint32 size, uint32 data);
unsigned short int CalcCRC(uint32 size, uint32 data);

void MCU_Reset(void);
void Appl_GotoAppSW(uint32 address);/*PRQA S 3006*/
void PitRtiNotification(void);
void Diagnostic_MainProc(void);
#endif /* UDS_H_ */
