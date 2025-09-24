/*
 * CanService.c
 *
 *  Created on: 2025年8月21日
 *      Author: mengfa3
 */

#include "CanService.h"
#include "Lpspi_Ip.h"
#include "FlexCAN_Ip.h"
#include "timers.h"
#include "string.h"
#include "uds.h"

#define MASTER_EXTERNAL_DEVICE     (Lpspi_Ip_DeviceAttributes_SpiExternalDevice_0_Instance_0)
#define MSG_ID 20u
#define RX_FUN_ID		0x7DF
#define RX_PHY_ID		0x671
#define RX_MB_FUN 3U
#define RX_MB_PHY 2U
#define RX_MB_IDX 1U
#define TX_MB_IDX 0U
uint8 IRQ_CAN0_RX  = 0;
udscan_message_t recvMsg_CAN1;
Flexcan_Ip_MsgBuffType rxData;
Flexcan_Ip_DataInfoType rx_info = {
		.msg_id_type = FLEXCAN_MSG_ID_STD,
		.data_length = 32u,
		.is_polling = FALSE,
		.is_remote = FALSE,
		.fd_enable = TRUE,
		.fd_padding = FALSE
};
TimerHandle_t xCanSendTimer = NULL;

/* The queue used by CAN tasks. */
QueueHandle_t xQueueCanTx = NULL;
QueueHandle_t xQueueCanRx = NULL;

extern void CAN0_ORED_0_31_MB_IRQHandler(void);

void Spi_Init(void)
{
	/* 初始化SPI 和IIC*/
	Lpspi_Ip_Init(&Lpspi_Ip_PhyUnitConfig_SpiPhyUnit_0_Instance_0);

	/* 定义SPI帧大小为8Bytes */
	Lpspi_Ip_UpdateFrameSize(&MASTER_EXTERNAL_DEVICE, 8U);

	/* 定义SPI为MSB模式 */
	Lpspi_Ip_UpdateLsb(&MASTER_EXTERNAL_DEVICE, FALSE);

//	/* 初始化 Sbc driver */
//	Sbc_fs23_InitDriver(NULL_PTR);
//	// 关闭FS23看门狗
//	Sbc_fs23_InitDevice();
	int status = 1;
	status = UJA1169_Init();
	if(status != 0)
	{

	}
}

void CAN0_Init(void)
{
	Flexcan_Ip_StatusType freeze_status;
	IntCtrl_Ip_EnableIrq(FlexCAN0_1_IRQn);
	IntCtrl_Ip_InstallHandler(FlexCAN0_1_IRQn, CAN0_ORED_0_31_MB_IRQHandler, NULL_PTR);

	FlexCAN_Ip_Init(INST_FLEXCAN_0, &FlexCAN_State0, &FlexCAN_Config0);

	freeze_status = FlexCAN_Ip_EnterFreezeMode_Privileged(INST_FLEXCAN_0);
	if(freeze_status == FLEXCAN_STATUS_SUCCESS)
	{
		//设置filter类型为独立filter
		FlexCAN_Ip_SetRxMaskType_Privileged(INST_FLEXCAN_0,FLEXCAN_RX_MASK_INDIVIDUAL);
		//为单个MB设置filter
		FlexCAN_Ip_SetRxIndividualMask_Privileged(INST_FLEXCAN_0,RX_MB_IDX,(0x7fe << 18));
		FlexCAN_Ip_SetRxIndividualMask_Privileged(INST_FLEXCAN_0,RX_MB_PHY,(0x7fe << 18));
		FlexCAN_Ip_SetRxIndividualMask_Privileged(INST_FLEXCAN_0,RX_MB_FUN,(0x7fe << 18));
	}
	FlexCAN_Ip_ExitFreezeMode_Privileged(INST_FLEXCAN_0);


	FlexCAN_Ip_ConfigRxMb(INST_FLEXCAN_0, RX_MB_IDX, &rx_info, MSG_ID);
	FlexCAN_Ip_ConfigRxMb(INST_FLEXCAN_0, RX_MB_PHY, &rx_info, RX_PHY_ID);
	FlexCAN_Ip_ConfigRxMb(INST_FLEXCAN_0, RX_MB_FUN, &rx_info, RX_FUN_ID);
	FlexCAN_Ip_SetStartMode(INST_FLEXCAN_0);
	FlexCAN_Ip_Receive(INST_FLEXCAN_0, RX_MB_IDX, &rxData, FALSE);
	FlexCAN_Ip_Receive(INST_FLEXCAN_0, RX_MB_PHY, &rxData, FALSE);
	FlexCAN_Ip_Receive(INST_FLEXCAN_0, RX_MB_FUN, &rxData, FALSE);
}

void CAN0_SetQueueHandle(QueueHandle_t xQueueTx, QueueHandle_t xQueueRx)
{
    xQueueCanTx = xQueueTx;
    xQueueCanRx = xQueueRx;
}
uint32 RxData_id = 0;
void flexcan0__Callback(uint8_t instance, Flexcan_Ip_EventType eventType,uint32 buffIdx, Flexcan_Ip_StateType * flexcanState)
{
	(void)flexcanState;
	(void)instance;
	(void) buffIdx;
	can_message_queue_t DataBuff = {0x00};

	switch(eventType)
	{
		case FLEXCAN_EVENT_RX_COMPLETE:
		FlexCAN_Ip_Receive(INST_FLEXCAN_0,buffIdx,&rxData,false);
		if(RX_MB_IDX == buffIdx)
		{
			//FlexCAN_Ip_Receive(INST_FLEXCAN_0,RX_MB_IDX,&rxData,false);

			if (xQueueCanRx != NULL)
			{
				//DataBuff.cs = recvMsg_CAN0.cs;
				DataBuff.id = rxData.msgId;
				DataBuff.length = rxData.dataLen;
				//DataBuff.ch = 0x00;
				memcpy(DataBuff.data, rxData.data, rxData.dataLen);

				xQueueSendFromISR(xQueueCanRx, &DataBuff,( TickType_t)0);
			}
//	    	FlexCAN_Ip_Send(INST_FLEXCAN_0, TX_MB_IDX, &rx_info, Tx_MSG_ID, (uint8 *)&rxData.data);
//
//			while(FlexCAN_Ip_GetTransferStatus(INST_FLEXCAN_0, TX_MB_IDX) != FLEXCAN_STATUS_SUCCESS)
//			{
//				FlexCAN_Ip_MainFunctionWrite(INST_FLEXCAN_0, TX_MB_IDX);
//			}
		}
		else if((RX_MB_PHY == buffIdx)||(RX_MB_FUN == buffIdx))
		{
	        if(IRQ_CAN0_RX==0 && (rxData.msgId==0x671 || rxData.msgId==0x7DF))
	        {
	            recvMsg_CAN1.id = rxData.msgId;
	            memcpy(recvMsg_CAN1.data, rxData.data, 8);
	            recvMsg_CAN1.length = rxData.dataLen;
	            IRQ_CAN0_RX = 1;
	        }
		}
		break;

		case FLEXCAN_EVENT_TX_COMPLETE:
		  break;

		default:
			break;
	}
}

uint32_t CAN0_SendMsg(uint32_t messageId, uint8_t * data, uint32_t len)
{
    static can_message_t Tx_msg = {0x00};

    Tx_msg.id     = messageId;
    Tx_msg.length = len;

    memcpy(Tx_msg.data, data, len);

	//status = CAN_Send(&can_instance0, TX_MAILBOX_CAN0, &Tx_msg);
    //LogPrintf("%X ", Tx_msg.id);
    FlexCAN_Ip_Send(INST_FLEXCAN_0, TX_MB_IDX, &rx_info, Tx_msg.id, Tx_msg.data);
    //while(FlexCAN_Ip_GetTransferStatus(INST_FLEXCAN_0, TX_MB_IDX) != FLEXCAN_STATUS_SUCCESS);

    return 0x00;
}
uint32_t CAN0_UDS_SendMsg(uint32_t messageId, uint8_t * data, uint32_t len)
{
    static can_message_t Tx_msg = {0x00};
    Flexcan_Ip_DataInfoType tx_info = {
    		.msg_id_type = FLEXCAN_MSG_ID_STD,
    		.data_length = 8u,
    		.is_polling = FALSE,
    		.is_remote = FALSE,
    		.fd_enable = FALSE,
    		.fd_padding = FALSE
    };
    Tx_msg.id     = messageId;
    Tx_msg.length = len;

    memcpy(Tx_msg.data, data, len);

	//status = CAN_Send(&can_instance0, TX_MAILBOX_CAN0, &Tx_msg);
    //LogPrintf("%X ", Tx_msg.id);
    FlexCAN_Ip_Send(INST_FLEXCAN_0, TX_MB_IDX, &tx_info, Tx_msg.id, Tx_msg.data);
    //while(FlexCAN_Ip_GetTransferStatus(INST_FLEXCAN_0, TX_MB_IDX) != FLEXCAN_STATUS_SUCCESS);

    return 0x00;
}
void StartCanSendMsgTimer(void)
{
    if (xCanSendTimer != NULL)
    {
        xTimerStart(xCanSendTimer, 0);
    }

    return;
}

/*============================================================================

	zjx0520
 *Can_ReturnType Can_Write_Queue(CAN_SFRmap* CANx, Can_PduType* PduInfo )
------------------------------------------------------------------------------
Describe   : Transmit the data into CAN queue
Input 	   : 1.CANx
			 2.PduInfo
Return     : CAN_BUSY/CAN_OK/CAN_NOT_OK
============================================================================*/
Can_ReturnType Can_Write_Queue( Can_PduType* PduInfo )
{
	Can_ReturnType eReturnValue = CAN_NOT_OK;
	Can_QueueMessage QueueMessage;
//	uint8_t i;

    /*transmit data to QueueMessage*/
    QueueMessage.id = PduInfo->id;
    QueueMessage.length = PduInfo->length;
   // QueueMessage.swPduHandle = PduInfo->swPduHandle;		//zjx0520

    for(uint8_t i=0;i<PduInfo->length;i++)
    {
     	QueueMessage.data[i] = PduInfo->sdu[i];
    }

    /*transmit QueueMessage into queue*/
    if(xQueueCanTx!=0)
    {
    	if (xQueueSend(xQueueCanTx,&QueueMessage,( TickType_t)0)==pdPASS)
    	{
    		eReturnValue=CAN_OK;
    	}
    	else
    	{
    		eReturnValue = CAN_BUSY;
    	}
    }
    return eReturnValue;
}
