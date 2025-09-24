/*
 * CanService.h
 *
 *  Created on: 2025年8月21日
 *      Author: mengfa3
 */

#ifndef SERVICE_CANSERVICE_H_
#define SERVICE_CANSERVICE_H_

#include "IntCtrl_Ip.h"
#include "CDD_Sbc_fs23.h"
#include "FreeRTOS.h"
#include "queue.h"

typedef enum
{
    CAN_OK,
    CAN_NOT_OK,
    CAN_BUSY
}Can_ReturnType;

/*! @brief CAN message queue format
 * Implements : can_message_t_Class
 */
typedef struct {
	uint32_t cs;        /*!< Code and Status*/
    uint32_t id;        /*!< ID of the message */
    uint8_t data[8];    /*!< Data bytes of the CAN message*/
    uint8_t length;     /*!< Length of payload in bytes */
    uint8_t ch;         /*!< channel of CAN */
} can_message_queue_t;

typedef struct
{
//	uint8_t CanChannel;
//	uint8_t swPduHandle;
	uint8_t FrameType;			//zjx0524
	uint8_t FrameFormat;		//zjx0524

	uint8_t length;
	uint16_t id;
	uint8_t data[8];
}Can_QueueMessage;

//增加帧格式，帧类型
typedef struct
{
	//	uint8_t swPduHandle;	//例程的，zjx0524
	uint8_t FrameType;			//zjx0524
	uint8_t FrameFormat;		//zjx0524

    uint8_t length;
    uint32_t id;
    uint8_t *sdu;
}Can_PduType;

/*! @brief CAN message format
 * Implements : can_message_t_Class
 */
typedef struct {
	uint32_t cs;       /*!< Code and Status*/
    uint32_t id;       /*!< ID of the message */
    uint8_t data[64];  /*!< Data bytes of the CAN message*/
    uint8_t length;    /*!< Length of payload in bytes */
} can_message_t;

extern QueueHandle_t xQueueCanTx;
extern QueueHandle_t xQueueCanRx;

void Spi_Init(void);
void CAN0_Init(void);
void CAN0_SetQueueHandle(QueueHandle_t xQueueTx, QueueHandle_t xQueueRx);
uint32_t CAN0_SendMsg(uint32_t messageId, uint8_t * data, uint32_t len);
Can_ReturnType Can_Write_Queue( Can_PduType* PduInfo );
void StartCanSendMsgTimer(void);
uint32_t CAN0_UDS_SendMsg(uint32_t messageId, uint8_t * data, uint32_t len);


extern int UJA1169_Init(void);
#endif /* SERVICE_CANSERVICE_H_ */
