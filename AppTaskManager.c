/*
 * AppTaskManager.c
 *
 *  Created on: 2025��8��21��
 *      Author: mengfa3
 */
#include "AppTaskManager.h"
#include "LD_CAN.h"
#include "Siul2_Port_Ip.h"
#include "Siul2_Dio_Ip.h"
#include "semphr.h"

#define App_TASK_PRIORITY                ( tskIDLE_PRIORITY + 4 )
#define Send_TASK_PRIORITY               ( tskIDLE_PRIORITY + 2 )
#define Rec_TASK_PRIORITY                ( tskIDLE_PRIORITY + 3 )
#define Uds_TASK_PRIORITY                ( tskIDLE_PRIORITY + 1 )
/* The queue used by CAN tasks. */
uint8_t isCutoff = 0;
uint8_t CRASH_MCU_Old;
uint8_t isCrash = 0;
uint8_t Driver_Step = 0;//�������̼���
uint8_t Driver_NO = 0;//��������

void Data_Init(void)
{
	CPM_State.X8.CPM_FLUnlockFlag = 1;
	CPM_State.X8.CPM_RLUnlockFlag = 1;
	CPM_State.X8.CPM_FRUnlockFlag = 1;
	CPM_State.X8.CPM_RRUnlockFlag = 1;
	CPM_State.X9.CPM_RLChdUnlockFlag = 1;
	CPM_State.X9.CPM_RRChdUnlockFlag = 1;
	CPM_State.X9.CPM_FLHandleFlag = 1;
	CPM_State.X9.CPM_RLHandleFlag = 1;
	CPM_State.X10.CPM_FRHandleFlag = 1;
	CPM_State.X10.CPM_RRHandleFlag = 1;
}

void APP_Program(void)
{
	float adcValue[2] = {0};
	float capaValue;
	uint8_t receive_buf[2];
	uint8_t CRASH_MCU_State;
	while (notif_triggered != TRUE);
	adcValue[0] = ADC_convert_buff[0]*5.0*230/(16384*30);
	adcValue[1] = ADC_convert_buff[1]*5.0*230/(16384*30);
	capaValue = Stack_Voltage()*1.0/1000;
	if(adcValue[0]>9 && adcValue[0]-capaValue>1.5f)
	{
		CPM_State.X10.CPM_ChargSt = 1;
		CPM_State.X10.CPM_ChgSt = 1;
		Siul2_Dio_Ip_WritePin(SEPIC_EN_PORT, SEPIC_EN_PIN, 1);
	}
	else if(adcValue[0]<9 || adcValue[0]-capaValue<1)
	{
		CPM_State.X10.CPM_ChargSt = 0;
		CPM_State.X10.CPM_ChgSt = 0;
		Siul2_Dio_Ip_WritePin(SEPIC_EN_PORT, SEPIC_EN_PIN, 0);
		if(Balance_Count > 20000)
		{
			Cell_Balancing(0x3E);
			Balance_Count = 0;
		}
	}
	CPM_State.X14.CPM_BalCmd1 = 1;
	CPM_State.X14.CPM_BalCmd2 = 1;
	CPM_State.X14.CPM_BalCmd3 = 1;
	CPM_State.X14.CPM_BalCmd4 = 1;
	CPM_State.X14.CPM_BalCmd5 = 1;
	if(adcValue[0] < 9)
	{
		if(isCutoff == 0)
		{
			isCutoff = 1;
			Drive_Count = 0;
			Driver_Step = 0;
			Driver_NO = 0;
		}
	}
	else
	{
		isCutoff = 0;
	}
	//��ײ�ź�
	CRASH_MCU_State = Siul2_Dio_Ip_ReadPin(CRASH_MCU_PORT, CRASH_MCU_PIN);
	if(CRASH_MCU_State!=0 && CRASH_MCU_Old==0)
	{
		CRASH_Count = 0;
	}
	else if(CRASH_MCU_State==0 && CRASH_MCU_Old!=0)
	{
		if(CRASH_Count>10 && CRASH_Count<=20)
		{
			isCrash = 1;
			CPM_State.X14.CPM_PWMSts = 1;
//			Siul2_Dio_Ip_WritePin(LED_PORT, LED_PIN, 0);
		}
		else if(CRASH_Count<6 || CRASH_Count>20)
		{
			isCrash = 0;
			CPM_State.X14.CPM_PWMSts = 0;
//			Siul2_Dio_Ip_WritePin(LED_PORT, LED_PIN, 1);
		}
	}
	CRASH_MCU_Old = CRASH_MCU_State;
//	if((isCutoff==1 || isCrash==1) && Driver_NO<2)
	if(0)
	{
		uint8_t Driver_ret = 1;
		if(Drive_Count<10000)
		{
			//�ȴ�10s
		}
		else if(Drive_Count>=10000 && Drive_Count<10100 && Driver_Step==0)
		{
			Driver_ret = Motor_Control_Safe(MOTOR_1, MOTOR_DIR_FORWARD, receive_buf);//��ʼ������ǰ�п�
			if(Driver_ret == 0)
			{
				Driver_Step = 1;
			}
		}
		else if(Drive_Count>=10100 && Drive_Count<10110 && Driver_Step==1)
		{
			Driver_ret = Motor_Control_Safe(MOTOR_1, MOTOR_DIR_STOP, receive_buf);//ֹͣ������ǰ�п�
			if(Driver_ret == 0)
			{
				Driver_Step = 2;
			}
		}
		else if(Drive_Count>=10110 && Drive_Count<10210 && Driver_Step==2)//��ʼ������ǰ�п�
		{
			Driver_ret = Motor_Control_Safe(MOTOR_2, MOTOR_DIR_FORWARD, receive_buf);
			if(Driver_ret == 0)
			{
				Driver_Step = 3;
			}
		}
		else if(Drive_Count>=10210 && Drive_Count<10220 && Driver_Step==3)//ֹͣ������ǰ�п�
		{
			Driver_ret = Motor_Control_Safe(MOTOR_2, MOTOR_DIR_STOP, receive_buf);
			if(Driver_ret == 0)
			{
				Driver_Step = 4;
			}
		}
		else if(Drive_Count>=10220 && Drive_Count<10320 && Driver_Step==4)//��ʼ��������п�
		{
			Driver_ret = Motor_Control_Safe(MOTOR_3, MOTOR_DIR_FORWARD, receive_buf);
			if(Driver_ret == 0)
			{
				Driver_Step = 5;
			}
		}
		else if(Drive_Count>=10320 && Drive_Count<10330 && Driver_Step==5)//ֹͣ��������п�
		{
			Driver_ret = Motor_Control_Safe(MOTOR_3, MOTOR_DIR_STOP, receive_buf);
			if(Driver_ret == 0)
			{
				Driver_Step = 6;
			}
		}
		else if(Drive_Count>=10330 && Drive_Count<10430 && Driver_Step==6)//��ʼ�����Һ��п�
		{
			Driver_ret = Motor_Control_Safe(MOTOR_4, MOTOR_DIR_FORWARD, receive_buf);
			if(Driver_ret == 0)
			{
				Driver_Step = 7;
			}
		}
		else if(Drive_Count>=10430 && Drive_Count<10440 && Driver_Step==7)//ֹͣ�����Һ��п�
		{
			Driver_ret = Motor_Control_Safe(MOTOR_4, MOTOR_DIR_STOP, receive_buf);
			if(Driver_ret == 0)
			{
				Driver_Step = 8;
			}
		}
		else if(Drive_Count>=10440 && Drive_Count<11240 && Driver_Step==8)//��ʼ������ǰ�Ű���
		{
			Driver_ret = Motor_Control_Safe(MOTOR_5, MOTOR_DIR_FORWARD, receive_buf);
			if(Driver_ret == 0)
			{
				Driver_Step = 9;
			}
		}
		else if(Drive_Count>=11240 && Drive_Count<11250 && Driver_Step==9)//ֹͣ������ǰ�Ű���
		{
			Driver_ret = Motor_Control_Safe(MOTOR_5, MOTOR_DIR_STOP, receive_buf);
			if(Driver_ret == 0)
			{
				Driver_Step = 10;
			}
		}
		else if(Drive_Count>=11250 && Drive_Count<12050 && Driver_Step==10)//��ʼ������ǰ�Ű���
		{
			Driver_ret = Motor_Control_Safe(MOTOR_6, MOTOR_DIR_FORWARD, receive_buf);
			if(Driver_ret == 0)
			{
				Driver_Step = 11;
			}
		}
		else if(Drive_Count>=12050 && Drive_Count<12060 && Driver_Step==11)//ֹͣ������ǰ�Ű���
		{
			Driver_ret = Motor_Control_Safe(MOTOR_6, MOTOR_DIR_STOP, receive_buf);
			if(Driver_ret == 0)
			{
				Driver_Step = 12;
			}
		}
		else if(Drive_Count>=12060 && Drive_Count<12860 && Driver_Step==12)//��ʼ��������Ű���
		{
//			Driver_ret = ;
			if(Driver_ret == 0)
			{
				Driver_Step = 13;
			}
		}
		else if(Drive_Count>=12860 && Drive_Count<12870 && Driver_Step==13)//ֹͣ��������Ű���
		{
//			Driver_ret = ;
			if(Driver_ret == 0)
			{
				Driver_Step = 14;
			}
		}
		else if(Drive_Count>=12870 && Drive_Count<13670 && Driver_Step==14)//��ʼ�����Һ��Ű���
		{
//			Driver_ret = ;
			if(Driver_ret == 0)
			{
				Driver_Step = 15;
			}
		}
		else if(Drive_Count>=13670 && Drive_Count<13680 && Driver_Step==15)//ֹͣ�����Һ��Ű���
		{
//			Driver_ret = ;
			if(Driver_ret == 0)
			{
				Driver_Step = 16;
			}
		}
		else if(Drive_Count>=13680 && Drive_Count<13780 && Driver_Step==16)//��ʼ��������ͯ��
		{
//			Driver_ret = ;
			if(Driver_ret == 0)
			{
				Driver_Step = 17;
			}
		}
		else if(Drive_Count>=13780 && Drive_Count<13790 && Driver_Step==17)//ֹͣ��������ͯ��
		{
//			Driver_ret = ;
			if(Driver_ret == 0)
			{
				Driver_Step = 18;
			}
		}
		else if(Drive_Count>=13790 && Drive_Count<13890 && Driver_Step==18)//��ʼ�����Һ��ͯ��
		{
//			Driver_ret = ;
			if(Driver_ret == 0)
			{
				Driver_Step = 19;
			}
		}
		else if(Drive_Count>=13890 && Drive_Count<13900 && Driver_Step==19)//ֹͣ�����Һ��ͯ��
		{
//			Driver_ret = ;
			if(Driver_ret == 0)
			{
				Driver_Step = 20;
			}
		}
		else if(Drive_Count >= 13900)
		{
			Drive_Count = 10000;
			Driver_Step = 0;
			Driver_NO++;
		}
	}
	CPM_State.CPM_Cell1Vol = ReadCellVoltage(0)/100;
	CPM_State.CPM_Cell2Vol = ReadCellVoltage(1)/100;
	CPM_State.CPM_Cell3Vol = ReadCellVoltage(2)/100;
	CPM_State.CPM_Cell4Vol = ReadCellVoltage(3)/100;
	CPM_State.CPM_Cell5Vol = ReadCellVoltage(4)/100;
	CPM_State.CPM_KL30Vol = adcValue[0]*10;
//	CPM_State.CPM_BoardTemp = ;
	CPM_State.CPM_Current = Read_CC1_Current()/100;
	Sys_CAN_Send_Message();
}

void AppTask( void *pvParameters )
{
    (void)pvParameters;
    uint8_t isCutoff = 0;
    uint8_t receive_buf[2];
    while(1)
    {
    	/* Wait for the notification to be triggered and read the data */
		while (notif_triggered != TRUE);
		float adcValue[2] = {0};
		adcValue[0] = ADC_convert_buff[0]*5.0*230/(4096*30);
		adcValue[1] = ADC_convert_buff[1]*5.0*230/(4096*30);
		if(adcValue[0] > 9 && adcValue[0]-(ReadCellVoltage(7)/1000) > 1.5f)
		{
			Siul2_Dio_Ip_WritePin(SEPIC_EN_PORT, (1 << SEPIC_EN_PIN), 1);
		}
		else if(adcValue[0] < 9 || adcValue[0]-(ReadCellVoltage(7)/1000) < 1)
		{
			Siul2_Dio_Ip_WritePin(SEPIC_EN_PORT, (1 << SEPIC_EN_PIN), 0);
			if(Balance_Count > 20000)
			{
				Cell_Balancing(0x3E);
				Balance_Count = 0;
			}
		}
		if(adcValue[0] < 9)
		{
			isCutoff = 1;
			Drive_Count = 0;
		}
		else
		{
			isCutoff = 0;
		}
		//��ײ�ź�
		if(isCutoff==1)
		{
			if(Drive_Count < 400)
			{
				Motor_Control_Safe(MOTOR_1, MOTOR_DIR_FORWARD, receive_buf);
			}
			else if(Drive_Count < 1400)
			{
				Motor_Control_Safe(MOTOR_1, MOTOR_DIR_STOP, receive_buf);
			}
			else if(Drive_Count < 1800)
			{
				Motor_Control_Safe(MOTOR_2, MOTOR_DIR_FORWARD, receive_buf);
			}
			else if(Drive_Count < 2800)
			{
				Motor_Control_Safe(MOTOR_2, MOTOR_DIR_STOP, receive_buf);
			}
			else if(Drive_Count < 3200)
			{
				Motor_Control_Safe(MOTOR_3, MOTOR_DIR_FORWARD, receive_buf);
			}
			else if(Drive_Count < 4200)
			{
				Motor_Control_Safe(MOTOR_3, MOTOR_DIR_STOP, receive_buf);
			}
			else
			{
				Drive_Count = 0;
			}
		}
		if(Sleep_Count>500)
		{
			uint16 RxData = 0;
			Sbc_fs23_ReadRegister(SBC_FS23_M_SYS_CFG_ADDR, &RxData);
			if((RxData & SBC_FS23_M_GO2LPOFF_MASK) == SBC_FS23_M_GO2LPOFF_NO_ACTION)
			{
				RxData = RxData | SBC_FS23_M_GO2LPOFF_MASK;
			}
			Sbc_fs23_WriteRegister(SBC_FS23_M_SYS_CFG_ADDR,RxData);
		}
		vTaskDelay(10);
    }
}

void SendTask( void *pvParameters )
{
    (void)pvParameters;
    can_message_queue_t DataTxBuff;

    StartCanSendMsgTimer();

    while (1)
    {
    	Sys_CAN_Send_Message();
        if( (xQueueCanTx != NULL) )
        {
            //���ն��в��ȴ�--
            //if(xQueueReceive( xQueueTx,&DataTxBuff, portMAX_DELAY)==pdPASS)
            if(xQueueReceive( xQueueCanTx, &DataTxBuff, 0) == pdPASS)
            {
				if(CAN0_SendMsg(DataTxBuff.id, DataTxBuff.data, DataTxBuff.length) != 0x00)
				{
					vTaskDelay(1);//vTaskPend(1);
					//xQueueSendToFront(xQueueCanTx, &DataTxBuff, (TickType_t) 0);
				}
            }
            else
            {
                vTaskDelay(1);
            }
        }
        else
        {
            vTaskDelay(1);
        }
    }//while(1)
}

void ReceiveTask( void *pvParameters )
{
    (void)pvParameters;
    can_message_queue_t DataReceived;

	while(1)
	{
		if( xQueueCanRx != 0 )
		{
			//if(xQueueReceive( xQueueRx,&DataReceived, portMAX_DELAY)==pdPASS)
			if(xQueueReceive( xQueueCanRx,&DataReceived, 0)==pdPASS)
			{
//				CAN_Rx_Message_Handle(&DataReceived);
			}
		}
		vTaskDelay(2);
	}
}
SemaphoreHandle_t sem_handle;
void TaskInit(void)
{
	vSemaphoreCreateBinary(sem_handle);
	/* Create the queue. */
	xQueueCanTx = xQueueCreate(30,sizeof(can_message_queue_t));
	xQueueCanRx = xQueueCreate(30,sizeof(can_message_queue_t));

	xTaskCreate( AppTask   , ( const char * const ) "AppTask", configMINIMAL_STACK_SIZE, (void*)0, App_TASK_PRIORITY, NULL );
	xTaskCreate( SendTask   , ( const char * const ) "SendTask", configMINIMAL_STACK_SIZE, (void*)0, Send_TASK_PRIORITY, NULL );
	xTaskCreate( ReceiveTask, ( const char * const ) "RecTask" , configMINIMAL_STACK_SIZE, (void*)0, Rec_TASK_PRIORITY, NULL );
//	xTaskCreate( UdsTask, ( const char * const ) "UdsTask" , configMINIMAL_STACK_SIZE, (void*)0, Uds_TASK_PRIORITY, NULL );
}
