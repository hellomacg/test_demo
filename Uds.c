/*
 * Uds.c
 *
 *  Created on: 2025��9��1��
 *      Author: maode1
 */
#include "uds.h"
#include "Flash_drivers.h"
#include <string.h>
#include <stdint.h>
#include "Pit_Ip.h"
#include "CanService.h"
uint16 usLPTCount=0;
uint16 usKeyCommand=0;/*����*/
uint8 gucFlagSeedStatic=0;/*����״̬ 0δ����  1������*/
uint8 gucMispassCount = 0;/*��Կ�������*/
uint8 IdentifierData[18];
uint8 flowadd_flag = 0;
uint8 FlowLost_time = 0;
uint8 flowadd_last = 0;
uint8 Time_Out_Flag = 1;
uint32 ulMemoryAdd_result = 0;         /****�ڴ��ַ****/
uint32 ulMemorySize_result = 0;        /****�ڴ��С****/
uint8 gucDownloadFlag=0;/*0:��������û�м��� 1:���������Ѽ��� 2:ˢ�¹������*/
uint16 usBlockNumer = 0;/*0xFF�ۼ�*/
uint16 usBlockNumerTemp = 0;/*0xFF��־*/
uint16 usBlockNo=0;/*�������*/
uint8 gSmallBatteryVoltage=120;      /***********С��ص�ѹ ��λV****************************/
uint16 usBlockNo_old=0;/*��ʷ����*/

uint8 handle1;

uint16_t gusJumpToUserTimer=0;/*��ϱ���ʱ�����*/
uint16_t usMispass_Time=0;/*��Կ����ʱ�����*/
uint8_t MultiLost_time = 0;

/* 1ms*/
void PitRtiNotification(void)
{
	usLPTCount++;
	if(usLPTCount>=500)
	{
		usLPTCount=0;
	}
	if(Time_Out_Flag==1 && gusJumpToUserTimer<10000)
	{
		gusJumpToUserTimer++;
	}
	if(gucMispassCount > 2)
	{
		usMispass_Time++;
		if(usMispass_Time>=10000)
		{
			gucMispassCount = 0;
			usMispass_Time = 0;
		}
	}
	if(MultiLost_time < 200)
	{
		MultiLost_time++;
	}
	if(FlowLost_time < 100)
	{
		FlowLost_time++;
	}
}
void flexcan_send_demo(uint8 handle, uint32 ID, uint8 *u8DataBuff, uint8 tx_index)
{
	CAN0_UDS_SendMsg(ID,u8DataBuff,8);
}
void Appl_GotoAppSW(uint32 address)/*PRQA S 3006*/
{

	uint32 func = address;
    func = *(uint32_t volatile *)(func + 0xC);
    func = *(uint32_t volatile *)(((uint32_t)func) + 0x4);
    func = ((((uint32_t)func) & 0xFFFFFFFFU));    // Reset_Handler+1  --> required to avoid hard fault
    (* (void (*) (void)) func)();
}

void MCU_Reset(void)
{
    // ���ÿ��Ź��жϣ����ʹ���ˣ�
    // Ȼ��ι�����ÿ��Ź���ʱ��λ

    // ֹͣι��
    while(1)
    {
        // ��ѭ�����ȴ����Ź���ʱ
    }
}

void udelay(uint32_t delay)
{
    static volatile uint32 DelayTimer = 0;
    delay=delay*160;
    while(DelayTimer < delay)
    {
        DelayTimer++;
    }
    DelayTimer=0;
}

void Did_read(uint32_t did, uint8_t *pbuf)
{
	uint8_t F180_Data[8]={0,0,0,0,0,0,0,0};
	uint8_t F1D0_Data[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};
	switch (did)
	{
	case 0xF180:
		memcpy(pbuf, F180_Data, 8);
        break;
    case 0xF1D0:
    	memcpy(pbuf, F1D0_Data, 13);;

        break;

    default:
        break;
	}
}
void dealwith10(uint8 *stic,uint8 *sticlock,uint8 AddressMode,CAN_Data pCAN_Data)
{
    uint8 SendData[8] = {0};
    if(pCAN_Data.Length != 2)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x10;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if((pCAN_Data.data[0]&0x7F)!=0x01 && (pCAN_Data.data[0]&0x7F)!=0x02 && (pCAN_Data.data[0]&0x7F)!=0x03)  /*0x12   �ӹ��ܲ�֧��*/
    {
        if(AddressMode == 0)
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x10;
            SendData[3] = 0x12;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
    }
    else
    {
        if((pCAN_Data.data[0]&0x7F)==0x01)/**Ĭ�Ϲ���ģʽ**/
        {
            *sticlock=0;
            gucFlagSeedStatic=0;
            gucDownloadFlag=0;
            if(pCAN_Data.data[0]>>7==0)
            {
                SendData[0] = 0x06;
                SendData[1] = 0x50;
                SendData[2] = 0x01;
                SendData[3] = PROGRAM_TIMER1>>8;
                SendData[4] = PROGRAM_TIMER1&0xFF;
                SendData[5] = PROGRAM_TIMER2>>8;
                SendData[6] = PROGRAM_TIMER2&0xFF;
                SendData[7] = 0x00;
                flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            }/* end if of if(ReceiveData.data[0]>>7 == 0) */

        	MCU_Reset();
            *stic = 1;
        }
        else if((pCAN_Data.data[0]&0x7F)==0x02)/**ˢ��ģʽ**/
        {
            if(*stic == 1)
            {
                if(AddressMode==0)
                {
                    SendData[0] = 0x03;
                    SendData[1] = 0x7F;
                    SendData[2] = 0x10;
                    SendData[3] = 0x7E;
                    SendData[4] = 0x00;
                    SendData[5] = 0x00;
                    SendData[6] = 0x00;
                    SendData[7] = 0x00;
                    flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
                }
            }
            else
            {
                *sticlock=0;
                gucFlagSeedStatic=0;
                gucDownloadFlag=0;
                if(pCAN_Data.data[0]>>7==0)
                {
                    SendData[0] = 0x06;
                    SendData[1] = 0x50;
                    SendData[2] = 0x02;
                    SendData[3] = PROGRAM_TIMER1>>8;
                    SendData[4] = PROGRAM_TIMER1&0xFF;
                    SendData[5] = PROGRAM_TIMER2>>8;
                    SendData[6] = PROGRAM_TIMER2&0xFF;
                    SendData[7] = 0x00;
                    flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
                }/* end if of if(ReceiveData.data[0]>>7 == 0) */
                *stic = 2;
            }
        }
        else if((pCAN_Data.data[0]&0x7F)==0x03)/**��չģʽ**/
        {
            if(*stic == 2)
            {
                if(AddressMode==0)
                {
                    SendData[0] = 0x03;
                    SendData[1] = 0x7F;
                    SendData[2] = 0x10;
                    SendData[3] = 0x7E;
                    SendData[4] = 0x00;
                    SendData[5] = 0x00;
                    SendData[6] = 0x00;
                    SendData[7] = 0x00;
                    flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
                }
            }
            else
            {
                *sticlock=0;
                gucFlagSeedStatic=0;
                gucDownloadFlag=0;
                if(pCAN_Data.data[0]>>7==0)
                {
                    SendData[0] = 0x06;
                    SendData[1] = 0x50;
                    SendData[2] = 0x03;
                    SendData[3] = PROGRAM_TIMER1>>8;
                    SendData[4] = PROGRAM_TIMER1&0xFF;
                    SendData[5] = PROGRAM_TIMER2>>8;
                    SendData[6] = PROGRAM_TIMER2&0xFF;
                    SendData[7] = 0x00;
                    flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
                }/* end if of if(ReceiveData.data[0]>>7 == 0) */
                *stic = 3;
            }/* end if of if(stic == 2 && ReceiveData.data[0]>>7 == 0) */
        }/* end if of if((ReceiveData.data[0]&0x7F)==0x01) */
    }
}

void dealwith11(uint8 stic,uint8 AddressMode,CAN_Data pCAN_Data)
{
    uint8 SendData[8] = {0};
    if(stic != 2)  /*0x7F   ����ǰ�Ự��֧�ָ÷���*/
    {
        if(AddressMode==0)
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x11;
            SendData[3] = 0x7F;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
    }
    else if(pCAN_Data.Length != 2)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x11;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if((pCAN_Data.data[0]&0x7F)!=0x01)  /*0x12   �ӹ��ܲ�֧��*/
    {
        if(AddressMode == 0)
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x11;
            SendData[3] = 0x12;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
    }
    else
    {
        if((pCAN_Data.data[0]&0x7F)==0x01)/**Ӳ��λ**/
        {
            //uint8 DataTemp8 = 0;
            if(pCAN_Data.data[0]>>7==0)
            {
                SendData[0] = 0x02;
                SendData[1] = 0x51;
                SendData[2] = 0x01;
                SendData[3] = 0x00;
                SendData[4] = 0x00;
                SendData[5] = 0x00;
                SendData[6] = 0x00;
                SendData[7] = 0x00;
                flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            }/* end if of if(ReceiveData.data[0]>>7 == 0) */
            //arch_irq_disable();
            //eeprom_write(0x00,&DataTemp8,1);
            //arch_irq_enable();
            udelay(1000);

        	MCU_Reset();
        }/* end if of if((ReceiveData.data[0]&0x7F)==0x01) */
    }/* end if of if(stic != 2) */
}

/****************************************************************************
*
*0x0080-0x0087   F180   	bootloader����汾��	        8�ֽ�	01.01.01
*0x0090-0x009C   F187   	�����㲿����			13�ֽ�	S31-2105100
*0x00A0-0x00A7   F188		����汾��(���������汾)	8�ֽ�	01.02.03
*0x00B0-0x00B7   F1B0		����汾��(�̶��汾)		8�ֽ�	01.02.03
*0x00C0-0x00C2   F18A		ϵͳ��Ӧ�̴���			3�ֽ�	6YT
*0x00D0-0x00D3   F18B		ECU��������			4�ֽ�	20180601
*0x00E0-0x00F1   F18C		���������к�			18�ֽ�	P  EP40 10BA AA31 A 0001
*0x0100-0x0110   F190		����VIN				17�ֽ�	UVVD C11B 6AD3 2428 6
*0x0120-0x0124   F191		Ӳ���汾��(�ڲ������汾)	5�ֽ�	H1.01
*0x0130-0x0134   F1BF		Ӳ���汾��(�̶��汾)		5�ֽ�	H1.01
*0x0140-0x0149   F198		ά�޵����			10�ֽ�	1234567899
*0x0150-0x0153   F199		ˢ������			4�ֽ�	20180601
*0x0160-0x0163   F19D		ECUװ������			4�ֽ�	20180601
*0x0170-0x0177   F1C0		����ܳɰ汾��			8�ֽ�	01.02.03
*0x0180-0x018C   F1D0		����ܳ������			13�ֽ�	S30-2101999CA
*
***************************************************************************/
void dealwith22(uint8 stic,uint8 AddressMode,CAN_Data pCAN_Data)
{
    uint8 SendData[8] = {0};
    uint16 ServiceFunction;
    ServiceFunction = (pCAN_Data.data[0]<<8) + pCAN_Data.data[1];
    if(stic != 2)  /*0x7F   ����ǰ�Ự��֧�ָ÷���*/
    {
        if(AddressMode == 0)
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x22;
            SendData[3] = 0x7F;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
    }
    else if(pCAN_Data.Length != 3)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x22;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(ServiceFunction!=0xF180 && ServiceFunction!=0xF1D0)  /*0x31   ���󳬳���Χ*/
    {
        if(AddressMode == 0)
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x22;
            SendData[3] = 0x31;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
    }
    else
    {
        if(ServiceFunction == 0xF180)  /*0080-0087   F180   	bootloader����汾��	8�ֽ�		01.01.01*/
        {
            Did_read(0xF180,IdentifierData);
            SendData[0] = 0x10;
            SendData[1] = 11;
            SendData[2] = 0x62;
            SendData[3] = pCAN_Data.data[0];
            SendData[4] = pCAN_Data.data[1];
            SendData[5] = IdentifierData[0];
            SendData[6] = IdentifierData[1];
            SendData[7] = IdentifierData[2];
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            FlowLost_time = 0;
            flowadd_flag = 1;
        }
        else if(ServiceFunction == 0xF1D0)  /*0180-018C   F1D0		����ܳ������			13�ֽ�	S30-2101999CA*/
        {
        	Did_read(0xF1D0,IdentifierData);
            SendData[0] = 0x10;
            SendData[1] = 16;
            SendData[2] = 0x62;
            SendData[3] = pCAN_Data.data[0];
            SendData[4] = pCAN_Data.data[1];
            SendData[5] = IdentifierData[0];
            SendData[6] = IdentifierData[1];
            SendData[7] = IdentifierData[2];
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            FlowLost_time = 0;
            flowadd_flag = 2;
        }
        flowadd_last = 0;
    }
}

void flowadd(uint8 BS, uint8 STmin)
{
    uint32 time_delay=0;
    uint8 SendData[8] = {0};
    if(STmin<0x7F)
    {
        time_delay=1000*STmin+3000;
    }
    else if(STmin>=0xF1 && STmin<=0xF9)
    {
        time_delay=100*(STmin&0x0F)+300;
    }
    else
    {
        time_delay=1000*0x7F+1000;
    }
    if(flowadd_flag == 1)
    {
        udelay(time_delay);
        SendData[0] = 0x21;
        SendData[1] = IdentifierData[3];
        SendData[2] = IdentifierData[4];
        SendData[3] = IdentifierData[5];
        SendData[4] = IdentifierData[6];
        SendData[5] = IdentifierData[7];
        SendData[6] = 0x0;
        SendData[7] = 0x0;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        flowadd_flag = 0;
    }
    else if(flowadd_flag == 2)
    {
        if(flowadd_last == 0)
        {
            udelay(time_delay);
            SendData[0] = 0x21;
            SendData[1] = IdentifierData[3];
            SendData[2] = IdentifierData[4];
            SendData[3] = IdentifierData[5];
            SendData[4] = IdentifierData[6];
            SendData[5] = IdentifierData[7];
            SendData[6] = IdentifierData[8];
            SendData[7] = IdentifierData[9];
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }

        if(flowadd_last<2 && (BS==0 || BS+flowadd_last>=2))
        {
            udelay(time_delay);
            SendData[0] = 0x22;
            SendData[1] = IdentifierData[10];
            SendData[2] = IdentifierData[11];
            SendData[3] = IdentifierData[12];
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            flowadd_flag = 0;
        }
    }
    flowadd_last = flowadd_last+BS;
}

void dealwith27(uint8 stic,uint8 *sticlock,CAN_Data pCAN_Data)
{
    uint8 SendData[8] = {0};
    if(stic != 2)  /*0x7F   ����ǰ�Ự��֧�ָ÷���*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x27;
        SendData[3] = 0x7F;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(pCAN_Data.Length < 2)
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x27;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(pCAN_Data.data[0]==0x11 && pCAN_Data.Length!=2)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x27;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(pCAN_Data.data[0]==0x12 && pCAN_Data.Length!=4)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x27;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(pCAN_Data.data[0]!=0x03 && pCAN_Data.data[0]!=0x04 && pCAN_Data.data[0]!=0x11 && pCAN_Data.data[0]!=0x12)  /*0x12   �ӹ��ܲ�֧��*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x27;
        SendData[3] = 0x12;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(pCAN_Data.data[0]==0x03 || pCAN_Data.data[0]==0x04)  /*0x7E   ����ǰ�Ự��֧�ָ��ӹ���*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x27;
        SendData[3] = 0x7E;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(pCAN_Data.data[0]==0x12 && gucFlagSeedStatic==0)/*********����˳�����***********/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x27;
        SendData[3] = 0x24;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(gucMispassCount > 2)  /*0x37   ��ʱû�����*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x27;
        SendData[3] = 0x37;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else
    {
        if(pCAN_Data.data[0]==0x11)
        {
            if(*sticlock==0)/*********û����״̬***********/
            {
                srand(usLPTCount);
                usKeyCommand = rand();
                gucFlagSeedStatic = 1;
                SendData[0] = 0x04;
                SendData[1] = 0x67;
                SendData[2] = 0x11;
                SendData[3] = (usKeyCommand>>8)&0xFF;
                SendData[4] = usKeyCommand&0xFF;
                SendData[5] = 0x00;
                SendData[6] = 0x00;
                SendData[7] = 0x00;
                flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            }
            else if(*sticlock==1)                        /*********�ѽ���״̬***********/
            {
                SendData[0] = 0x04;
                SendData[1] = 0x67;
                SendData[2] = 0x11;
                SendData[3] = 0x00;
                SendData[4] = 0x00;
                SendData[5] = 0x00;
                SendData[6] = 0x00;
                SendData[7] = 0x00;
                flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            }/* end if of if(flag_lock_static==0) */
        }
        else if(pCAN_Data.data[0]==0x12)
        {
            if(*sticlock==0)/*********û����״̬***********/
            {
                uint16 result_Key = calcKey(usKeyCommand);/*��Կ*/
                uint8 Key_MSB = (uint8)(result_Key >> 8);/*��Կ���ֽ�*/
                uint8 Key_LSB = (uint8)(result_Key);/*��Կ���ֽ�*/
                if(pCAN_Data.data[1]==Key_MSB && pCAN_Data.data[2]==Key_LSB)
                {
                    gucMispassCount = 0;
                    *sticlock = 1;
                    SendData[0] = 0x02;
                    SendData[1] = 0x67;
                    SendData[2] = 0x12;
                    SendData[3] = 0x00;
                    SendData[4] = 0x00;
                    SendData[5] = 0x00;
                    SendData[6] = 0x00;
                    SendData[7] = 0x00;
                    flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
                }
                else
                {
                    gucMispassCount++;
                    if(gucMispassCount < 3)  /*0x35   ��Կ������*/
                    {
                        SendData[0] = 0x03;
                        SendData[1] = 0x7F;
                        SendData[2] = 0x27;
                        SendData[3] = 0x35;
                        SendData[4] = 0x00;
                        SendData[5] = 0x00;
                        SendData[6] = 0x00;
                        SendData[7] = 0x00;
                        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
                    }
                    else  /*0x36   ��������Դ���*/
                    {
                        gucFlagSeedStatic = 0;
                        SendData[0] = 0x03;
                        SendData[1] = 0x7F;
                        SendData[2] = 0x27;
                        SendData[3] = 0x36;
                        SendData[4] = 0x00;
                        SendData[5] = 0x00;
                        SendData[6] = 0x00;
                        SendData[7] = 0x00;
                        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
                    }/* end if of if(Mispass_Count < 3) */
                }/* end if of if(ReceiveData.data[1]==Key_MSB && ReceiveData.data[2]==Key_LSB) */
            }
            else if(*sticlock==1)                        /*********�ѽ���״̬***********/
            {
                SendData[0] = 0x03;
                SendData[1] = 0x7F;
                SendData[2] = 0x27;
                SendData[3] = 0x24;
                SendData[4] = 0x00;
                SendData[5] = 0x00;
                SendData[6] = 0x00;
                SendData[7] = 0x00;
                flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            }/* end if of if(flag_seed_static == 0) */
        }/* end if of if(ReceiveData.data[0]==REQUEST_SEED) */
    }
}

/****************************************************************************
*
*0x0080-0x0087   F180   	bootloader����汾��	        8�ֽ�	01.01.01
*0x0090-0x009C   F187   	�����㲿����			13�ֽ�	S31-2105100
*0x00A0-0x00A7   F188		����汾��(���������汾)	8�ֽ�	01.02.03
*0x00B0-0x00B7   F1B0		����汾��(�̶��汾)		8�ֽ�	01.02.03
*0x00C0-0x00C2   F18A		ϵͳ��Ӧ�̴���			3�ֽ�	6YT
*0x00D0-0x00D3   F18B		ECU��������			4�ֽ�	20180601
*0x00E0-0x00F1   F18C		���������к�			18�ֽ�	P  EP40 10BA AA31 A 0001
*0x0100-0x0110   F190		����VIN				17�ֽ�	UVVD C11B 6AD3 2428 6
*0x0120-0x0124   F191		Ӳ���汾��(�ڲ������汾)	5�ֽ�	H1.01
*0x0130-0x0134   F1BF		Ӳ���汾��(�̶��汾)		5�ֽ�	H1.01
*0x0140-0x0149   F198		ά�޵����			10�ֽ�	1234567899
*0x0150-0x0153   F199		ˢ������			4�ֽ�	20180601
*0x0160-0x0163   F19D		ECUװ������			4�ֽ�	20180601
*0x0170-0x0177   F1C0		����ܳɰ汾��			8�ֽ�	01.02.03
*0x0180-0x018C   F1D0		����ܳ������			13�ֽ�	S30-2101999CA
*
***************************************************************************/
void dealwith2E(uint8 stic,uint8 sticlock,CAN_Data pCAN_Data)
{
    uint8 SendData[8] = {0};
    uint16 ServiceFunction;
    ServiceFunction = (pCAN_Data.data[0]<<8) + pCAN_Data.data[1];
    if(stic != 2)  /*0x7F   ����ǰ�Ự��֧�ָ÷���*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x2E;
        SendData[3] = 0x7F;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(sticlock == 0)  /*0x33   ��ȫ�ܾ�����*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x2E;
        SendData[3] = 0x33;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(pCAN_Data.Length < 3)
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x2E;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(ServiceFunction==0xF199 && pCAN_Data.Length!=7)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x2E;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(ServiceFunction==0xF198 && pCAN_Data.Length!=13)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x2E;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(ServiceFunction!=0xF199 && ServiceFunction!=0xF198)  /*0x31   ���󳬳���Χ*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x2E;
        SendData[3] = 0x31;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else
    {
#if 0
        uint8 DataTemp8[10];
        uint8 ret;
        if(ServiceFunction == 0xF199)  /*0150-0153   F199		ˢ������				4�ֽ�		20180601*/
        {
            DataTemp8[0] = pCAN_Data.data[2];
            DataTemp8[1] = pCAN_Data.data[3];
            DataTemp8[2] = pCAN_Data.data[4];
            DataTemp8[3] = pCAN_Data.data[5];
            arch_irq_disable();
            ret = eeprom_write(0x150,DataTemp8,4);
            arch_irq_enable();
        }
        else if(ServiceFunction == 0xF198)  /*0140-0149   F198		ά�޵����				10�ֽ�	1234567899*/
        {
            DataTemp8[0] = pCAN_Data.data[2];
            DataTemp8[1] = pCAN_Data.data[3];
            DataTemp8[2] = pCAN_Data.data[4];
            DataTemp8[3] = pCAN_Data.data[5];
            DataTemp8[4] = pCAN_Data.data[6];
            DataTemp8[5] = pCAN_Data.data[7];
            DataTemp8[6] = pCAN_Data.data[8];
            DataTemp8[7] = pCAN_Data.data[9];
            DataTemp8[8] = pCAN_Data.data[10];
            DataTemp8[9] = pCAN_Data.data[11];
            arch_irq_disable();
            ret = eeprom_write(0x140,DataTemp8,10);
            arch_irq_enable();
        }/* end if of if(ServiceFunction == 0XF199) */
        if(ret == 0)
        {
            SendData[0] = 0x03;
            SendData[1] = 0x6E;
            SendData[2] = pCAN_Data.data[0];
            SendData[3] = pCAN_Data.data[1];
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
        }
        else
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x2E;
            SendData[3] = 0x72;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
        }
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
#endif
    }/* end if of if(stic != 2) */
}

void dealwith31(uint8 stic,uint8 sticlock,CAN_Data pCAN_Data)
{
    uint8 SendData[8] = {0};
    uint16 SubServiceFunction;
    SubServiceFunction = (pCAN_Data.data[1]<<8) + pCAN_Data.data[2];
    if(stic != 2)/*0x7E   ����ǰ�Ựģʽ��֧�ָ÷���*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x31;
        SendData[3] = 0x7F;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(sticlock == 0)/*0x33   ��ȫ�ܾ�����*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x31;
        SendData[3] = 0x33;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(pCAN_Data.Length < 4)
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x31;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(SubServiceFunction==0xFF00 && pCAN_Data.Length!=12)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x31;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(SubServiceFunction==0xFF01 && pCAN_Data.Length!=14)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x31;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(pCAN_Data.data[0]!=0x01)  /*0x12   �ӹ��ܲ�֧��*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x31;
        SendData[3] = 0x12;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(SubServiceFunction!=0xFF00 && SubServiceFunction!=0xFF01)  /*0x31   ���󳬳���Χ*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x31;
        SendData[3] = 0x31;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else
    {
        if(SubServiceFunction == 0xFF00)/*��ʼ�����ڴ�*/
        {
            uint32 EraseLogicalBlock = 0;         /****�ڴ���ʼ��ַ****/
            uint32 LogicalBlockNumber = 0;         /****�ڴ��С****/
            EraseLogicalBlock = ((uint32)pCAN_Data.data[3]<<24) + ((uint32)pCAN_Data.data[4]<<16) + ((uint32)pCAN_Data.data[5]<<8) + (uint32)pCAN_Data.data[6];
            LogicalBlockNumber = ((uint32)pCAN_Data.data[7]<<24) + ((uint32)pCAN_Data.data[8]<<16) + ((uint32)pCAN_Data.data[9]<<8) + (uint32)pCAN_Data.data[10];
            if(EraseLogicalBlock<0x00400000 || LogicalBlockNumber<1 || EraseLogicalBlock+LogicalBlockNumber>0x00600000)  /*0x31   ���󳬳���Χ*/
            {
                SendData[0] = 0x03;
                SendData[1] = 0x7F;
                SendData[2] = 0x31;
                SendData[3] = 0x31;
                SendData[4] = 0x00;
                SendData[5] = 0x00;
                SendData[6] = 0x00;
                SendData[7] = 0x00;
                flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            }
            else
            {
                uint8 ret;
                Delay_5s(0x31);/*��ʱ5s*/
                Flash_EraseRange(0x10012000,1);
                ret = Flash_EraseRange(EraseLogicalBlock, LogicalBlockNumber);
                if(ret != 0)
                {
                    SendData[0] = 0x05;
                    SendData[1] = 0x71;
                    SendData[2] = 0x01;
                    SendData[3] = 0xFF;
                    SendData[4] = 0x00;
                    SendData[5] = 0x02;
                    SendData[6] = 0x00;
                    SendData[7] = 0x00;
                    flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
                }
                else
                {
                    SendData[0] = 0x05;
                    SendData[1] = 0x71;
                    SendData[2] = 0x01;
                    SendData[3] = 0xFF;
                    SendData[4] = 0x00;
                    SendData[5] = 0x05;
                    SendData[6] = 0x00;
                    SendData[7] = 0x00;
                    flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
                }/* end if of if(LogicalBlockNumber <= 0x200) */
            }/* end if of if(EraseLogicalBlock < 0xFC00 || LogicalBlockNumber < 1 || EraseLogicalBlock+LogicalBlockNumber-1 >= 0x30400) */
        }
        else if(SubServiceFunction == 0xFF01)/*��ʼ����ˢ�¿ɿ���*/
        {
            uint32 MemoryAdd_Check = 0;        /****�ڴ��ַ****/
            uint32 MemorySize_Check = 0;       /****�ڴ��С****/
            MemoryAdd_Check = ((uint32)pCAN_Data.data[3]<<24) + ((uint32)pCAN_Data.data[4]<<16) + ((uint32)pCAN_Data.data[5]<<8) + (uint32)pCAN_Data.data[6];
            MemorySize_Check = ((uint32)pCAN_Data.data[7]<<24) + ((uint32)pCAN_Data.data[8]<<16) + ((uint32)pCAN_Data.data[9]<<8) + (uint32)pCAN_Data.data[10];
            if(MemoryAdd_Check<0x00400000 || MemorySize_Check<1 || MemoryAdd_Check+MemorySize_Check>0x00600000)
            {
                SendData[0] = 0x03;
                SendData[1] = 0x7F;
                SendData[2] = 0x31;
                SendData[3] = 0x31;
                SendData[4] = 0x00;
                SendData[5] = 0x00;
                SendData[6] = 0x00;
                SendData[7] = 0x00;
                flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            }
            else
            {
                uint16 result_crc;
                uint8 crc_MSB;
                uint8 crc_LSB;

                Delay_5s(0x31);/*��ʱ5s*/
                Time_Out_Flag = 0;
                //result_crc = calculate_flash_crc(MemoryAdd_Check,MemorySize_Check);
                crc_MSB = (result_crc>>8)&0xFF;
                crc_LSB = result_crc&0xFF;
                if(pCAN_Data.data[11]==crc_MSB && pCAN_Data.data[12]==crc_LSB)
                {
                    //APP2toAPP1();
                    Time_Out_Flag = 1;
                    Flash_WriteRange(0x10004000,1,Time_Out_Flag);
                    SendData[0] = 0x07;
                    SendData[1] = 0x71;
                    SendData[2] = 0x01;
                    SendData[3] = 0xFF;
                    SendData[4] = 0x01;
                    SendData[5] = 0x02;
                    SendData[6] = crc_MSB;
                    SendData[7] = crc_LSB;
                    flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
                }
                else
                {
                    SendData[0] = 0x07;
                    SendData[1] = 0x71;
                    SendData[2] = 0x01;
                    SendData[3] = 0xFF;
                    SendData[4] = 0x01;
                    SendData[5] = 0x05;
                    SendData[6] = crc_MSB;
                    SendData[7] = crc_LSB;
                    flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
                }/* end if of if(ReceiveData.data[11] == crc_MSB && ReceiveData.data[12] == crc_LSB) */
            }/* end if of if(MemoryAdd_Check < 0xFC00 || MemorySize_Check < 1 || MemoryAdd_Check+MemorySize_Check-1 >= 0x30400) */
        }/* end if of if(SubServiceFunction == 0x0203) */
    }
}

void dealwith34(uint8 stic,uint8 sticlock,CAN_Data pCAN_Data)
{
    uint8 SendData[8] = {0};
    if(stic != 2)  /*0x7F   ����ǰ�Ự��֧�ָ÷���*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x34;
        SendData[3] = 0x7F;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(sticlock == 0)  /*0x33   ��ȫ�ܾ�����*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x34;
        SendData[3] = 0x33;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(pCAN_Data.Length < 5)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x34;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else
    {
        uint8 MemoryAdd_Length = 0;/*��ַ��ʽ��ʶ��*/
        uint8 MemorySize_Length = 0;/*���ȸ�ʽ��ʶ��*/
        uint32 MemoryAdd = 0;/*�ڴ���ʼ��ַ*/
        uint32 MemorySize = 0;/*�ڴ��С*/

        MemoryAdd_Length = (pCAN_Data.data[1]>>4)&0x0F;
        MemorySize_Length = pCAN_Data.data[1]&0x0F;
        if(MemoryAdd_Length == 1)/*�ڴ���ʼ��ַ1�ֽ�*/
        {
            MemoryAdd = (uint32)pCAN_Data.data[2];
        }
        else if(MemoryAdd_Length == 2)/*�ڴ���ʼ��ַ2�ֽ�*/
        {
            MemoryAdd = ((uint32)pCAN_Data.data[2]<<8) + (uint32)pCAN_Data.data[3];
        }
        else if(MemoryAdd_Length == 3)/*�ڴ���ʼ��ַ3�ֽ�*/
        {
            MemoryAdd = ((uint32)pCAN_Data.data[2]<<16) + ((uint32)pCAN_Data.data[3]<<8) + (uint32)pCAN_Data.data[4];
        }
        else if(MemoryAdd_Length == 4)/*�ڴ���ʼ��ַ4�ֽ�*/
        {
            MemoryAdd = ((uint32)pCAN_Data.data[2]<<24) + ((uint32)pCAN_Data.data[3]<<16) + ((uint32)pCAN_Data.data[4]<<8) + (uint32)pCAN_Data.data[5];
        }/* end if of if(MemoryAdd_Length == 1) */
        if(MemorySize_Length ==1)/*�ڴ��С1�ֽ�*/
        {
            MemorySize = (uint32)pCAN_Data.data[2+MemoryAdd_Length];
        }
        else if(MemorySize_Length ==2)/*�ڴ��С2�ֽ�*/
        {
            MemorySize = ((uint32)pCAN_Data.data[2+MemoryAdd_Length]<<8) + (uint32)pCAN_Data.data[2+MemoryAdd_Length+1];
        }
        else if(MemorySize_Length ==3)/*�ڴ��С3�ֽ�*/
        {
            MemorySize = ((uint32)pCAN_Data.data[2+MemoryAdd_Length]<<16) + ((uint32)pCAN_Data.data[2+MemoryAdd_Length+1]<<8) + (uint32)pCAN_Data.data[2+MemoryAdd_Length+2];
        }
        else if(MemorySize_Length == 4)/*�ڴ��С4�ֽ�*/
        {
            MemorySize = ((uint32)pCAN_Data.data[2+MemoryAdd_Length]<<24) + ((uint32)pCAN_Data.data[2+MemoryAdd_Length+1]<<16) + ((uint32)pCAN_Data.data[2+MemoryAdd_Length+2]<<8) + (uint32)pCAN_Data.data[2+MemoryAdd_Length+3];
        }/* end if of if(MemorySize_Length ==1) */

        if(pCAN_Data.data[0]!=0 || MemoryAdd_Length<1 || MemoryAdd_Length>4 ||
           MemorySize_Length<1 || MemorySize_Length>4)  /*0x31   ���󳬳���Χ*/
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x34;
            SendData[3] = 0x31;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
        else if(pCAN_Data.Length != 3+MemoryAdd_Length+MemorySize_Length)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x34;
            SendData[3] = 0x13;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
        else if(MemoryAdd<0x00400000 || MemorySize<1 || MemoryAdd+MemorySize>0x00600000)  /*0x31   ���󳬳���Χ*/
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x34;
            SendData[3] = 0x31;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
        else if(gucDownloadFlag == 1)  /*0x22   ����������*/
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x34;
            SendData[3] = 0x22;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
        else
        {
            uint32 NumberOfBlockLength = 0;/*������ݿ鳤��*/
            NumberOfBlockLength = 0x42;
            ulMemoryAdd_result = MemoryAdd;
            ulMemorySize_result = MemorySize;
            gucDownloadFlag = 1;
            usBlockNumer = 0;
            usBlockNo_old = 0;
            SendData[0] = 0x04;
            SendData[1] = 0x74;
            SendData[2] = 0x20;
            SendData[3] = (NumberOfBlockLength>>8)&0xFF;
            SendData[4] = NumberOfBlockLength&0xFF;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }/* end if of if(ReceiveData.data[0] != 0 || MemoryAdd_Length < 1 || MemorySize_Length < 1) */
    }/* end if of if(stic != 2) */
}

void dealwith36(uint8 stic,CAN_Data pCAN_Data)
{
    uint8 SendData[8] = {0};
    if(stic != 2)  /*0x7F   ����ǰ�Ự��֧�ָ÷���*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x36;
        SendData[3] = 0x7F;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(pCAN_Data.Length<3 || pCAN_Data.Length>0x42)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x36;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(gucDownloadFlag != 1)  /*0x24   ����˳�����*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x36;
        SendData[3] = 0x24;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else
    {
        if(pCAN_Data.data[0]==0x00 && usBlockNumerTemp==0xFF)
        {
            usBlockNumer=usBlockNumer+256;/***�������������******/
        }/* end if of if(ReceiveData.data[0]==0x00 && BlockNumerTemp == 0xFF) */
        usBlockNumerTemp = pCAN_Data.data[0];
        usBlockNo = usBlockNumer+pCAN_Data.data[0];


        if(usBlockNo==0 || (usBlockNo!=usBlockNo_old && usBlockNo!=usBlockNo_old+1))  /*0x73   ��������ݿ�˳�������*/
        {
            gucDownloadFlag = 0;
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x36;
            SendData[3] = 0x73;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
        else if(pCAN_Data.Length != 0x42)
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x36;
            SendData[3] = 0x31;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
        else if(gSmallBatteryVoltage>165)
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x36;
            SendData[3] = 0x92;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
        else if(gSmallBatteryVoltage<85)
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x36;
            SendData[3] = 0x93;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
        else if(usBlockNo == usBlockNo_old)
        {
            SendData[0] = 0x02;
            SendData[1] = 0x76;
            SendData[2] = pCAN_Data.data[0];
            SendData[3] = 0x00;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
        else if(usBlockNo == usBlockNo_old+1)
        {
            uint8 u8flashProgramDataBuff[64] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
            0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
            0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
            0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
            0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
            0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
            0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
            0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
            uint8 ret;
            usBlockNo_old = usBlockNo;
            memcpy(u8flashProgramDataBuff,pCAN_Data.data+1,64);
            ret = Flash_WriteRange(ulMemoryAdd_result+(usBlockNo-1)*0x40,0x40,u8flashProgramDataBuff);
            /*
            arch_irq_disable();
            ret = sdrv_spi_nor_write(&flash,ulMemoryAdd_result+(usBlockNo-1)*0x40,u8flashProgramDataBuff,0x40);
            arch_irq_enable();
            */
            if(ret != 0)  /*0x72   ���ʧ��*/
            {
                if(usBlockNo*0x40 >= ulMemorySize_result)
                {
                    gucDownloadFlag = 2;
                }
                SendData[0] = 0x02;
                SendData[1] = 0x76;
                SendData[2] = pCAN_Data.data[0];
                SendData[3] = 0x00;
                SendData[4] = 0x00;
                SendData[5] = 0x00;
                SendData[6] = 0x00;
                SendData[7] = 0x00;
                flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            }
            else
            {
                SendData[0] = 0x03;
                SendData[1] = 0x7F;
                SendData[2] = 0x36;
                SendData[3] = 0x72;
                SendData[4] = 0x00;
                SendData[5] = 0x00;
                SendData[6] = 0x00;
                SendData[7] = 0x00;
                flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            }/* end if of if(ret == FL_FAIL) */
        }/* end if of if(BlockNo != BlockNo_old && BlockNo != BlockNo_old+1) */
    }
}

void dealwith37(uint8 stic,CAN_Data pCAN_Data)
{
    uint8 SendData[8] = {0};
    if(stic != 2)  /*0x7F   ����ǰ�Ự��֧�ָ÷���*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x37;
        SendData[3] = 0x7F;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if(pCAN_Data.Length != 1)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x37;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else
    {
        if(gucDownloadFlag!=2)  /*0x24   ����˳�����*/
        {
            gucDownloadFlag = 0;
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x37;
            SendData[3] = 0x24;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
        else
        {
            unsigned short int calc;
            unsigned char crc;
            Delay_5s(0x37);/*��ʱ5s*/
            calc = CalcCRC(ulMemorySize_result,ulMemoryAdd_result);
            crc = ~((char) calc);

            gucDownloadFlag = 0;
            SendData[0] = 0x02;
            SendData[1] = 0x77;
            SendData[2] = crc;
            SendData[3] = 0x00;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }/* end if of if(Download_flag!=2 || ret==FL_FAIL) */
    }/* end if of if(stic != 2) */
}

void dealwith3E(uint8 stic,uint8 AddressMode,CAN_Data pCAN_Data)
{
    uint8 SendData[8] = {0};
    if(stic != 2)  /*0x7F   ����ǰ�Ự��֧�ָ÷���*/
    {
        if(AddressMode == 0)
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x3E;
            SendData[3] = 0x7F;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
    }
    else if(pCAN_Data.Length != 2)  /*0x13   ��Ϣ�ĳ��ȴ�����ʽ������*/
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x3E;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else if((pCAN_Data.data[0]&0x7F)!=0x00)  /*0x12   �ӹ��ܲ�֧��*/
    {
        if(AddressMode == 0)
        {
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = 0x3E;
            SendData[3] = 0x12;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
    }
    else
    {
        if(pCAN_Data.data[0]>>7==0)
        {
            SendData[0] = 0x02;
            SendData[1] = 0x7E;
            SendData[2] = pCAN_Data.data[0];
            SendData[3] = 0x00;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }/* end if of if(ReceiveData.data[0]>>7 == 0) */
    }/* end if of if(stic != 2) */
}

void dealwith3E_80(uint8 *hold_buf)
{
    uint8 SendData[8] = {0};
    if(hold_buf[0] != 2)  //0x13   ��Ϣ�ĳ��ȴ�����ʽ������
    {
        SendData[0] = 0x03;
        SendData[1] = 0x7F;
        SendData[2] = 0x3E;
        SendData[3] = 0x13;
        SendData[4] = 0x00;
        SendData[5] = 0x00;
        SendData[6] = 0x00;
        SendData[7] = 0x00;
        flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
    }
    else
    {
        if(hold_buf[2]>>7 == 0)
        {
            SendData[0] = 0x02;
            SendData[1] = 0x7E;
            SendData[2] = hold_buf[2];
            SendData[3] = 0x00;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }/* end if of if(hold_buf[2]>>7 == 0) */
    }
}

uint16 calcKey(uint16 seed)
{
    #define TOPBIT 0x8000
    #define POLYNOM_1 0x9367
    #define POLYNOM_2 0x2956
    #define BITMASK 0x0080
    #define INITIAL_REMINDER 0xFFFE
    #define MSG_LEN 2 /* seed length in bytes */
    uint8 bSeed[2];
    uint16 remainder;
    uint8 n;
    uint8 i;
    bSeed[0] = (uint8)(seed >> 8); /* MSB */
    bSeed[1] = (uint8)seed; /* LSB */
    remainder = INITIAL_REMINDER;
    for (n = 0; n < MSG_LEN; n++)
    {
        /* Bring the next byte into the remainder. */
        remainder ^= ((bSeed[n]) << 8);
        /* Perform modulo-2 division, a bit at a time. */
        for (i = 0; i < 12; i++)
        {
            /* Try to divide the current data bit. */
            if (remainder & TOPBIT)
            {
                if(remainder & BITMASK)
                {
                    remainder = (remainder << 2) ^ POLYNOM_1;
                }
                else
                {
                    remainder = (remainder << 2) ^ POLYNOM_2;
                }
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }
    /* The final remainder is the key */
    return remainder;
}

void Delay_5s(uint8 mark)
{
    uint8 SendData[8] = {0};
    SendData[0] = 0x03;
    SendData[1] = 0x7F;
    SendData[2] = mark;
    SendData[3] = 0x78;
    SendData[4] = 0x00;
    SendData[5] = 0x00;
    SendData[6] = 0x00;
    SendData[7] = 0x00;
    flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
}

/*
void APP2toAPP1()
{
    uint16 i;
    uint8 DataTemp8[512];

    uint8 DataTemp_boot = 1;
    arch_irq_disable();
    eeprom_write(0x500,&DataTemp_boot,1);
    arch_irq_enable();

    arch_irq_disable();
    sdrv_spi_nor_erase(&flash,0x140000,0x100000);
    arch_irq_enable();
    Delay_5s(0x31);
    arch_irq_disable();
    sdrv_spi_nor_erase(&flash,0x240000,0x40000);
    for(i=0;i<0x140000/512;i++)
    {
        sdrv_spi_nor_read(&flash, 0x280000+i*512, DataTemp8, 512);
        sdrv_spi_nor_write(&flash, 0x140000+i*512, DataTemp8, 512);
        if(i == 1280)
        {
          arch_irq_enable();
          Delay_5s(0x31);
          arch_irq_disable();
        }
        else if(i == 2559)
        {
            DataTemp_boot = 2;
            arch_irq_disable();
            eeprom_write(0x500,&DataTemp_boot,1);
            arch_irq_enable();
        }
    }
    arch_irq_enable();
}
*/

/* Calculate CRC */
unsigned short int CalcCRC(uint32 size, uint32 data)
{
    unsigned short int tmp=0;
    uint16 i;
    for(i=0;i<size/0x20;i++)
    {
        uint8 DataTemp8[32];
        uint8 j;
        //sdrv_spi_nor_read(&flash, data+i*0x20, DataTemp8, 0x20);
        for(j=0;j<32;j++)
        {
            tmp= tmp + DataTemp8[j];
        }
    } /* result of above calculation shall be: tmp=0x1F0 */
    return tmp;
}



void CAN_busoff_check(void)
{


    return;
}
void Diagnostic_MainProc(void)
{
    uint8_t updata=0;
    uint8_t temp_8data[8];
    CAN_Data ReceiveData;
    uint8_t FinishReceivePhysical = 0;	//һ֡��������ݽ�����ɱ�־
    uint8_t FinishReceiveFunction = 0;
    uint8_t SendData[8] = {0};
    uint8_t ucFlagStatic=1;/****MCU����״̬ 1Ĭ��ģʽ 2ˢ��ģʽ  3��չģʽ******/
    uint8_t ucFlagUnlockStatus=0;/****MCU����״̬ 0 û����  1�ѽ���******/
    uint8_t boot_31_flag = 0;
while(IRQ_CAN0_RX==1 && (recvMsg_CAN1.id==0x671 || recvMsg_CAN1.id==0x7DF))
{
    gusJumpToUserTimer=0;
    ReceiveData.SingleOrMore = recvMsg_CAN1.data[0]>>4;				        /***��һ���ֽ�����4λ***/
    if(ReceiveData.SingleOrMore == 0)							/***�����0***/
    {
        if(recvMsg_CAN1.id==0x7DF && recvMsg_CAN1.data[1]==0x3E && recvMsg_CAN1.data[2]>>7==1)
        {
            uint8_t HoldBuff[8]={0};
            HoldBuff[0] = recvMsg_CAN1.data[0];
            for(uint8_t i=1;i<HoldBuff[0]+1;i++)
                HoldBuff[i] = recvMsg_CAN1.data[i];
            dealwith3E_80(HoldBuff);
        }
        else
        {
            ReceiveData.Length = recvMsg_CAN1.data[0]&0x0F;				/***��0�ֽڵĵ�4λ���ǳ���***/
            ReceiveData.Service = recvMsg_CAN1.data[1];					/***��1�ֽھ��Ƿ�������***/
            for(uint8_t i=0;i<ReceiveData.Length-1;i++)
                ReceiveData.data[i] = recvMsg_CAN1.data[2+i];
            if(ReceiveData.Length>0 && ReceiveData.Length<8)
            {
                if(recvMsg_CAN1.id == 0x671)
                    FinishReceivePhysical = 1;
                else if(recvMsg_CAN1.id == 0x7DF)
                    FinishReceiveFunction = 1;
                MultiLost_time = 200;
            }
        }
    }
    else if(ReceiveData.SingleOrMore==1 && recvMsg_CAN1.id==0x671)						/***�����1***/
    {
        ReceiveData.Length = ((recvMsg_CAN1.data[0]&0x0F)<<8) + recvMsg_CAN1.data[1];	/***��0�ֽڵĵ�4λ�͵�һ���ֽھ������ݵĳ���***/
        ReceiveData.Service = recvMsg_CAN1.data[2];					/***��2�ֽھ��Ƿ�������***/
        for(uint8_t i=0;i<5;i++)
            ReceiveData.data[i] = recvMsg_CAN1.data[3+i];
        if(ReceiveData.Length>7 && ReceiveData.Length<150)/*waiting for the next message*/
        {
            ReceiveData.Number = 0;
            MultiLost_time = 0;
            SendData[0] = 0x30;
            SendData[1] = 0x00;
            SendData[2] = 0x00;
            SendData[3] = 0x00;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
        }
        else if(ReceiveData.Length>=150)
        {
            SendData[0] = 0x32;
            SendData[1] = 0x00;
            SendData[2] = 0x00;
            SendData[3] = 0x00;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);             /*******���������Ʊ���********/
        }
    }
    else if(ReceiveData.SingleOrMore==2 && recvMsg_CAN1.id==0x671 && MultiLost_time<150)//Data length < 120, 16*7=112
    {
        if((recvMsg_CAN1.data[0]&0x0F) == ReceiveData.Number+1)
{
            if(ReceiveData.Length-6-ReceiveData.Number*7 > 7)//waiting for the next message
            {
                for(uint8_t i=0;i<7;i++)
                    ReceiveData.data[i+5+ReceiveData.Number*7] = recvMsg_CAN1.data[1+i];
                MultiLost_time = 0;
            }
            else//finish of receiving data
            {
                for(uint8_t i=0;i<ReceiveData.Length-6-ReceiveData.Number*7;i++)
                    ReceiveData.data[i+5+ReceiveData.Number*7] = recvMsg_CAN1.data[1+i];
                FinishReceivePhysical = 1;
                MultiLost_time = 200;
            }
            ReceiveData.Number = recvMsg_CAN1.data[0]&0x0F;
        }
        else
        {
            MultiLost_time = 200;
        }
    }
    else if(ReceiveData.SingleOrMore==3 && recvMsg_CAN1.id==0x671 && FlowLost_time<90)
    {
        if((recvMsg_CAN1.data[0]&0x0F) == 0)
        {
            flowadd(recvMsg_CAN1.data[1], recvMsg_CAN1.data[2]);
            FlowLost_time = 0;
        }
        else if((recvMsg_CAN1.data[0]&0x0F) == 1)
        {
            FlowLost_time = 0;
        }
        else
        {
            FlowLost_time = 100;
        }
    }
    IRQ_CAN0_RX = 0;

    if(FinishReceivePhysical == 1)										/***����������ݽ��������***/
    {
        switch(ReceiveData.Service)
        {
        case 0x10:						/**********0x10 �޸�ϵͳ����״̬************/
            dealwith10(&ucFlagStatic,&ucFlagUnlockStatus,0,ReceiveData);
            break;
        case 0x11:         				        /**********0x11��λ*************************/
            dealwith11(ucFlagStatic,0,ReceiveData);
            break;
        case 0x14:                                              /***********0x14 �������********************/
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = ReceiveData.Service;
            SendData[3] = 0x7F;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            break;
        case 0x19:                                              /***********0x19 ���϶�ȡ********************/
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = ReceiveData.Service;
            SendData[3] = 0x7F;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            break;
        case 0x22:						/***********0x22 ͨ����ʶ��������************/
            dealwith22(ucFlagStatic,0,ReceiveData);
            break;
        case 0x27:						/**********0x27��ȫ��֤*********************/
            dealwith27(ucFlagStatic,&ucFlagUnlockStatus,ReceiveData);
            break;
        case 0x28:                                              /***********0x28 ͨѶ����*******************/
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = ReceiveData.Service;
            SendData[3] = 0x7F;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            break;
        case 0x2E:						/**********0x2Eͨ����ʶ��д����*************/
            dealwith2E(ucFlagStatic,ucFlagUnlockStatus,ReceiveData);
            break;
        case 0x31:						/**********0x31���г������*****************/
            dealwith31(ucFlagStatic,ucFlagUnlockStatus,ReceiveData);
            break;
        case 0x34:						/**********0x34��������*********************/
            dealwith34(ucFlagStatic,ucFlagUnlockStatus,ReceiveData);
            break;
        case 0x36:						/**********0x36��������*********************/
            dealwith36(ucFlagStatic,ReceiveData);
            break;
        case 0x37:						/**********0x37�����˳�����*****************/
            dealwith37(ucFlagStatic,ReceiveData);
            break;
        case 0x3E:						/**********0x3E��·����*********************/
            dealwith3E(ucFlagStatic,0,ReceiveData);
            break;
        case 0x85:                                              /**********0x85��Ͽ���*********************/
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = ReceiveData.Service;
            SendData[3] = 0x7F;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            break;
        default:
            SendData[0] = 0x03;
            SendData[1] = 0x7F;
            SendData[2] = ReceiveData.Service;
            SendData[3] = 0x11;
            SendData[4] = 0x00;
            SendData[5] = 0x00;
            SendData[6] = 0x00;
            SendData[7] = 0x00;
            flexcan_send_demo(handle1, 0x679, SendData, TX_MB_INDEX);
            break;
        }
        FinishReceivePhysical = 0;
    }
    else if(FinishReceiveFunction == 1)
    {
        switch(ReceiveData.Service)
        {
        case 0x10:						/**********0x10 �޸�ϵͳ����״̬************/
            dealwith10(&ucFlagStatic,&ucFlagUnlockStatus,1,ReceiveData);
            break;
        case 0x11:         				        /**********0x11��λ*************************/
            dealwith11(ucFlagStatic,1,ReceiveData);
            break;
        case 0x22:						/***********0x22 ͨ����ʶ��������************/
            dealwith22(ucFlagStatic,1,ReceiveData);
            break;
        case 0x3E:						/**********0x3E��·����*********************/
            dealwith3E(ucFlagStatic,1,ReceiveData);
            break;
        default:
            break;
        }
        FinishReceiveFunction = 0;
    }
}
}
