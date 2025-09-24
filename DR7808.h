/*
 * DR7808.h
 *
 *  Created on: 2025��8��21��
 *      Author: maode1
 */

#ifndef SERVES_INCLUDE_DR7808_H_
#define SERVES_INCLUDE_DR7808_H_
#include "Platform_Types.h"
/*
   DR7808 SPIÿ��֡��24bit�������

*/


#define DR7808_GENCTRL1 0x00
#define DR7808_GENCTRL2 0x01
#define DR7808_VDS1 0x02
#define DR7808_VDS2 0x03
#define DR7808_CCP_BLK1 0x04

#define DR7808_CCP_BLK2_ACT 0x05        //��Ҫ����reg bank =0
#define DR7808_CCP_BLK2_FW 0x05         //��Ҫ����reg bank =1

#define DR7808_HBMODE 0x06
#define DR7808_PWMSET 0x07
#define DR7808_TPRECHG 0x08
#define DR7808_HBIDIAG 0x09

#define DR7808_ST_ICHG 0x0A                     //��Ҫ����reg bank =0
#define DR7808_PWM_PCHG_INIT 0x0A               //��Ҫ����reg bank =1
#define DR7808_PWM_ICHG_ACT 0x0B                //��Ҫ����reg bank =0
#define DR7808_PWM_ICHG_FW 0x0B                 //��Ҫ����reg bank =1
#define DR7808_PWM_IDCHG_ACT 0x0C               //��Ҫ����reg bank =0
#define DR7808_PWM_PDCHG_INIT 0x0C              //��Ҫ����reg bank =1
#define DR7808_PWM_ICHGMAX_CCP_BLK3_ACT 0x0D    //��Ҫ����reg bank =0
#define DR7808_PWM_ICHGMAX_CCP_BLK3_FW 0x0D     //��Ҫ����reg bank =1


#define DR7808_TDON_OFF1 0x0E
#define DR7808_TDON_OFF2 0x0F
#define DR7808_TDON_OFF3 0x10
#define DR7808_GENSTAT 0x11
#define DR7808_DSOV 0x12
#define DR7808_HBVOUT_PWMERR 0x13
#define DR7808_EFF_TDON_OFF1 0x14
#define DR7808_EFF_TDON_OFF2 0x15
#define DR7808_EFF_TDON_OFF3 0x16
#define DR7808_TRISE_FALL1 0x17
#define DR7808_TRISE_FALL2 0x18
#define DR7808_TRISE_FALL3 0x19
#define DR7808_DEVID 0x1F
#define DR7808_DRV_LPWR_EN 0x1C
#define DR7808_CSA_OC_SH 0x1D
#define DR7808_MISC 0x1E

#define DR7808_GENSTAT 0x11
#define DR7808_DSOV 0x12
#define DR7808_HBVOUT_PWMERR 0x13
#define DR7808_EFF_TDON_OFF1 0x14
#define DR7808_EFF_TDON_OFF2 0x15
#define DR7808_EFF_TDON_OFF3 0x16
#define DR7808_TRISE_FALL 0x17
#define DR7808_TRISE_FALL2 0x18
#define DR7808_TRISE_FALL3 0x19
#define DR7808_GDF 0x1A
#define DR7808_DEVID 0x1F





//ע�� DR7808 ��ͬ�Ĵ����в�ͬBank
//Note DR7808 has different banks for different registers

// ͨ��ѡ������
typedef enum {
    DR7808_CH1 = (1 << 0),
    DR7808_CH2 = (1 << 1),
    DR7808_CH3 = (1 << 2),
    DR7808_CH4 = (1 << 3),
    DR7808_CH5 = (1 << 4),
    DR7808_CH6 = (1 << 5),
    DR7808_CH7 = (1 << 6),
    DR7808_CH8 = (1 << 7),
    DR7808_ALL_CH = 0xFF   // ����ͨ��
} DR7808_ChannelMask_t;

// DR7808 half bridge config
// DR7808 ��������
enum DR7808_HB_Mode{HighImpedance=0,LSn_ON=1,HSn_ON=2};
typedef struct{
  uint8_t HB8_Mode;
  uint8_t HB7_Mode;
  uint8_t HB6_Mode;
  uint8_t HB5_Mode;
  uint8_t HB4_Mode;
  uint8_t HB3_Mode;
  uint8_t HB2_Mode;
  uint8_t HB1_Mode;
}HBMODE_InitTypeDef;

// DR7808 PWM Setting
// DR7808 PWM ����
enum DR7808_PWM_HB{HB1=0,HB2=1,HB3=2,HB4=3,HB5=4,HB6=5,HB7=6,HB8=7};
typedef struct{
  uint8_t PWM1_HB;
  uint8_t PWM2_HB;
  uint8_t PWM3_HB;
  uint8_t PWM1_EN;
  uint8_t PWM2_EN;
  uint8_t PWM3_EN;
//Drain-Source monitoring in bridge passive mode
//©Դ������ű���ģʽ
  uint8_t PASS_VDS;
//Settings for bridge driver passive mode
//00B: LS1-4 are always off.
//Note: Changing PASS_MOD from
//00B: to any other value requires to clear DSOV1) first before writing PASS_MOD,
//01B: LS1-4 are always on (static brake).
//10B: LS1-4 are activated if passive VM OV is detected (overvoltage brake) (default).
//11B: LS1-4 are activated if passive VM OV is detected and PWM1 = High (overvoltage brake conditioned by PWM1).
  uint8_t PASS_Mode;
}PWMSET_InitTypeDef;

// half bridge Vds threshold Setting
// ����Vds��ֵ�趨
/*
    000B: 0.15V
    001B: 0.20V (default)
    010B: 0.25V
    011B: 0.30V
    100B: 0.40V
    101B: 0.50V
    110B: 0.60V
    111B: 2.0V
*/

enum DR7808_HB_VDSTh{Vdsth_15=0,Vdsth_20=1,Vdsth_25=2,Vdsth_30=3,
Vdsth_40=4,Vdsth_50=5,Vdsth_60=6,Vdsth2_0=7};
typedef struct{
  uint8_t HB8_VDSTh;
  uint8_t HB7_VDSTh;
  uint8_t HB6_VDSTh;
  uint8_t HB5_VDSTh;
  uint8_t HB4_VDSTh;
  uint8_t HB3_VDSTh;
  uint8_t HB2_VDSTh;
  uint8_t HB1_VDSTh;
}VDS_Vth_InitTypeDef;
// ��̬��������
// ���鿴 ��̬���ͷŵ�������
enum DR7808_ST_ICHG_Current{GateDriverCurrent1=0,GateDriverCurrent2=1,ICHGST1=0,ICHGST2=1,ICHGST3=2,ICHGST4=3,
ICHGST5=4,ICHGST6=5,ICHGST7=6,ICHGST8=7,ICHGST9=8,ICHGST10=9,ICHGST11=10
,ICHGST12=11,ICHGST13=12,ICHGST14=13,ICHGST15=14,ICHGST16=15,ICHGST_Default=4};
typedef struct{
  //������GateDriverCurrent1 ����GateDriverCurrent2
  uint8_t HB8_ICHG;
  uint8_t HB7_ICHG;
  uint8_t HB6_ICHG;
  uint8_t HB5_ICHG;
  uint8_t HB4_ICHG;
  uint8_t HB3_ICHG;
  uint8_t HB2_ICHG;
  uint8_t HB1_ICHG;
  //����Gate Driver Current 2������С һ��16����λ���ο�Static charge and discharge currents table
  uint8_t ICHGST_2;
  //����Gate Driver Current 1������С һ��16����λ���ο�Static charge and discharge currents table
  uint8_t ICHGST_1;
}ST_ICHG_InitTypeDef;


// CSA �������
enum DR7808_CSA_Config{
//  CSA1_Gain and CSA2_Gain
  Gain10=0,Gain20=1,Gain40=2,Gain80=3,
//  CSA1_Direction and CSA2_Direction
CSA_Unidirectional=0,CSA_Bidirectional=1,
// OCTH1 and OCTH2
OCThLevel1=0,OCThLevel2=1,
OCThLevel3=2,OCThLevel4=3,
// CSA1_SEL and CSA2_SEL
Con2HB1=0,Con2HB2=1,Con2HB3=2,Con2HB4=3,
Con2HB5=4,Con2HB6=5,Con2HB7=6,Con2HB8=7};
typedef struct{
  //����CSA���� CSA_Bidirectional ���� CSA_Unidirectional
  //����˫����ߵ���
  uint8_t CSA1_Direction;
  uint8_t CSA2_Direction;
  // ���� CSA Gain�Ĵ�С 10,20,40,80
  uint8_t CSA1_Gain;
  uint8_t CSA2_Gain;
  // ����OCP�Ƿ���
  uint8_t OverCurrentProtect;
  //����CSA �ڸ߱߻��ߵױ�
  //Configure CSA on the high-side or the low-side
  uint8_t CSA1_Level;
  uint8_t CSA2_Level;
  //���ù����˲�ʱ��
  //Overcurrent filter time
  uint8_t OC1FILT;
  uint8_t OC2FILT;
  //�ر�CSA
  //DisEnable CSA ����Ϊ1
  //Enable CSA ����Ϊ0
  uint8_t CSA1_OFF;
  uint8_t CSA2_OFF;
  // Overcurrent detection threshold
  // ���������ֵ����
  uint8_t OCTH1;
  uint8_t OCTH2;
//  Overcurrent detection threshold of SO1 with CSD1 = 0
//00B: VSO1 > VDD / 2 (default)
//01B: VSO1 > VDD / 2 + VDD / 10
//10B: VSO1 > VDD / 2 + 2 x VDD / 10
//11B: VSO1 > VDD / 2 + 3x VDD / 10
//Overcurrent detection threshold of SO1 with CSD1 = 1
//00B: VSO1 > VDD / 2+2 x VDD / 20 or VCSOx < VDD / 2 �C 2 �� VDD / 20 (default)
//01B: VSO1 > VDD / 2+ 4x VDD / 20 or VCSOx < VDD / 2 �C 4 �� VDD / 20
//10B: VSO1 > VDD / 2+ 5 x VDD / 20 or VCSOx < VDD / 2 ? 5 �� VDD / 20
//11B: VSO1 > VDD / 2+ 6x VDD / 20 or VCSOx < VDD / 2 �C 6 �� VDD / 20
  uint8_t CSA_BLK_SEL;
//1: CSA1 and CSA2 hold blank time is tccp + tblank/2
//0��CSA1 and CSA2 hold blank time is tccp + tblank(default)
// blank time ����PWM ���ƹ���ʱ������
  uint8_t CSA2_SH_EN;
  uint8_t CSA1_SH_EN;
//ѡ���Ƿ�ʹ��PWM ���ƹ���
//Select whether to use PWM suppression
  uint8_t OC_SEP_EN;
// ѡ�� OCEN�����ֻ��CSA���ӵİ���turn off �������а���Turn off
  uint8_t CSA2_SEL;
  uint8_t CSA1_SEL;
// CSA SEL����CSA���ӵİ���ѡ��

}CSA_InitTypeDef;

// ���ѡ��ö�٣���ߴ���ɶ���
typedef enum {
    MOTOR_NONE = 0,
    MOTOR_1,  // ʹ��CH1(HS)��CH2(LS)
    MOTOR_2,  // ʹ��CH1(HS)��CH3(LS)
    MOTOR_3,  // ʹ��CH1(HS)��CH4(LS)
    MOTOR_4,  // ʹ��CH5(HS)��CH6(LS)
    MOTOR_5,  // ʹ��CH5(HS)��CH7(LS)
    MOTOR_6,  // ʹ��CH5(HS)��CH8(LS)
    MOTOR_ALL_OFF
} Motor_Selection_t;

// �������ö��
typedef enum {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_REVERSE,
    MOTOR_DIR_BRAKE
} Motor_Direction_t;

// ������ýṹ��
typedef struct {
    DR7808_ChannelMask_t hs_channel;
    DR7808_ChannelMask_t ls_channel;
    const char *name; // ������ƣ����ڵ���
} Motor_Config_t;

// ������ñ�
static const Motor_Config_t motor_config[] = {
    [MOTOR_1] = {DR7808_CH1, DR7808_CH4, "Motor 1"},
    [MOTOR_2] = {DR7808_CH2, DR7808_CH4, "Motor 2"},
    [MOTOR_3] = {DR7808_CH3, DR7808_CH4, "Motor 3"},
    [MOTOR_4] = {DR7808_CH5, DR7808_CH8, "Motor 4"},
    [MOTOR_5] = {DR7808_CH6, DR7808_CH8, "Motor 5"},
    [MOTOR_6] = {DR7808_CH7, DR7808_CH8, "Motor 6"}
};



/**
 * @brief DR7808��ʼ������
 * @return ��ʼ��״̬����
 * @note ִ��������оƬ��ʼ������
 */
uint8_t DR7808_init(void);

/**
  * @brief  ��ȫ�ĵ�����ƣ�������������ֹͬʱ������������
  * @param  motor: Ҫ���Ƶĵ��
  * @param  direction: �������ֹͣ����ת����ת��ɲ����
  * @param  RData: ���ڽ���SPI�ض����ݵĻ�����
  * @retval bool: ���óɹ�����true��ʧ�ܷ���false
  */
bool Motor_Control_Safe(Motor_Selection_t motor, Motor_Direction_t direction, uint8_t *RData);

#endif /* SERVES_INCLUDE_DR7808_H_ */
