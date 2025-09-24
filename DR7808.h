/*
 * DR7808.h
 *
 *  Created on: 2025年8月21日
 *      Author: maode1
 */

#ifndef SERVES_INCLUDE_DR7808_H_
#define SERVES_INCLUDE_DR7808_H_
#include "Platform_Types.h"
/*
   DR7808 SPI每个帧是24bit数据组成

*/


#define DR7808_GENCTRL1 0x00
#define DR7808_GENCTRL2 0x01
#define DR7808_VDS1 0x02
#define DR7808_VDS2 0x03
#define DR7808_CCP_BLK1 0x04

#define DR7808_CCP_BLK2_ACT 0x05        //需要配置reg bank =0
#define DR7808_CCP_BLK2_FW 0x05         //需要配置reg bank =1

#define DR7808_HBMODE 0x06
#define DR7808_PWMSET 0x07
#define DR7808_TPRECHG 0x08
#define DR7808_HBIDIAG 0x09

#define DR7808_ST_ICHG 0x0A                     //需要配置reg bank =0
#define DR7808_PWM_PCHG_INIT 0x0A               //需要配置reg bank =1
#define DR7808_PWM_ICHG_ACT 0x0B                //需要配置reg bank =0
#define DR7808_PWM_ICHG_FW 0x0B                 //需要配置reg bank =1
#define DR7808_PWM_IDCHG_ACT 0x0C               //需要配置reg bank =0
#define DR7808_PWM_PDCHG_INIT 0x0C              //需要配置reg bank =1
#define DR7808_PWM_ICHGMAX_CCP_BLK3_ACT 0x0D    //需要配置reg bank =0
#define DR7808_PWM_ICHGMAX_CCP_BLK3_FW 0x0D     //需要配置reg bank =1


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





//注意 DR7808 不同寄存器有不同Bank
//Note DR7808 has different banks for different registers

// 通道选择掩码
typedef enum {
    DR7808_CH1 = (1 << 0),
    DR7808_CH2 = (1 << 1),
    DR7808_CH3 = (1 << 2),
    DR7808_CH4 = (1 << 3),
    DR7808_CH5 = (1 << 4),
    DR7808_CH6 = (1 << 5),
    DR7808_CH7 = (1 << 6),
    DR7808_CH8 = (1 << 7),
    DR7808_ALL_CH = 0xFF   // 所有通道
} DR7808_ChannelMask_t;

// DR7808 half bridge config
// DR7808 半桥配置
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
// DR7808 PWM 设置
enum DR7808_PWM_HB{HB1=0,HB2=1,HB3=2,HB4=3,HB5=4,HB6=5,HB7=6,HB8=7};
typedef struct{
  uint8_t PWM1_HB;
  uint8_t PWM2_HB;
  uint8_t PWM3_HB;
  uint8_t PWM1_EN;
  uint8_t PWM2_EN;
  uint8_t PWM3_EN;
//Drain-Source monitoring in bridge passive mode
//漏源监控在桥被动模式
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
// 半桥Vds阈值设定
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
// 静态电流配置
// 详情看 静态充电和放电电流表格
enum DR7808_ST_ICHG_Current{GateDriverCurrent1=0,GateDriverCurrent2=1,ICHGST1=0,ICHGST2=1,ICHGST3=2,ICHGST4=3,
ICHGST5=4,ICHGST6=5,ICHGST7=6,ICHGST8=7,ICHGST9=8,ICHGST10=9,ICHGST11=10
,ICHGST12=11,ICHGST13=12,ICHGST14=13,ICHGST15=14,ICHGST16=15,ICHGST_Default=4};
typedef struct{
  //配置是GateDriverCurrent1 还是GateDriverCurrent2
  uint8_t HB8_ICHG;
  uint8_t HB7_ICHG;
  uint8_t HB6_ICHG;
  uint8_t HB5_ICHG;
  uint8_t HB4_ICHG;
  uint8_t HB3_ICHG;
  uint8_t HB2_ICHG;
  uint8_t HB1_ICHG;
  //配置Gate Driver Current 2电流大小 一共16个挡位，参考Static charge and discharge currents table
  uint8_t ICHGST_2;
  //配置Gate Driver Current 1电流大小 一共16个挡位，参考Static charge and discharge currents table
  uint8_t ICHGST_1;
}ST_ICHG_InitTypeDef;


// CSA 相关配置
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
  //配置CSA方向 CSA_Bidirectional 或者 CSA_Unidirectional
  //配置双向或者单向
  uint8_t CSA1_Direction;
  uint8_t CSA2_Direction;
  // 配置 CSA Gain的大小 10,20,40,80
  uint8_t CSA1_Gain;
  uint8_t CSA2_Gain;
  // 配置OCP是否开启
  uint8_t OverCurrentProtect;
  //配置CSA 在高边或者底边
  //Configure CSA on the high-side or the low-side
  uint8_t CSA1_Level;
  uint8_t CSA2_Level;
  //配置过流滤波时间
  //Overcurrent filter time
  uint8_t OC1FILT;
  uint8_t OC2FILT;
  //关闭CSA
  //DisEnable CSA 设置为1
  //Enable CSA 设置为0
  uint8_t CSA1_OFF;
  uint8_t CSA2_OFF;
  // Overcurrent detection threshold
  // 过流检测阈值设置
  uint8_t OCTH1;
  uint8_t OCTH2;
//  Overcurrent detection threshold of SO1 with CSD1 = 0
//00B: VSO1 > VDD / 2 (default)
//01B: VSO1 > VDD / 2 + VDD / 10
//10B: VSO1 > VDD / 2 + 2 x VDD / 10
//11B: VSO1 > VDD / 2 + 3x VDD / 10
//Overcurrent detection threshold of SO1 with CSD1 = 1
//00B: VSO1 > VDD / 2+2 x VDD / 20 or VCSOx < VDD / 2 – 2 × VDD / 20 (default)
//01B: VSO1 > VDD / 2+ 4x VDD / 20 or VCSOx < VDD / 2 – 4 × VDD / 20
//10B: VSO1 > VDD / 2+ 5 x VDD / 20 or VCSOx < VDD / 2 ? 5 × VDD / 20
//11B: VSO1 > VDD / 2+ 6x VDD / 20 or VCSOx < VDD / 2 – 6 × VDD / 20
  uint8_t CSA_BLK_SEL;
//1: CSA1 and CSA2 hold blank time is tccp + tblank/2
//0：CSA1 and CSA2 hold blank time is tccp + tblank(default)
// blank time 用于PWM 抑制功能时间设置
  uint8_t CSA2_SH_EN;
  uint8_t CSA1_SH_EN;
//选择是否使用PWM 抑制功能
//Select whether to use PWM suppression
  uint8_t OC_SEP_EN;
// 选择 OCEN情况下只有CSA连接的半桥turn off 还是所有半桥Turn off
  uint8_t CSA2_SEL;
  uint8_t CSA1_SEL;
// CSA SEL根据CSA连接的半桥选择

}CSA_InitTypeDef;

// 电机选择枚举，提高代码可读性
typedef enum {
    MOTOR_NONE = 0,
    MOTOR_1,  // 使用CH1(HS)和CH2(LS)
    MOTOR_2,  // 使用CH1(HS)和CH3(LS)
    MOTOR_3,  // 使用CH1(HS)和CH4(LS)
    MOTOR_4,  // 使用CH5(HS)和CH6(LS)
    MOTOR_5,  // 使用CH5(HS)和CH7(LS)
    MOTOR_6,  // 使用CH5(HS)和CH8(LS)
    MOTOR_ALL_OFF
} Motor_Selection_t;

// 电机方向枚举
typedef enum {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_REVERSE,
    MOTOR_DIR_BRAKE
} Motor_Direction_t;

// 电机配置结构体
typedef struct {
    DR7808_ChannelMask_t hs_channel;
    DR7808_ChannelMask_t ls_channel;
    const char *name; // 电机名称，用于调试
} Motor_Config_t;

// 电机配置表
static const Motor_Config_t motor_config[] = {
    [MOTOR_1] = {DR7808_CH1, DR7808_CH4, "Motor 1"},
    [MOTOR_2] = {DR7808_CH2, DR7808_CH4, "Motor 2"},
    [MOTOR_3] = {DR7808_CH3, DR7808_CH4, "Motor 3"},
    [MOTOR_4] = {DR7808_CH5, DR7808_CH8, "Motor 4"},
    [MOTOR_5] = {DR7808_CH6, DR7808_CH8, "Motor 5"},
    [MOTOR_6] = {DR7808_CH7, DR7808_CH8, "Motor 6"}
};



/**
 * @brief DR7808初始化函数
 * @return 初始化状态代码
 * @note 执行完整的芯片初始化序列
 */
uint8_t DR7808_init(void);

/**
  * @brief  安全的电机控制（互锁保护，防止同时开启多个电机）
  * @param  motor: 要控制的电机
  * @param  direction: 电机方向（停止、正转、反转、刹车）
  * @param  RData: 用于接收SPI回读数据的缓冲区
  * @retval bool: 配置成功返回true，失败返回false
  */
bool Motor_Control_Safe(Motor_Selection_t motor, Motor_Direction_t direction, uint8_t *RData);

#endif /* SERVES_INCLUDE_DR7808_H_ */
