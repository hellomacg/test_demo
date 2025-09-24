/*
 * DR7808.c
 *
 *  Created on: 2025年8月21日
 *      Author: maode1
 */

/* Includes */
#include "DR7808.h"
#include "Siul2_Port_Ip.h"
#include "Siul2_Dio_Ip.h"
#include "Lpspi_Ip.h"
/* User includes */
/* Time to transfer all frame data */
#define TIMEOUT             ((uint32)1000000UL)
#define NUMBER_OF_BYTES     (10)

// 通道工作模式定义
#define DR7808_CH_OFF     0x00  // 通道关闭
#define DR7808_CH_HS_ON   0x01  // 高边开启 (GH高)
#define DR7808_CH_LS_ON   0x02  // 低边开启 (GL高)
/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/
#define MASTER_EXTERNAL_DEVICE_7808          	(Lpspi_Ip_DeviceAttributes_SpiExternalDevice_1_Instance_1)
#define SLAVE_EXTERNAL_DEVICE           	4//(Lpspi_Ip_DeviceAttributes_SpiExternalDevice_Slave_Instance_4)

void Lpspi_Read(uint8_t Length, const uint8 *TxBuffer, uint8 *RxBuffer)
{
    Lpspi_Ip_SyncTransmit(&MASTER_EXTERNAL_DEVICE_7808, TxBuffer, RxBuffer, Length, TIMEOUT);
}

void DR7808_delay(uint32_t count)
{
    volatile uint32_t i = 0;
    for (i = 0; i < count; ++i)
    {
        __asm("NOP"); /* 调用nop空指令 */
         __asm("NOP"); /* 调用nop空指令 */
          __asm("NOP"); /* 调用nop空指令 */
           __asm("NOP"); /* 调用nop空指令 */
            __asm("NOP"); /* 调用nop空指令 */
    }
}
//获取寄存器信息
void DR7808_Read_Reg(uint8_t Reg,uint8_t *RData)
{
  uint8_t TxData[3]={0X00,0X00,0X00};
  //读取寄存器
  TxData[0]=(Reg<<1)|0xC0;
  Lpspi_Read(3,TxData,RData);
}
//读取与清除寄存器信息
void DR7808_Read_Clear_Reg(uint8_t Reg,uint8_t *RData)
{
  uint8_t TxData[3]={0X00,0X00,0X00};
  //读取寄存器
  TxData[0]=(Reg<<1)|0xC1;
  Lpspi_Read(3,TxData,RData);
}
//写寄存器信息
void DR7808_Write_Reg(uint8_t Reg,uint16_t WData,uint8_t *RData)
{
  uint8_t TxData[3]={0X00,0X00,0X00};
  //读取寄存器&写入寄存器
  TxData[0]=(Reg<<1)|0xC1;
  TxData[1]=WData>>8;
  TxData[2]=WData&0x00FF;
  Lpspi_Read(3,TxData,RData);
}
// dr7808 退出FS Mode
void DR7808_Quit_FS(void)
{
  uint8_t Data[3]={0X00,0X00,0X00};
  //读取并且清除Gensata
  DR7808_Read_Clear_Reg(DR7808_GENSTAT,Data);
  DR7808_delay(2000);
  //看门狗位置1
  DR7808_Write_Reg(DR7808_GENCTRL1,0x0027,Data);
  DR7808_delay(2000);
  //看门狗位置0
  DR7808_Write_Reg(DR7808_GENCTRL1,0x0026,Data);
  DR7808_delay(2000);
}
// dr7808 禁用看门狗
void DR7808_Dis_WD(void)
{
  uint8_t Data[3];
  DR7808_delay(2000);
  //看门狗Unlock 置1
  DR7808_Write_Reg(DR7808_GENCTRL1,0x00A7,Data);
  DR7808_delay(2000);
  //禁用看门狗
  DR7808_Write_Reg(DR7808_GENCTRL2,0x4380,Data);
}
// dr7808桥壁配置
void DR7808_Half_Bridge_Mode(HBMODE_InitTypeDef* config,uint8_t *RData)
{
  uint16_t WData=0;
  WData|=(config->HB8_Mode)<<14;
  WData|=(config->HB7_Mode)<<12;
  WData|=(config->HB6_Mode)<<10;
  WData|=(config->HB5_Mode)<<8;
  WData|=(config->HB4_Mode)<<6;
  WData|=(config->HB3_Mode)<<4;
  WData|=(config->HB2_Mode)<<2;
  WData|=(config->HB1_Mode);
  DR7808_Write_Reg(DR7808_HBMODE,WData,RData);
}
// 未测试
// dr7808 PWM 通道配置
void DR7808_PWMSET_Channel(PWMSET_InitTypeDef* config,uint8_t *RData)
{
  uint16_t WData=0;
  WData|=(config->PWM1_HB)<<1;
  WData|=(config->PWM2_HB)<<5;
  WData|=(config->PWM3_HB)<<9;
  WData|=(config->PWM1_EN);
  WData|=(config->PWM2_EN)<<4;
  WData|=(config->PWM3_EN)<<8;
  WData|=(config->PASS_VDS)<<14;
  WData|=(config->PASS_Mode)<<12;
  DR7808_Write_Reg(DR7808_PWMSET,WData,RData);

}
//未测试
//VDS Monitor threshold setting HB1~HB4
void DR7808_VDS_Monitoring_1_4(VDS_Vth_InitTypeDef* config,uint8_t *RData)
{
  uint16_t WData=0;
  WData|=(config->HB1_VDSTh);
  WData|=(config->HB2_VDSTh)<<3;
  WData|=(config->HB3_VDSTh)<<6;
  WData|=(config->HB4_VDSTh)<<9;
  DR7808_Write_Reg(DR7808_VDS1,WData,RData);
}
//未测试
//VDS Monitor threshold setting HB5~HB8
void DR7808_VDS_Monitoring_5_8(VDS_Vth_InitTypeDef* config,uint8_t *RData)
{
  uint16_t WData=0;
  WData|=(config->HB5_VDSTh);
  WData|=(config->HB6_VDSTh)<<3;
  WData|=(config->HB7_VDSTh)<<6;
  WData|=(config->HB8_VDSTh)<<9;
  DR7808_Write_Reg(DR7808_VDS2,WData,RData);
}
// dr7808喂狗操作
void DR7808_Feed_Dog(uint8_t *RData)
{
  uint16_t WData=0;
  uint8_t Data[3];
  //读取看门狗位
  DR7808_Read_Reg(DR7808_GENCTRL1,Data);
  DR7808_delay(20);
  //如果看门狗位为1
  if((Data[2]&0x01)==1)
  {
    //看门狗位置0并写入
    Data[2]&=0XFE;
    WData|=Data[1]<<8;
    WData|=Data[2];
    DR7808_Write_Reg(DR7808_GENCTRL1,WData,RData);
  }
  else
  {
    //看门狗位置1并写入
    Data[2]|=0X01;
    WData|=Data[1]<<8;
    WData|=Data[2];
    DR7808_Write_Reg(DR7808_GENCTRL1,WData,RData);
  }
}
// dr7808 csa相关配置
void DR7808_CSA_Conifg(CSA_InitTypeDef* config,uint8_t *RData)
{
  uint16_t WData=0;
  uint8_t Data[3];
  //读取GENCTRL1配置
  DR7808_Read_Reg(DR7808_GENCTRL1,Data);
  WData|=(config->CSA2_Direction)<<15;
  WData|=(config->CSA2_Gain)<<13;
  WData|=(config->CSA1_Direction)<<12;
  WData|=(config->CSA1_Gain)<<10;
  //接收值后十位不变
  WData|=(Data[1]&0x03)<<8;
  WData|=Data[2];
  DR7808_Write_Reg(DR7808_GENCTRL1,WData,RData);
  //清空WData
  WData=0;
  DR7808_delay(20);
  DR7808_Read_Reg(DR7808_HBIDIAG,Data);
  WData|=Data[2];
  WData|=(config->CSA2_Level)<<15;
  WData|=(config->CSA1_Level)<<14;
  WData|=(config->OC2FILT)<<12;
  WData|=(config->OC1FILT)<<10;
  WData|=(config->CSA2_OFF)<<9;
  WData|=(config->CSA1_OFF)<<8;
  DR7808_Write_Reg(DR7808_HBIDIAG,WData,RData);
    //清空WData
  WData=0;
  DR7808_delay(20);
   //读取GENCTRL2配置
  DR7808_Read_Reg(DR7808_GENCTRL2,Data);
  //清空后四位
  Data[2]&=0XF0;
  //高8位不变
  WData|=Data[1]<<8;
  WData|=Data[2];
  WData|=(config->OCTH2)<<2;
  WData|=(config->OCTH1);
  DR7808_Write_Reg(DR7808_GENCTRL2,WData,RData);
  //清空WData
  WData=0;
  DR7808_delay(20);
  //读取GENCTRL2配置
  DR7808_Read_Reg(DR7808_CSA_OC_SH,Data);
  WData=0;
  //最高位PWM不变
  WData=(Data[1]&0X80)<<8;
  WData|=(config->CSA_BLK_SEL)<<9;
  WData|=(config->CSA2_SH_EN)<<8;
  WData|=(config->CSA1_SH_EN)<<7;
  WData|=(config->OC_SEP_EN)<<6;
  WData|=(config->CSA2_SEL)<<3;
  WData|=(config->CSA1_SEL);
  //写入过流数据
  DR7808_Write_Reg(DR7808_CSA_OC_SH,WData,RData);
}
// 需要配置Reg Bank
// 静态充放电电流设置
void DR7808_ST_ICHG_Conifg(ST_ICHG_InitTypeDef* config,uint8_t *RData)
{
  uint16_t WData=0;
  uint8_t Data[3];
 // 读取DR7808_GENCTRL1配置 并将REG BANK 其配置为0
  DR7808_Read_Reg(DR7808_GENCTRL1,Data);
  Data[1]=Data[1]&0xfd;
  WData=Data[1]<<8;
  WData|=Data[2];
  //写入reg bank
  DR7808_Write_Reg(DR7808_GENCTRL1,WData,RData);
  DR7808_delay(20);
  WData=0;
  //设置是电流1还是电流2
  WData|=(config->HB8_ICHG)<<15;
  WData|=(config->HB7_ICHG)<<14;
  WData|=(config->HB6_ICHG)<<13;
  WData|=(config->HB5_ICHG)<<12;
  WData|=(config->HB4_ICHG)<<11;
  WData|=(config->HB3_ICHG)<<10;
  WData|=(config->HB2_ICHG)<<9;
  WData|=(config->HB1_ICHG)<<8;
  //设置电流挡位2
  WData|=(config->ICHGST_2)<<4;
  //设置电流挡位1
  WData|=(config->ICHGST_1);
  //写入电流挡位
  DR7808_Write_Reg(DR7808_ST_ICHG,WData,RData);
}
// dr7808 ID获取
uint8_t DR7808_GetChipID(void)
{
  uint8_t Data[3];
  DR7808_Read_Reg(DR7808_DEVID,Data);
  return Data[2];
}
// dr7808 ID获取
uint8_t DR7808_GetTEST(void)
{
  uint8_t Data[3];
  DR7808_Read_Reg(DR7808_GENCTRL1,Data);
  return Data[2];
}
//读取所有寄存器
void Get_All_Reg(uint8_t* RData)
{
  uint8_t i=0;
  for(i=0;i<0x1f;i++)
  {
    DR7808_Read_Reg(i,RData);
    RData++;
    DR7808_delay(330);
  }

}

/**
  * @brief  设置DR7808Q任意通道的工作模式
  * @param  hs_channels: 要设置为高边开启(GH高)的通道掩码
  * @param  ls_channels: 要设置为低边开启(GL高)的通道掩码
  * @param  RData: 用于接收SPI回读数据的缓冲区
  * @retval bool: 配置成功返回true，失败返回false
  * @note   同一个通道不能同时在hs_channels和ls_channels中设置
  */
bool DR7808_SetChannels(uint16_t hs_channels, uint16_t ls_channels, uint8_t *RData)
{
    HBMODE_InitTypeDef config;

    // 安全检查：确保没有通道冲突
    if (hs_channels & ls_channels) {
        // 有通道冲突，不能同时设置为HS和LS
        return false;
    }

    // 配置每个通道的模式
    config.HB1_Mode = (hs_channels & DR7808_CH1) ? DR7808_CH_HS_ON :
                     (ls_channels & DR7808_CH1) ? DR7808_CH_LS_ON : DR7808_CH_OFF;

    config.HB2_Mode = (hs_channels & DR7808_CH2) ? DR7808_CH_HS_ON :
                     (ls_channels & DR7808_CH2) ? DR7808_CH_LS_ON : DR7808_CH_OFF;

    config.HB3_Mode = (hs_channels & DR7808_CH3) ? DR7808_CH_HS_ON :
                     (ls_channels & DR7808_CH3) ? DR7808_CH_LS_ON : DR7808_CH_OFF;

    config.HB4_Mode = (hs_channels & DR7808_CH4) ? DR7808_CH_HS_ON :
                     (ls_channels & DR7808_CH4) ? DR7808_CH_LS_ON : DR7808_CH_OFF;

    config.HB5_Mode = (hs_channels & DR7808_CH5) ? DR7808_CH_HS_ON :
                     (ls_channels & DR7808_CH5) ? DR7808_CH_LS_ON : DR7808_CH_OFF;

    config.HB6_Mode = (hs_channels & DR7808_CH6) ? DR7808_CH_HS_ON :
                     (ls_channels & DR7808_CH6) ? DR7808_CH_LS_ON : DR7808_CH_OFF;

    config.HB7_Mode = (hs_channels & DR7808_CH7) ? DR7808_CH_HS_ON :
                     (ls_channels & DR7808_CH7) ? DR7808_CH_LS_ON : DR7808_CH_OFF;

    config.HB8_Mode = (hs_channels & DR7808_CH8) ? DR7808_CH_HS_ON :
                     (ls_channels & DR7808_CH8) ? DR7808_CH_LS_ON : DR7808_CH_OFF;

     DR7808_Half_Bridge_Mode(&config, RData);
}

/**
  * @brief  控制指定电机的方向和状态
  * @param  motor: 要控制的电机
  * @param  direction: 电机方向（停止、正转、反转、刹车）
  * @param  RData: 用于接收SPI回读数据的缓冲区
  * @retval bool: 配置成功返回true，失败返回false
  */
bool Motor_Control(Motor_Selection_t motor, Motor_Direction_t direction, uint8_t *RData)
{
    if (motor <= MOTOR_NONE || motor > MOTOR_6) {
        return DR7808_SetChannels(0, 0, RData); // 无效电机，关闭所有
    }

    const Motor_Config_t *config = &motor_config[motor];

    switch (direction) {
        case MOTOR_DIR_FORWARD:
            // 正转：HS在A相，LS在B相
            return DR7808_SetChannels(config->hs_channel, config->ls_channel, RData);

        case MOTOR_DIR_REVERSE:
            // 反转：HS在B相，LS在A相（交换通道）
            return DR7808_SetChannels(config->ls_channel, config->hs_channel, RData);

        case MOTOR_DIR_BRAKE:
            // 刹车：两个LS都开启或两个HS都开启
            return DR7808_SetChannels(0, config->hs_channel | config->ls_channel, RData);

        case MOTOR_DIR_STOP:
        default:
            // 停止：关闭相关通道
            return DR7808_SetChannels(0, 0, RData);
    }
}
uint8_t ChipID_test;
/**
 * @brief DR7808初始化函数
 * @return 初始化状态代码
 * @note 执行完整的芯片初始化序列
 */
uint8_t Data_test[3]={0xff,0xff,0xff};
uint8_t ChipID;
uint8_t DR7808_init(void)
{

	Siul2_Dio_Ip_WritePin(SEPIC_EN_PORT, SEPIC_EN_PIN, 1u);
	Siul2_Dio_Ip_WritePin(Driver_CP_EN_PORT, Driver_CP_EN_PIN, 1u);
	Siul2_Dio_Ip_WritePin(Driver_EN_PORT, Driver_EN_PIN, 1u);
	 DR7808_delay(2000);
	/* 初始化SPI 和IIC*/
	Lpspi_Ip_Init(&Lpspi_Ip_PhyUnitConfig_SpiPhyUnit_1_Instance_1);

	/* 定义SPI帧大小为8Bytes */
	Lpspi_Ip_UpdateFrameSize(&MASTER_EXTERNAL_DEVICE_7808, 8U);

	/* 定义SPI为MSB模式 */
	Lpspi_Ip_UpdateLsb(&MASTER_EXTERNAL_DEVICE_7808, FALSE);


	  DR7808_Read_Reg(DR7808_PWMSET,Data_test);
	//DR7808_Quit_FS();
	//DR7808_Dis_WD();
	ChipID = DR7808_GetChipID();
	 DR7808_delay(2000);
	if(ChipID == 0x01)
	{
		return true;
	}
	else
	{
		return false;
	}
}

// 当前活动的电机状态
static Motor_Selection_t current_active_motor = MOTOR_NONE;

/**
  * @brief  安全的电机控制（互锁保护，防止同时开启多个电机）
  * @param  motor: 要控制的电机
  * @param  direction: 电机方向（停止、正转、反转、刹车）
  * @param  RData: 用于接收SPI回读数据的缓冲区
  * @retval bool: 配置成功返回true，失败返回false
  */
bool Motor_Control_Safe(Motor_Selection_t motor, Motor_Direction_t direction, uint8_t *RData)
{
    // 如果请求停止或者请求的电机不是当前活动的电机，先停止当前电机
    if (direction == MOTOR_DIR_STOP || motor != current_active_motor) {
        Motor_Control(current_active_motor, MOTOR_DIR_STOP, RData);
        current_active_motor = MOTOR_NONE;
    }

    // 如果请求的是停止，就直接返回
    if (direction == MOTOR_DIR_STOP) {
        return true;
    }

    // 控制新电机
    bool result = Motor_Control(motor, direction, RData);
    if (result) {
        current_active_motor = motor;
    }

    return result;
}
