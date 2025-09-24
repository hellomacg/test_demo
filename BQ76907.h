/*
 * BQ76907.h
 *
 *  Created on: 2025年8月7日
 *      Author: maode1
 */

#ifndef BQ76907_H_
#define BQ76907_H_
#include "Platform_Types.h"


#define Bq76907_I2cInst	0u
#define Bq76907_ADD		(0X08)

#define R               0       //Read
#define W               1       //Write
#define W2              2       //write data with two bytes
#define R2              3       //Read data with two bytes
#define Vcell_Mode            5
#define Cell_Overvoltage      3200
#define Charge_Threshold      8
#define FET_Options           0x8C
#define Protections_A         0xBD
#define Protections_B         0x32
#define DSG_ProtectionsA      0x75
#define CHG_ProtectionsA      0xE5
#define Both_FET_Protections  0x02
#define Short_Circuit         0x01
#define Short_Circuit_Delay   0x0A
/**
 * @brief 读取指定电芯的电压值
 * @param cell 电芯编号（通常为0~N-1，取决于电池组串联数）
 * @return uint16_t 电压值（单位为mV，具体分辨率需参考器件手册）
 *                 返回0xFFFF表示读取失败
 */
int16_t ReadCellVoltage(uint8_t cell);

/**
 * @brief 读取内部温度传感器值（单位：0.1°C）
 * @return int16_t 温度值（范围：-400~1250，对应-40.0°C~125.0°C）
 *                 返回0x8000表示读取失败
 */
int16_t Read_Int_Temperature(void);

/**
 * @brief 读取CC1电流值（单位：mA）
 * @return int16_t 电流值（正值表示充电，负值表示放电）
 *                返回0x8000表示读取失败
 * @note 假设：
 *       1. 寄存器0x3C返回16位有符号补码
 *       2. 电流分辨率：1mA/LSB
 *       3. 量程：±32767mA
 */
int16_t Read_CC1_Current(void);

/**
 * @brief 读取CC2电流值（单位：mA）
 * @return int16_t 电流值（正值表示充电，负值表示放电）
 *                返回0x8000表示读取失败
 * @note 假设：
 *       1. 寄存器0x3C返回16位有符号补码
 *       2. 电流分辨率：1mA/LSB
 *       3. 量程：±32767mA
 */
int16_t Read_CC2_Current(void);


/*16-bit voltage on top of stack*/
uint16_t Stack_Voltage(void);


uint8_t Safety_StatusA(void);

uint8_t Safety_StatusB(void);

uint8_t Safety_AlertA(void);

uint8_t Safety_AlertB(void);

uint16_t Alarm_Status(void);

uint16_t Alarm_Raw_Status(void);

uint16_t Battery_Status(void);

/**
 * @brief 开启指定电芯的均衡功能
 * @param Cell 电芯选择位掩码：
 *             0x00 - 禁止均衡
 *             0x02 - 电芯1
 *             0x04 - 电芯2
 *             0x06 - 电芯3
 *             ...（支持多电芯组合，如0x07表示电芯1+2+3）
 * @return true  操作成功
 * @return false 操作失败
 */
bool Cell_Balancing(uint8_t Cell);

/**
 * @brief 检查已开启均衡功能的电芯
 * @return 电芯编号
 */
uint8_t Cell_Balancing_cbk(void);

/**
 * @brief 允许或禁止休眠
 * @param enable 控制标志: true=允许休眠, false=禁止休眠
 * @return true 操作成功
 * @return false 操作失败
 */
bool Sleep_enable(bool enable);

/**
 * @brief 开启/停止充电或放电
 * @param E_Charging 控制命令：
 *                   0x01 - 开启充电
 *                   0x00 - 停止充电
 * @return true  操作成功
 * @return false 操作失败
 */
bool OnOff_Charging(uint8_t E_Charging);

/*This command is sent to reset the device*/
void Bq76907_reset(void);


void Bq76907_Init(void);


#endif /* SERVES_INCLUDE_BQ76907_H_ */
