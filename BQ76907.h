/*
 * BQ76907.h
 *
 *  Created on: 2025��8��7��
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
 * @brief ��ȡָ����о�ĵ�ѹֵ
 * @param cell ��о��ţ�ͨ��Ϊ0~N-1��ȡ���ڵ���鴮������
 * @return uint16_t ��ѹֵ����λΪmV������ֱ�����ο������ֲᣩ
 *                 ����0xFFFF��ʾ��ȡʧ��
 */
int16_t ReadCellVoltage(uint8_t cell);

/**
 * @brief ��ȡ�ڲ��¶ȴ�����ֵ����λ��0.1��C��
 * @return int16_t �¶�ֵ����Χ��-400~1250����Ӧ-40.0��C~125.0��C��
 *                 ����0x8000��ʾ��ȡʧ��
 */
int16_t Read_Int_Temperature(void);

/**
 * @brief ��ȡCC1����ֵ����λ��mA��
 * @return int16_t ����ֵ����ֵ��ʾ��磬��ֵ��ʾ�ŵ磩
 *                ����0x8000��ʾ��ȡʧ��
 * @note ���裺
 *       1. �Ĵ���0x3C����16λ�з��Ų���
 *       2. �����ֱ��ʣ�1mA/LSB
 *       3. ���̣���32767mA
 */
int16_t Read_CC1_Current(void);

/**
 * @brief ��ȡCC2����ֵ����λ��mA��
 * @return int16_t ����ֵ����ֵ��ʾ��磬��ֵ��ʾ�ŵ磩
 *                ����0x8000��ʾ��ȡʧ��
 * @note ���裺
 *       1. �Ĵ���0x3C����16λ�з��Ų���
 *       2. �����ֱ��ʣ�1mA/LSB
 *       3. ���̣���32767mA
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
 * @brief ����ָ����о�ľ��⹦��
 * @param Cell ��оѡ��λ���룺
 *             0x00 - ��ֹ����
 *             0x02 - ��о1
 *             0x04 - ��о2
 *             0x06 - ��о3
 *             ...��֧�ֶ��о��ϣ���0x07��ʾ��о1+2+3��
 * @return true  �����ɹ�
 * @return false ����ʧ��
 */
bool Cell_Balancing(uint8_t Cell);

/**
 * @brief ����ѿ������⹦�ܵĵ�о
 * @return ��о���
 */
uint8_t Cell_Balancing_cbk(void);

/**
 * @brief ������ֹ����
 * @param enable ���Ʊ�־: true=��������, false=��ֹ����
 * @return true �����ɹ�
 * @return false ����ʧ��
 */
bool Sleep_enable(bool enable);

/**
 * @brief ����/ֹͣ����ŵ�
 * @param E_Charging �������
 *                   0x01 - �������
 *                   0x00 - ֹͣ���
 * @return true  �����ɹ�
 * @return false ����ʧ��
 */
bool OnOff_Charging(uint8_t E_Charging);

/*This command is sent to reset the device*/
void Bq76907_reset(void);


void Bq76907_Init(void);


#endif /* SERVES_INCLUDE_BQ76907_H_ */
