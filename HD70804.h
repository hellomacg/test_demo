/*
 * HD70804.h
 *
 *  Created on: 2025��8��22��
 *      Author: maode1
 */

#ifndef SERVES_INCLUDE_HD70804_H_
#define SERVES_INCLUDE_HD70804_H_
#include "Platform_Types.h"

#define HD70804Q_NUM_CHANNELS   4

/* �������� */
#define CHANNEL_INTERLOCK_GROUPS 2

typedef struct {
    uint8_t channel_mask;
    uint8_t enabled;
} HD70804Q_InterlockGroup_t;

/* �������ýṹ�� */
typedef struct {
    uint8_t port;    // �˿ں�
    uint8_t pin;     // ���ź�
} HD70804Q_PinConfig_t;

/* ����ģʽ */
typedef enum {
    HD70804Q_MODE_MANUAL = 0,    // �ֶ�ģʽ�����Ϻ����ֶ���λ
    HD70804Q_MODE_AUTO_RESTART   // �Զ�����ģʽ
} HD70804Q_OperationMode_t;

/* �������� */
typedef struct {
    uint8_t auto_restart_enabled;    // �Ƿ������Զ�����
    uint16_t restart_delay_ms;       // �����ӳ�ʱ��(ms)
    uint8_t max_restart_attempts;    // ����������Դ���
} HD70804Q_RestartConfig_t;

/* ���������� */
#define K_RATIO                 1000u
#define RSENSE_OHMS             1u
//#define V_CLAMP_TYP             5.1f
//#define CS_ADC_MAX_VOLTAGE      3.3f
//#define ADC_MAX_RESOLUTION      4095.0f

/* ������ֵ */
#define OVERLOAD_CURRENT_MA     5500u
#define OPEN_LOAD_CURRENT_MA    5000u
#define SHORT_CIRCUIT_VSENSE_V  4900u

/* Ĭ���������� */
#define DEFAULT_RESTART_DELAY_MS    1000    // Ĭ��1�������
#define DEFAULT_MAX_RESTART_ATTEMPTS 3      // Ĭ����ೢ��3��

/* ���ϱ�־���� */
typedef enum {
    HD70804Q_FAULT_NONE = 0x00,
    HD70804Q_FAULT_OVERLOAD = 0x01,
    HD70804Q_FAULT_SHORT_CIRCUIT = 0x02,
    HD70804Q_FAULT_OPEN_LOAD = 0x04,
    HD70804Q_FAULT_INTERLOCK = 0x08,
    HD70804Q_FAULT_OVERTEMP = 0x10,
} HD70804Q_Fault_t;

typedef struct {
	uint16_t load_current_ma;
    uint8_t is_enabled;
    HD70804Q_Fault_t fault;
    uint32_t fault_timestamp;
    uint32_t disable_timestamp;     // ͨ������ʱ���
    uint8_t restart_attempts;       // �������Դ���
    uint8_t needs_restart;          // ��Ҫ������־
} HD70804Q_ChannelState_t;

typedef struct {
    // ������������
    HD70804Q_PinConfig_t in_pins[HD70804Q_NUM_CHANNELS];

    // ��ϺͿ�����������
    HD70804Q_PinConfig_t sen_pin;       // Sense Enable
    HD70804Q_PinConfig_t sel0_pin;      // Mux Select 0
    HD70804Q_PinConfig_t sel1_pin;      // Mux Select 1
    HD70804Q_PinConfig_t fault_rst_pin; // Fault Reset

    // ��������
    HD70804Q_InterlockGroup_t interlock_groups[CHANNEL_INTERLOCK_GROUPS];

    // ��������
    HD70804Q_RestartConfig_t restart_config;

    // ADC���
    void* hadc_cs;
    uint32_t adc_channel;

    // ״̬
    HD70804Q_ChannelState_t channels[HD70804Q_NUM_CHANNELS];
    HD70804Q_OperationMode_t op_mode;   // ����ģʽ
    uint8_t diagnostics_ready;
    uint32_t last_fault_clear_time;     // ���������ʱ��
} HD70804Q_HandleTypeDef;

// �˿����Ͷ���
typedef enum {
    PORT_A = 0,
    PORT_B,
    PORT_C,
    PORT_D,
    PORT_E,
    PORT_MAX
} GpioPort_t;
// ������������
const HD70804Q_PinConfig_t input_pins[HD70804Q_NUM_CHANNELS] = {
    {PORT_C, 2},  // ͨ��0:
    {PORT_C, 1},  // ͨ��1:
    {PORT_E, 7},  // ͨ��2:
    {PORT_B, 17}  // ͨ��3:
};

const HD70804Q_PinConfig_t hd_sen_pin = {PORT_C, 3};
const HD70804Q_PinConfig_t hd_sel0_pin = {PORT_E, 3};
const HD70804Q_PinConfig_t hd_sel1_pin = {PORT_D, 15};
const HD70804Q_PinConfig_t hd_fault_rst_pin = {PORT_D, 11};


/**
 * @brief HD70804Q��������ʼ������
 *
 * ���������HD70804Q��������������ʼ�����̣�������
 * 1. Ӳ���������ó�ʼ��
 * 2. ͨ��������������
 * 3. ����ģʽ�������������ã���ѡ��
 *
 * @note �˺���Ӧ��ϵͳ����ʱ����һ��
 * @note ��������ȷ��ָ����ͨ���鲻��ͬʱ���ã���ֹ��Դ��·
 */
void HD70804Q_Init(void);

/**
 * @brief ���������İ�ȫ����ͨ��
 * @param hdev HD70804Q�豸���
 * @param ch ͨ���� (0 - HD70804Q_NUM_CHANNELS-1)
 * @param state ͨ��״̬ (0:�ر�, 1:����)
 * @return 0:�ɹ�, 1:ʧ�ܣ��������󡢻�����ͻ����ϣ�
 */
uint8_t HD70804Q_Set_Channel_With_Interlock(HD70804Q_HandleTypeDef* hdev, uint8_t ch, uint8_t state);

/**

@brief �ֶ�������ϣ����ֶ�ģʽ��Ч��

@param hdev HD70804Q�豸���
*/
void HD70804Q_Manual_Clear_Fault(HD70804Q_HandleTypeDef* hdev);


/*ʹ�� ���ָ��ͨ����״̬ ��Ҫ���HSDת��ѹ����*/
uint16_t Get_ADC_HSD_IS_MV();
/**
 * @brief ���ָ��ͨ����״̬
 * @param hdev HD70804Q�豸���
 * @param ch Ҫ��ص�ͨ����
 * @return ����õ���ͨ������ֵ����λmA��
 */
uint16_t HD70804Q_Monitor_Channel(HD70804Q_HandleTypeDef* hdev, uint8_t ch);


/**
 * @brief �ֶ��������
 */
void HD70804Q_Manual_Clear_Fault(HD70804Q_HandleTypeDef* hdev);



#endif /* SERVES_INCLUDE_HD70804_H_ */
