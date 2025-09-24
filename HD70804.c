/*
 * HD70804.c
 *
 *  Created on: 2025��8��22��
 *      Author: maode1
 */
#include "hd70804.h"
#include "Siul2_Port_Ip.h"
#include "Siul2_Dio_Ip.h"
// ���ŵ�ƽ����
#define PIN_LEVEL_LOW   0
#define PIN_LEVEL_HIGH  1

HD70804Q_HandleTypeDef hd70804q; // �ں��ʵ�λ�ö���ȫ�ֱ���

uint16_t Get_ADC_HSD_IS_MV()
{

}
/**
 * @brief ��ȡGPIO�˿ڻ���ַ
 * @param port �˿ں� (0-4 ��Ӧ A-E)
 * @param pin ���ź� (����ȷ��ʹ�ø߰�λ��ǵͰ��)
 * @return �˿ڻ���ַ����Ч��������NULL
 */
static Siul2_Dio_Ip_GpioType* HD70804Q_Get_Port_Base(uint8_t port, uint8_t pin)
{
    if (port >= PORT_MAX) {
        return NULL;
    }

    // �������źž���ʹ�ø߰�λ��ǵͰ��
    // ͨ������0-15ʹ�õͰ�Σ�16-31ʹ�ø߰��
    uint8_t use_high_half = (pin >= 16);

    switch (port) {
        case PORT_A:
            return use_high_half ? PTA_H_HALF : PTA_L_HALF;
        case PORT_B:
            return use_high_half ? PTB_H_HALF : PTB_L_HALF;
        case PORT_C:
            return use_high_half ? PTC_H_HALF : PTC_L_HALF;
        case PORT_D:
            return use_high_half ? PTD_H_HALF : PTD_L_HALF;
        case PORT_E:
            return use_high_half ? PTE_H_HALF : PTE_L_HALF;
        default:
            return NULL;
    }
}

/**
 * @brief ����GPIO����״̬
 * @param port �˿ں� (0-4 ��Ӧ A-E)
 * @param pin ���ź� (0-31)
 * @param state ����״̬ (0: LOW, 1: HIGH)
 * @return �����ɹ�����true��ʧ�ܷ���false
 */
static bool HD70804Q_Write_Pin(uint8_t port, uint8_t pin, uint8_t state)
{
    // ������֤
    if (port >= PORT_MAX || pin > 31) {
        return false;
    }

    // ��ȡ�˿ڻ���ַ
    Siul2_Dio_Ip_GpioType* port_base = HD70804Q_Get_Port_Base(port, pin);
    if (port_base == NULL) {
        return false;
    }

    // ����ʵ�ʵ�����λ��0-15��
    //uint8_t actual_pin = pin % 16;
    //Siul2_Dio_Ip_PinsChannelType pin_mask = (1U << actual_pin);

    // д������״̬
    Siul2_Dio_Ip_WritePin(port_base, pin, state);

    return true;
}



/* �����ʱ�亯�� - ��Ҫ����ʵ��ƽ̨�滻 */
static uint32_t HD70804Q_GetTick(void) {
    // ������Ҫ�滻Ϊʵ�ʵ�ʱ���ȡ����
    // ����: return HAL_GetTick(); �� return osKernelGetTickCount();
    static uint32_t tick = 0;
    return tick++;
}

/**
 * @brief ʹ�ܻ���õ�����⹦��
 */
void HD70804Q_Enable_Current_Sense(HD70804Q_HandleTypeDef* hdev, uint8_t enable) {
    HD70804Q_Write_Pin(hdev->sen_pin.port, hdev->sen_pin.pin, enable);
}

/**
 * @brief ѡ���·������ͨ��
 */
void HD70804Q_Select_Mux_Channel(HD70804Q_HandleTypeDef* hdev, uint8_t channel) {
    if (channel >= HD70804Q_NUM_CHANNELS) return;

    HD70804Q_Write_Pin(hdev->sel0_pin.port, hdev->sel0_pin.pin, channel & 0x01);
    HD70804Q_Write_Pin(hdev->sel1_pin.port, hdev->sel1_pin.pin, (channel >> 1) & 0x01);
}

/**
 * @brief ���ָ��ͨ���Ĺ��ϱ�־
 */
void HD70804Q_Clear_Fault(HD70804Q_HandleTypeDef* hdev, uint8_t ch) {
    if (ch < HD70804Q_NUM_CHANNELS) {
        hdev->channels[ch].fault = HD70804Q_FAULT_NONE;
    }
}
/**
 * @brief ��ʼ��HD70804Q������
 */
void HD70804Q_Cfg_Init(HD70804Q_HandleTypeDef* hdev,
                  const HD70804Q_PinConfig_t in_pins[HD70804Q_NUM_CHANNELS],
                  const HD70804Q_PinConfig_t* sen_pin,
                  const HD70804Q_PinConfig_t* sel0_pin,
                  const HD70804Q_PinConfig_t* sel1_pin,
                  const HD70804Q_PinConfig_t* fault_rst_pin,
                  void* hadc, uint32_t adc_ch)
{

    // ��ʼ����������
    for (int i = 0; i < HD70804Q_NUM_CHANNELS; i++)
    {
        hdev->in_pins[i] = in_pins[i];

        hdev->channels[i].load_current_ma = 0.0f;
        hdev->channels[i].is_enabled = 0;
        hdev->channels[i].fault = HD70804Q_FAULT_NONE;
        hdev->channels[i].fault_timestamp = 0;
        hdev->channels[i].disable_timestamp = 0;
        hdev->channels[i].restart_attempts = 0;
        hdev->channels[i].needs_restart = 0;

        HD70804Q_Write_Pin(in_pins[i].port, in_pins[i].pin, 0);
    }

    hdev->sen_pin = *sen_pin;
    hdev->sel0_pin = *sel0_pin;
    hdev->sel1_pin = *sel1_pin;
    hdev->fault_rst_pin = *fault_rst_pin;

    hdev->hadc_cs = hadc;
    hdev->adc_channel = adc_ch;

    // ��ʼ����������
    for (int i = 0; i < CHANNEL_INTERLOCK_GROUPS; i++) {
        hdev->interlock_groups[i].channel_mask = 0;
        hdev->interlock_groups[i].enabled = 0;
    }

    // ��ʼ����������
    hdev->restart_config.auto_restart_enabled = 1;
    hdev->restart_config.restart_delay_ms = DEFAULT_RESTART_DELAY_MS;
    hdev->restart_config.max_restart_attempts = DEFAULT_MAX_RESTART_ATTEMPTS;

    // ����Ĭ�ϲ���ģʽ
    hdev->op_mode = HD70804Q_MODE_MANUAL;
    HD70804Q_Write_Pin(hdev->fault_rst_pin.port, hdev->fault_rst_pin.pin, 1);

    // ��ʼ��Ĭ��״̬
    HD70804Q_Enable_Current_Sense(hdev, 0);
    HD70804Q_Select_Mux_Channel(hdev, 0);

    hdev->diagnostics_ready = 1;
    hdev->last_fault_clear_time = 0;
}

/**
 * @brief ���ò���ģʽ
 */
void HD70804Q_Set_Operation_Mode(HD70804Q_HandleTypeDef* hdev, HD70804Q_OperationMode_t mode) {
    hdev->op_mode = mode;

    if (mode == HD70804Q_MODE_AUTO_RESTART) {
        HD70804Q_Write_Pin(hdev->fault_rst_pin.port, hdev->fault_rst_pin.pin, 0);
    } else {
        HD70804Q_Write_Pin(hdev->fault_rst_pin.port, hdev->fault_rst_pin.pin, 1);
    }
}

/**
 * @brief ������������
 */
void HD70804Q_Set_Restart_Config(HD70804Q_HandleTypeDef* hdev, uint8_t enabled,
                                uint16_t delay_ms, uint8_t max_attempts) {
    hdev->restart_config.auto_restart_enabled = enabled;
    hdev->restart_config.restart_delay_ms = delay_ms;
    hdev->restart_config.max_restart_attempts = max_attempts;
}

/**
 * @brief ����ͨ��������
 */
void HD70804Q_Configure_Interlock(HD70804Q_HandleTypeDef* hdev, uint8_t group_id,
                                 uint8_t channel_mask, uint8_t enabled) {
    if (group_id < CHANNEL_INTERLOCK_GROUPS) {
        hdev->interlock_groups[group_id].channel_mask = channel_mask;
        hdev->interlock_groups[group_id].enabled = enabled;
    }
}

/**
 * @brief ���ͨ������
 */
uint8_t HD70804Q_Check_Interlock(HD70804Q_HandleTypeDef* hdev, uint8_t ch_to_enable)
{
    if (ch_to_enable >= HD70804Q_NUM_CHANNELS) return 1;

    for (int i = 0; i < CHANNEL_INTERLOCK_GROUPS; i++)
    {
        if (hdev->interlock_groups[i].enabled)
        {
            uint8_t group_mask = hdev->interlock_groups[i].channel_mask;

            if (group_mask & (1 << ch_to_enable))
            {
                for (int ch = 0; ch < HD70804Q_NUM_CHANNELS; ch++)
                {
                    if (ch != ch_to_enable && (group_mask & (1 << ch)) && hdev->channels[ch].is_enabled)
                    {
                        return 1;
                    }
                }
            }
        }
    }

    return 0;
}

/**
 * @brief ������ͨ��ǰ������
 */
HD70804Q_Fault_t HD70804Q_Check_Channel_Before_Enable(HD70804Q_HandleTypeDef* hdev, uint8_t ch)
{
    if (ch >= HD70804Q_NUM_CHANNELS)
    	return HD70804Q_FAULT_SHORT_CIRCUIT;

    HD70804Q_Enable_Current_Sense(hdev, 1);
    HD70804Q_Select_Mux_Channel(hdev, ch);

    // ������ʱ�ȴ��ȶ�
    for(volatile int i = 0; i < 1000; i++);

    // ģ��ADC��ȡ - ��Ҫ����ʵ��ADC����ʵ��
    uint16_t vsense_voltage = 0u;
    vsense_voltage = Get_ADC_HSD_IS_MV();

    if (vsense_voltage > 4500u) {
        return HD70804Q_FAULT_SHORT_CIRCUIT;
    }

    return HD70804Q_FAULT_NONE;
}

/**
 * @brief ���������İ�ȫ����ͨ��
 */
uint8_t HD70804Q_Set_Channel_With_Interlock(HD70804Q_HandleTypeDef* hdev, uint8_t ch, uint8_t state) {
    if (ch >= HD70804Q_NUM_CHANNELS) return 1;

    if (state)
    {
        // ���������������
        if (hdev->op_mode == HD70804Q_MODE_AUTO_RESTART &&
            hdev->channels[ch].restart_attempts >= hdev->restart_config.max_restart_attempts)
        {
            return 1;
        }

        // ��黥��
        if (HD70804Q_Check_Interlock(hdev, ch))
        {
            hdev->channels[ch].fault = HD70804Q_FAULT_INTERLOCK;
            return 1;
        }

        // ����������
        HD70804Q_Fault_t fault = HD70804Q_Check_Channel_Before_Enable(hdev, ch);
        if (fault != HD70804Q_FAULT_NONE)
        {
            hdev->channels[ch].fault = fault;
            return 1;
        }

        // ������������
        hdev->channels[ch].restart_attempts = 0;
        hdev->channels[ch].needs_restart = 0;
    }

    HD70804Q_Write_Pin(hdev->in_pins[ch].port, hdev->in_pins[ch].pin, state);
    Siul2_Dio_Ip_WritePin(PTD_L_HALF, 6u, state);

    hdev->channels[ch].is_enabled = state;

    if (!state) {
        HD70804Q_Clear_Fault(hdev, ch);
        hdev->channels[ch].disable_timestamp = HD70804Q_GetTick();
    }

    return 0;
}


/**
 * @brief ��������ͨ��״̬
 */
void HD70804Q_Set_All_Channels(HD70804Q_HandleTypeDef* hdev, uint8_t state) {
    for (int i = 0; i < HD70804Q_NUM_CHANNELS; i++) {
        HD70804Q_Set_Channel_With_Interlock(hdev, i, state);
    }
}

/**
 * @brief �ֶ��������
 */
void HD70804Q_Manual_Clear_Fault(HD70804Q_HandleTypeDef* hdev) {
    if (hdev->op_mode == HD70804Q_MODE_MANUAL) {
        // �����͵�ƽ����
        HD70804Q_Write_Pin(hdev->fault_rst_pin.port, hdev->fault_rst_pin.pin, 0);

        for(volatile int i = 0; i < 1000; i++);

        HD70804Q_Write_Pin(hdev->fault_rst_pin.port, hdev->fault_rst_pin.pin, 1);

        hdev->last_fault_clear_time = HD70804Q_GetTick();

        // ���������ϱ�־
        for (int i = 0; i < HD70804Q_NUM_CHANNELS; i++) {
            hdev->channels[i].fault &= HD70804Q_FAULT_OPEN_LOAD;
            hdev->channels[i].restart_attempts = 0;
            hdev->channels[i].needs_restart = 0;
        }
    }
}

/**
 * @brief �����Զ������߼�
 */
void HD70804Q_Process_Auto_Restart(HD70804Q_HandleTypeDef* hdev) {
    if (hdev->op_mode != HD70804Q_MODE_AUTO_RESTART ||
        !hdev->restart_config.auto_restart_enabled) {
        return;
    }

    uint32_t current_time = HD70804Q_GetTick();

    for (int i = 0; i < HD70804Q_NUM_CHANNELS; i++) {
        if (hdev->channels[i].needs_restart &&
            !hdev->channels[i].is_enabled &&
            (current_time - hdev->channels[i].disable_timestamp) >= hdev->restart_config.restart_delay_ms) {

            if (hdev->channels[i].restart_attempts < hdev->restart_config.max_restart_attempts) {
                hdev->channels[i].restart_attempts++;
                hdev->channels[i].needs_restart = 0;

                HD70804Q_Clear_Fault(hdev, i);
                HD70804Q_Write_Pin(hdev->in_pins[i].port, hdev->in_pins[i].pin, 1);
                hdev->channels[i].is_enabled = 1;
            } else {
                hdev->channels[i].needs_restart = 0;
            }
        }
    }
}



/**
 * @brief ���ָ��ͨ����״̬
 * @param hdev HD70804Q�豸���
 * @param ch Ҫ��ص�ͨ����
 * @return ����õ���ͨ������ֵ����λmA��
 */
uint16_t HD70804Q_Monitor_Channel(HD70804Q_HandleTypeDef* hdev, uint8_t ch)
{
    // ���ͨ�����Ƿ���Ч��ͨ����ʹ��
    if (ch >= HD70804Q_NUM_CHANNELS|| !hdev->channels[ch].is_enabled)
    	return 0;

    // ���õ�����⹦�ܣ���ѡ��ָ��ͨ�����м��
    HD70804Q_Enable_Current_Sense(hdev, 1);
    HD70804Q_Select_Mux_Channel(hdev, ch);

    // ������ʱ���ȴ��ź��ȶ�
    for(volatile int i = 0; i < 1000; i++);

    // ģ��ADC��ȡ���˴�ӦΪʵ��ADC��ȡ���룩
    uint16_t vsense_voltage = 0u;
    vsense_voltage = Get_ADC_HSD_IS_MV();
    // ���ݼ���ѹ����ʵ�ʵ���ֵ
    uint16_t calculated_current_ma = (vsense_voltage * K_RATIO) / RSENSE_OHMS * 1000;

    // ����ͨ���ĸ��ص���ֵ
    hdev->channels[ch].load_current_ma = calculated_current_ma;

    // ���ϼ���߼�
    if (vsense_voltage >= SHORT_CIRCUIT_VSENSE_V) {
        // ��⵽��·���ϣ��ر��������¼����״̬��ʱ���
        hdev->channels[ch].fault = HD70804Q_FAULT_SHORT_CIRCUIT;
        HD70804Q_Write_Pin(hdev->in_pins[ch].port, hdev->in_pins[ch].pin, 0);
        hdev->channels[ch].is_enabled = 0;
        hdev->channels[ch].disable_timestamp = HD70804Q_GetTick();

        // �Զ�����ģʽ�£������Ҫ����
        if (hdev->op_mode == HD70804Q_MODE_AUTO_RESTART) {
            hdev->channels[ch].needs_restart = 1;
        }
    }
    else if (calculated_current_ma > OVERLOAD_CURRENT_MA) {
        // ��⵽���ع���
        hdev->channels[ch].fault = HD70804Q_FAULT_OVERLOAD;
    }
    else if (calculated_current_ma < OPEN_LOAD_CURRENT_MA) {
        // ��⵽��·���ع���
        hdev->channels[ch].fault = HD70804Q_FAULT_OPEN_LOAD;
    }
    else {
        // �޹��ϣ�������ϱ�־
        hdev->channels[ch].fault = HD70804Q_FAULT_NONE;
    }

    // ���ؼ���õ��ĵ���ֵ
    return calculated_current_ma;
}

/**
 * @brief �������ͨ��
 */
void HD70804Q_Monitor_All_Channels(HD70804Q_HandleTypeDef* hdev) {
    for (int i = 0; i < HD70804Q_NUM_CHANNELS; i++) {
        if (hdev->channels[i].is_enabled) {
            HD70804Q_Monitor_Channel(hdev, i);
        }
    }
}

/**
 * @brief �ر�����ͨ��
 */
void HD70804Q_Disable_All_Channels(HD70804Q_HandleTypeDef* hdev) {
    for (int i = 0; i < HD70804Q_NUM_CHANNELS; i++) {
        HD70804Q_Write_Pin(hdev->in_pins[i].port, hdev->in_pins[i].pin, 0);
        hdev->channels[i].is_enabled = 0;
        HD70804Q_Clear_Fault(hdev, i);
    }
}

/**
 * @brief ��ȡͨ������
 */
float HD70804Q_Get_Current_MA(HD70804Q_HandleTypeDef* hdev, uint8_t ch) {
    return (ch < HD70804Q_NUM_CHANNELS) ? hdev->channels[ch].load_current_ma : 0.0f;
}

/**
 * @brief ��ȡͨ������״̬
 */
HD70804Q_Fault_t HD70804Q_Get_Fault_State(HD70804Q_HandleTypeDef* hdev, uint8_t ch) {
    return (ch < HD70804Q_NUM_CHANNELS) ? hdev->channels[ch].fault : HD70804Q_FAULT_SHORT_CIRCUIT;
}

/**
 * @brief ���ͨ���Ƿ�ʹ��
 */
uint8_t HD70804Q_Is_Channel_Enabled(HD70804Q_HandleTypeDef* hdev, uint8_t ch) {
    return (ch < HD70804Q_NUM_CHANNELS) ? hdev->channels[ch].is_enabled : 0;
}

/**
 * @brief ���ͨ���Ƿ��й���
 */
uint8_t HD70804Q_Has_Fault(HD70804Q_HandleTypeDef* hdev, uint8_t ch) {
    return (ch < HD70804Q_NUM_CHANNELS) ? (hdev->channels[ch].fault != HD70804Q_FAULT_NONE) : 1;
}

/**
 * @brief ��ȡ�������Դ���
 */
uint8_t HD70804Q_Get_Restart_Attempts(HD70804Q_HandleTypeDef* hdev, uint8_t ch) {
    return (ch < HD70804Q_NUM_CHANNELS) ? hdev->channels[ch].restart_attempts : 0;
}


void HD70804Q_Init(void) {
    // ��ʼ��HD70804Q
	HD70804Q_Cfg_Init(&hd70804q, input_pins, &hd_sen_pin, &hd_sel0_pin, &hd_sel1_pin, &hd_fault_rst_pin, NULL, 0);

    // ���û�����ͨ��0��1��2��3����ͬʱ����
   HD70804Q_Configure_Interlock(&hd70804q, 0, 0x0f, 1);

    // �����Զ�����ģʽ
   // HD70804Q_Set_Operation_Mode(&hd70804q, HD70804Q_MODE_MANUAL);
    //HD70804Q_Set_Restart_Config(&hd70804q, 1, 2000, 3);
}

/**
 * @brief ��ȫ����ͨ��״̬
 */
uint8_t HD70804Q_Set_Channel_Safe(HD70804Q_HandleTypeDef* hdev, uint8_t ch, uint8_t state)
{
    return HD70804Q_Set_Channel_With_Interlock(hdev, ch, state);
}



