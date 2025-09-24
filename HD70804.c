/*
 * HD70804.c
 *
 *  Created on: 2025年8月22日
 *      Author: maode1
 */
#include "hd70804.h"
#include "Siul2_Port_Ip.h"
#include "Siul2_Dio_Ip.h"
// 引脚电平定义
#define PIN_LEVEL_LOW   0
#define PIN_LEVEL_HIGH  1

HD70804Q_HandleTypeDef hd70804q; // 在合适的位置定义全局变量

uint16_t Get_ADC_HSD_IS_MV()
{

}
/**
 * @brief 获取GPIO端口基地址
 * @param port 端口号 (0-4 对应 A-E)
 * @param pin 引脚号 (用于确定使用高半段还是低半段)
 * @return 端口基地址，无效参数返回NULL
 */
static Siul2_Dio_Ip_GpioType* HD70804Q_Get_Port_Base(uint8_t port, uint8_t pin)
{
    if (port >= PORT_MAX) {
        return NULL;
    }

    // 根据引脚号决定使用高半段还是低半段
    // 通常引脚0-15使用低半段，16-31使用高半段
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
 * @brief 设置GPIO引脚状态
 * @param port 端口号 (0-4 对应 A-E)
 * @param pin 引脚号 (0-31)
 * @param state 引脚状态 (0: LOW, 1: HIGH)
 * @return 操作成功返回true，失败返回false
 */
static bool HD70804Q_Write_Pin(uint8_t port, uint8_t pin, uint8_t state)
{
    // 参数验证
    if (port >= PORT_MAX || pin > 31) {
        return false;
    }

    // 获取端口基地址
    Siul2_Dio_Ip_GpioType* port_base = HD70804Q_Get_Port_Base(port, pin);
    if (port_base == NULL) {
        return false;
    }

    // 计算实际的引脚位（0-15）
    //uint8_t actual_pin = pin % 16;
    //Siul2_Dio_Ip_PinsChannelType pin_mask = (1U << actual_pin);

    // 写入引脚状态
    Siul2_Dio_Ip_WritePin(port_base, pin, state);

    return true;
}



/* 假设的时间函数 - 需要根据实际平台替换 */
static uint32_t HD70804Q_GetTick(void) {
    // 这里需要替换为实际的时间获取函数
    // 例如: return HAL_GetTick(); 或 return osKernelGetTickCount();
    static uint32_t tick = 0;
    return tick++;
}

/**
 * @brief 使能或禁用电流检测功能
 */
void HD70804Q_Enable_Current_Sense(HD70804Q_HandleTypeDef* hdev, uint8_t enable) {
    HD70804Q_Write_Pin(hdev->sen_pin.port, hdev->sen_pin.pin, enable);
}

/**
 * @brief 选择多路复用器通道
 */
void HD70804Q_Select_Mux_Channel(HD70804Q_HandleTypeDef* hdev, uint8_t channel) {
    if (channel >= HD70804Q_NUM_CHANNELS) return;

    HD70804Q_Write_Pin(hdev->sel0_pin.port, hdev->sel0_pin.pin, channel & 0x01);
    HD70804Q_Write_Pin(hdev->sel1_pin.port, hdev->sel1_pin.pin, (channel >> 1) & 0x01);
}

/**
 * @brief 清除指定通道的故障标志
 */
void HD70804Q_Clear_Fault(HD70804Q_HandleTypeDef* hdev, uint8_t ch) {
    if (ch < HD70804Q_NUM_CHANNELS) {
        hdev->channels[ch].fault = HD70804Q_FAULT_NONE;
    }
}
/**
 * @brief 初始化HD70804Q驱动器
 */
void HD70804Q_Cfg_Init(HD70804Q_HandleTypeDef* hdev,
                  const HD70804Q_PinConfig_t in_pins[HD70804Q_NUM_CHANNELS],
                  const HD70804Q_PinConfig_t* sen_pin,
                  const HD70804Q_PinConfig_t* sel0_pin,
                  const HD70804Q_PinConfig_t* sel1_pin,
                  const HD70804Q_PinConfig_t* fault_rst_pin,
                  void* hadc, uint32_t adc_ch)
{

    // 初始化引脚配置
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

    // 初始化互锁配置
    for (int i = 0; i < CHANNEL_INTERLOCK_GROUPS; i++) {
        hdev->interlock_groups[i].channel_mask = 0;
        hdev->interlock_groups[i].enabled = 0;
    }

    // 初始化重启配置
    hdev->restart_config.auto_restart_enabled = 1;
    hdev->restart_config.restart_delay_ms = DEFAULT_RESTART_DELAY_MS;
    hdev->restart_config.max_restart_attempts = DEFAULT_MAX_RESTART_ATTEMPTS;

    // 设置默认操作模式
    hdev->op_mode = HD70804Q_MODE_MANUAL;
    HD70804Q_Write_Pin(hdev->fault_rst_pin.port, hdev->fault_rst_pin.pin, 1);

    // 初始化默认状态
    HD70804Q_Enable_Current_Sense(hdev, 0);
    HD70804Q_Select_Mux_Channel(hdev, 0);

    hdev->diagnostics_ready = 1;
    hdev->last_fault_clear_time = 0;
}

/**
 * @brief 设置操作模式
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
 * @brief 设置重启配置
 */
void HD70804Q_Set_Restart_Config(HD70804Q_HandleTypeDef* hdev, uint8_t enabled,
                                uint16_t delay_ms, uint8_t max_attempts) {
    hdev->restart_config.auto_restart_enabled = enabled;
    hdev->restart_config.restart_delay_ms = delay_ms;
    hdev->restart_config.max_restart_attempts = max_attempts;
}

/**
 * @brief 配置通道互锁组
 */
void HD70804Q_Configure_Interlock(HD70804Q_HandleTypeDef* hdev, uint8_t group_id,
                                 uint8_t channel_mask, uint8_t enabled) {
    if (group_id < CHANNEL_INTERLOCK_GROUPS) {
        hdev->interlock_groups[group_id].channel_mask = channel_mask;
        hdev->interlock_groups[group_id].enabled = enabled;
    }
}

/**
 * @brief 检查通道互锁
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
 * @brief 在启用通道前检查故障
 */
HD70804Q_Fault_t HD70804Q_Check_Channel_Before_Enable(HD70804Q_HandleTypeDef* hdev, uint8_t ch)
{
    if (ch >= HD70804Q_NUM_CHANNELS)
    	return HD70804Q_FAULT_SHORT_CIRCUIT;

    HD70804Q_Enable_Current_Sense(hdev, 1);
    HD70804Q_Select_Mux_Channel(hdev, ch);

    // 短暂延时等待稳定
    for(volatile int i = 0; i < 1000; i++);

    // 模拟ADC读取 - 需要根据实际ADC驱动实现
    uint16_t vsense_voltage = 0u;
    vsense_voltage = Get_ADC_HSD_IS_MV();

    if (vsense_voltage > 4500u) {
        return HD70804Q_FAULT_SHORT_CIRCUIT;
    }

    return HD70804Q_FAULT_NONE;
}

/**
 * @brief 带互锁检查的安全设置通道
 */
uint8_t HD70804Q_Set_Channel_With_Interlock(HD70804Q_HandleTypeDef* hdev, uint8_t ch, uint8_t state) {
    if (ch >= HD70804Q_NUM_CHANNELS) return 1;

    if (state)
    {
        // 检查重启次数限制
        if (hdev->op_mode == HD70804Q_MODE_AUTO_RESTART &&
            hdev->channels[ch].restart_attempts >= hdev->restart_config.max_restart_attempts)
        {
            return 1;
        }

        // 检查互锁
        if (HD70804Q_Check_Interlock(hdev, ch))
        {
            hdev->channels[ch].fault = HD70804Q_FAULT_INTERLOCK;
            return 1;
        }

        // 检查电气故障
        HD70804Q_Fault_t fault = HD70804Q_Check_Channel_Before_Enable(hdev, ch);
        if (fault != HD70804Q_FAULT_NONE)
        {
            hdev->channels[ch].fault = fault;
            return 1;
        }

        // 重置重启计数
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
 * @brief 设置所有通道状态
 */
void HD70804Q_Set_All_Channels(HD70804Q_HandleTypeDef* hdev, uint8_t state) {
    for (int i = 0; i < HD70804Q_NUM_CHANNELS; i++) {
        HD70804Q_Set_Channel_With_Interlock(hdev, i, state);
    }
}

/**
 * @brief 手动清除故障
 */
void HD70804Q_Manual_Clear_Fault(HD70804Q_HandleTypeDef* hdev) {
    if (hdev->op_mode == HD70804Q_MODE_MANUAL) {
        // 产生低电平脉冲
        HD70804Q_Write_Pin(hdev->fault_rst_pin.port, hdev->fault_rst_pin.pin, 0);

        for(volatile int i = 0; i < 1000; i++);

        HD70804Q_Write_Pin(hdev->fault_rst_pin.port, hdev->fault_rst_pin.pin, 1);

        hdev->last_fault_clear_time = HD70804Q_GetTick();

        // 清除软件故障标志
        for (int i = 0; i < HD70804Q_NUM_CHANNELS; i++) {
            hdev->channels[i].fault &= HD70804Q_FAULT_OPEN_LOAD;
            hdev->channels[i].restart_attempts = 0;
            hdev->channels[i].needs_restart = 0;
        }
    }
}

/**
 * @brief 处理自动重启逻辑
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
 * @brief 监控指定通道的状态
 * @param hdev HD70804Q设备句柄
 * @param ch 要监控的通道号
 * @return 计算得到的通道电流值（单位mA）
 */
uint16_t HD70804Q_Monitor_Channel(HD70804Q_HandleTypeDef* hdev, uint8_t ch)
{
    // 检查通道号是否有效且通道已使能
    if (ch >= HD70804Q_NUM_CHANNELS|| !hdev->channels[ch].is_enabled)
    	return 0;

    // 启用电流检测功能，并选择指定通道进行监控
    HD70804Q_Enable_Current_Sense(hdev, 1);
    HD70804Q_Select_Mux_Channel(hdev, ch);

    // 短暂延时，等待信号稳定
    for(volatile int i = 0; i < 1000; i++);

    // 模拟ADC读取（此处应为实际ADC读取代码）
    uint16_t vsense_voltage = 0u;
    vsense_voltage = Get_ADC_HSD_IS_MV();
    // 根据检测电压计算实际电流值
    uint16_t calculated_current_ma = (vsense_voltage * K_RATIO) / RSENSE_OHMS * 1000;

    // 更新通道的负载电流值
    hdev->channels[ch].load_current_ma = calculated_current_ma;

    // 故障检测逻辑
    if (vsense_voltage >= SHORT_CIRCUIT_VSENSE_V) {
        // 检测到短路故障：关闭输出，记录故障状态和时间戳
        hdev->channels[ch].fault = HD70804Q_FAULT_SHORT_CIRCUIT;
        HD70804Q_Write_Pin(hdev->in_pins[ch].port, hdev->in_pins[ch].pin, 0);
        hdev->channels[ch].is_enabled = 0;
        hdev->channels[ch].disable_timestamp = HD70804Q_GetTick();

        // 自动重启模式下，标记需要重启
        if (hdev->op_mode == HD70804Q_MODE_AUTO_RESTART) {
            hdev->channels[ch].needs_restart = 1;
        }
    }
    else if (calculated_current_ma > OVERLOAD_CURRENT_MA) {
        // 检测到过载故障
        hdev->channels[ch].fault = HD70804Q_FAULT_OVERLOAD;
    }
    else if (calculated_current_ma < OPEN_LOAD_CURRENT_MA) {
        // 检测到开路负载故障
        hdev->channels[ch].fault = HD70804Q_FAULT_OPEN_LOAD;
    }
    else {
        // 无故障，清除故障标志
        hdev->channels[ch].fault = HD70804Q_FAULT_NONE;
    }

    // 返回计算得到的电流值
    return calculated_current_ma;
}

/**
 * @brief 监控所有通道
 */
void HD70804Q_Monitor_All_Channels(HD70804Q_HandleTypeDef* hdev) {
    for (int i = 0; i < HD70804Q_NUM_CHANNELS; i++) {
        if (hdev->channels[i].is_enabled) {
            HD70804Q_Monitor_Channel(hdev, i);
        }
    }
}

/**
 * @brief 关闭所有通道
 */
void HD70804Q_Disable_All_Channels(HD70804Q_HandleTypeDef* hdev) {
    for (int i = 0; i < HD70804Q_NUM_CHANNELS; i++) {
        HD70804Q_Write_Pin(hdev->in_pins[i].port, hdev->in_pins[i].pin, 0);
        hdev->channels[i].is_enabled = 0;
        HD70804Q_Clear_Fault(hdev, i);
    }
}

/**
 * @brief 获取通道电流
 */
float HD70804Q_Get_Current_MA(HD70804Q_HandleTypeDef* hdev, uint8_t ch) {
    return (ch < HD70804Q_NUM_CHANNELS) ? hdev->channels[ch].load_current_ma : 0.0f;
}

/**
 * @brief 获取通道故障状态
 */
HD70804Q_Fault_t HD70804Q_Get_Fault_State(HD70804Q_HandleTypeDef* hdev, uint8_t ch) {
    return (ch < HD70804Q_NUM_CHANNELS) ? hdev->channels[ch].fault : HD70804Q_FAULT_SHORT_CIRCUIT;
}

/**
 * @brief 检查通道是否使能
 */
uint8_t HD70804Q_Is_Channel_Enabled(HD70804Q_HandleTypeDef* hdev, uint8_t ch) {
    return (ch < HD70804Q_NUM_CHANNELS) ? hdev->channels[ch].is_enabled : 0;
}

/**
 * @brief 检查通道是否有故障
 */
uint8_t HD70804Q_Has_Fault(HD70804Q_HandleTypeDef* hdev, uint8_t ch) {
    return (ch < HD70804Q_NUM_CHANNELS) ? (hdev->channels[ch].fault != HD70804Q_FAULT_NONE) : 1;
}

/**
 * @brief 获取重启尝试次数
 */
uint8_t HD70804Q_Get_Restart_Attempts(HD70804Q_HandleTypeDef* hdev, uint8_t ch) {
    return (ch < HD70804Q_NUM_CHANNELS) ? hdev->channels[ch].restart_attempts : 0;
}


void HD70804Q_Init(void) {
    // 初始化HD70804Q
	HD70804Q_Cfg_Init(&hd70804q, input_pins, &hd_sen_pin, &hd_sel0_pin, &hd_sel1_pin, &hd_fault_rst_pin, NULL, 0);

    // 配置互锁：通道0、1、2、3不能同时启用
   HD70804Q_Configure_Interlock(&hd70804q, 0, 0x0f, 1);

    // 设置自动重启模式
   // HD70804Q_Set_Operation_Mode(&hd70804q, HD70804Q_MODE_MANUAL);
    //HD70804Q_Set_Restart_Config(&hd70804q, 1, 2000, 3);
}

/**
 * @brief 安全设置通道状态
 */
uint8_t HD70804Q_Set_Channel_Safe(HD70804Q_HandleTypeDef* hdev, uint8_t ch, uint8_t state)
{
    return HD70804Q_Set_Channel_With_Interlock(hdev, ch, state);
}



