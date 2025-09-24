/*
 * HD70804.h
 *
 *  Created on: 2025年8月22日
 *      Author: maode1
 */

#ifndef SERVES_INCLUDE_HD70804_H_
#define SERVES_INCLUDE_HD70804_H_
#include "Platform_Types.h"

#define HD70804Q_NUM_CHANNELS   4

/* 互锁配置 */
#define CHANNEL_INTERLOCK_GROUPS 2

typedef struct {
    uint8_t channel_mask;
    uint8_t enabled;
} HD70804Q_InterlockGroup_t;

/* 引脚配置结构体 */
typedef struct {
    uint8_t port;    // 端口号
    uint8_t pin;     // 引脚号
} HD70804Q_PinConfig_t;

/* 工作模式 */
typedef enum {
    HD70804Q_MODE_MANUAL = 0,    // 手动模式，故障后需手动复位
    HD70804Q_MODE_AUTO_RESTART   // 自动重启模式
} HD70804Q_OperationMode_t;

/* 重启配置 */
typedef struct {
    uint8_t auto_restart_enabled;    // 是否启用自动重启
    uint16_t restart_delay_ms;       // 重启延迟时间(ms)
    uint8_t max_restart_attempts;    // 最大重启尝试次数
} HD70804Q_RestartConfig_t;

/* 电流检测参数 */
#define K_RATIO                 1000u
#define RSENSE_OHMS             1u
//#define V_CLAMP_TYP             5.1f
//#define CS_ADC_MAX_VOLTAGE      3.3f
//#define ADC_MAX_RESOLUTION      4095.0f

/* 故障阈值 */
#define OVERLOAD_CURRENT_MA     5500u
#define OPEN_LOAD_CURRENT_MA    5000u
#define SHORT_CIRCUIT_VSENSE_V  4900u

/* 默认重启配置 */
#define DEFAULT_RESTART_DELAY_MS    1000    // 默认1秒后重启
#define DEFAULT_MAX_RESTART_ATTEMPTS 3      // 默认最多尝试3次

/* 故障标志定义 */
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
    uint32_t disable_timestamp;     // 通道禁用时间戳
    uint8_t restart_attempts;       // 重启尝试次数
    uint8_t needs_restart;          // 需要重启标志
} HD70804Q_ChannelState_t;

typedef struct {
    // 控制引脚配置
    HD70804Q_PinConfig_t in_pins[HD70804Q_NUM_CHANNELS];

    // 诊断和控制引脚配置
    HD70804Q_PinConfig_t sen_pin;       // Sense Enable
    HD70804Q_PinConfig_t sel0_pin;      // Mux Select 0
    HD70804Q_PinConfig_t sel1_pin;      // Mux Select 1
    HD70804Q_PinConfig_t fault_rst_pin; // Fault Reset

    // 互锁配置
    HD70804Q_InterlockGroup_t interlock_groups[CHANNEL_INTERLOCK_GROUPS];

    // 重启配置
    HD70804Q_RestartConfig_t restart_config;

    // ADC相关
    void* hadc_cs;
    uint32_t adc_channel;

    // 状态
    HD70804Q_ChannelState_t channels[HD70804Q_NUM_CHANNELS];
    HD70804Q_OperationMode_t op_mode;   // 操作模式
    uint8_t diagnostics_ready;
    uint32_t last_fault_clear_time;     // 最后故障清除时间
} HD70804Q_HandleTypeDef;

// 端口类型定义
typedef enum {
    PORT_A = 0,
    PORT_B,
    PORT_C,
    PORT_D,
    PORT_E,
    PORT_MAX
} GpioPort_t;
// 定义引脚配置
const HD70804Q_PinConfig_t input_pins[HD70804Q_NUM_CHANNELS] = {
    {PORT_C, 2},  // 通道0:
    {PORT_C, 1},  // 通道1:
    {PORT_E, 7},  // 通道2:
    {PORT_B, 17}  // 通道3:
};

const HD70804Q_PinConfig_t hd_sen_pin = {PORT_C, 3};
const HD70804Q_PinConfig_t hd_sel0_pin = {PORT_E, 3};
const HD70804Q_PinConfig_t hd_sel1_pin = {PORT_D, 15};
const HD70804Q_PinConfig_t hd_fault_rst_pin = {PORT_D, 11};


/**
 * @brief HD70804Q驱动器初始化函数
 *
 * 本函数完成HD70804Q驱动器的完整初始化流程，包括：
 * 1. 硬件引脚配置初始化
 * 2. 通道互锁功能配置
 * 3. 操作模式和重启参数设置（可选）
 *
 * @note 此函数应在系统启动时调用一次
 * @note 互锁功能确保指定的通道组不能同时启用，防止电源短路
 */
void HD70804Q_Init(void);

/**
 * @brief 带互锁检查的安全设置通道
 * @param hdev HD70804Q设备句柄
 * @param ch 通道号 (0 - HD70804Q_NUM_CHANNELS-1)
 * @param state 通道状态 (0:关闭, 1:开启)
 * @return 0:成功, 1:失败（参数错误、互锁冲突或故障）
 */
uint8_t HD70804Q_Set_Channel_With_Interlock(HD70804Q_HandleTypeDef* hdev, uint8_t ch, uint8_t state);

/**

@brief 手动清除故障（仅手动模式有效）

@param hdev HD70804Q设备句柄
*/
void HD70804Q_Manual_Clear_Fault(HD70804Q_HandleTypeDef* hdev);


/*使用 监控指定通道的状态 需要添加HSD转电压函数*/
uint16_t Get_ADC_HSD_IS_MV();
/**
 * @brief 监控指定通道的状态
 * @param hdev HD70804Q设备句柄
 * @param ch 要监控的通道号
 * @return 计算得到的通道电流值（单位mA）
 */
uint16_t HD70804Q_Monitor_Channel(HD70804Q_HandleTypeDef* hdev, uint8_t ch);


/**
 * @brief 手动清除故障
 */
void HD70804Q_Manual_Clear_Fault(HD70804Q_HandleTypeDef* hdev);



#endif /* SERVES_INCLUDE_HD70804_H_ */
