#ifndef UJA1169_REGS_H
#define UJA1169_REGS_H

/* Register Addresses (0x00 - 0x7F) */
#define REG_WDOG_CTRL              0x00U
#define REG_MODE_CTRL              0x01U
#define REG_FAILSAFE_CTRL          0x02U
#define REG_MAIN_STATUS            0x03U
#define REG_SYS_EVENT_EN           0x04U
#define REG_WDOG_STATUS            0x05U
#define REG_MEM0                   0x06U
#define REG_MEM1                   0x07U
#define REG_MEM2                   0x08U
#define REG_MEM3                   0x09U
#define REG_LOCK_CTRL              0x0AU

#define REG_REGU_CTRL              0x10U
#define REG_SUPPLY_STATUS          0x1BU
#define REG_SUPPLY_EVENT_EN        0x1CU

#define REG_CAN_CTRL               0x20U
#define REG_TRANSCEIVER_STAT       0x22U
#define REG_TRANS_EVENT_EN         0x23U
#define REG_CAN_DATA_RATE          0x26U
#define REG_CAN_ID0                0x27U
#define REG_CAN_ID1                0x28U

#define REG_WAKE_STATUS            0x4BU
#define REG_WAKE_EN                0x4CU

#define REG_GLOBAL_EVT_STAT        0x60U
#define REG_SYS_EVT_STAT           0x61U
#define REG_SUPPLY_EVT_STAT        0x62U
#define REG_TRANS_EVT_STAT         0x63U
#define REG_WAKE_EVT_STAT          0x64U

#define REG_MTPNV_STATUS           0x70U
#define REG_STARTUP_CTRL           0x73U
#define REG_SBC_CONF_CTRL          0x74U
#define REG_CRC_CTRL               0x75U
#define REG_DEVICE_ID              0x7EU

/* Bitfield Masks (fill gradually, placeholders where unsure) */
/* WDOG_CTRL (0x00) */
#define WDOG_CTRL_EN_MASK          (0x01U)
/* MODE_CTRL (0x01) */
#define MODE_CTRL_NORMAL_MASK      (0x01U)
#define MODE_CTRL_STBY_MASK        (0x02U)
#define MODE_CTRL_SLEEP_MASK       (0x04U)
/* FAILSAFE_CTRL (0x02) */
#define FAILSAFE_CTRL_RCC          (0x01U)  /* Reset cause clear (示例) */

/* MAIN_STATUS (0x03) */
#define MAIN_STATUS_NMS            (0x01U)  /* Normal mode status bit (示例) */

/* REGU_CTRL (0x10) */
#define REGU_CTRL_V1RTC            (0x01U)
#define REGU_CTRL_VEXTC            (0x02U)

/* SUPPLY_STATUS (0x1B) */
#define SUPPLY_STATUS_V1S          (0x01U)

/* CAN_CTRL (0x20) */
#define CAN_CTRL_CMC               (0x01U)
#define CAN_CTRL_CPNC              (0x02U)

/* WDOG_STATUS (0x05) */
#define WDOG_STATUS_WDS            (0x01U)

/* SYS_EVENT_EN / SUPPLY_EVENT_EN / TRANS_EVENT_EN bits (示例) */
#define SYS_EVENT_EN_SPIFE         (0x01U)
#define SYS_EVENT_EN_OTWE          (0x02U)
#define SUPPLY_EVENT_EN_V1UE       (0x01U)
#define SUPPLY_EVENT_EN_V2UE       (0x02U)
#define TRANS_EVENT_EN_CWE         (0x01U)
#define TRANS_EVENT_EN_CFE         (0x02U)

/* Add more bit definitions as they are confirmed */


/* ===== WDOG_CTRL (0x00) ===== */
/* WMC: Timeout 模式，bits[7:5] = 010 */
#ifndef WDOG_CTRL_WMC_TIMEOUT
#define WDOG_CTRL_WMC_TIMEOUT      (0x40u)
#endif
/* NWP: 128 ms，bits[3:0] = 0100 */
#ifndef WDOG_CTRL_NWP_128MS
#define WDOG_CTRL_NWP_128MS        (0x04u)
#endif
/* 假设:
 *  bits[7:5] WMC (Window/Timeout/Autonomous)
 *  bits[3:0] NWP (周期/窗口参数)
 *  bit0 可能与 EN 不同芯片实现不同，以下仅示例，请按手册核实
 */
#ifndef WDOG_CTRL_WMC_MASK
#define WDOG_CTRL_WMC_MASK         (0xE0u)
#endif
#ifndef WDOG_CTRL_NWP_MASK
#define WDOG_CTRL_NWP_MASK         (0x0Fu)
#endif
#ifdef WDOG_CTRL_EN_MASK
#undef WDOG_CTRL_EN_MASK   /* 手册无独立 EN 位：移除避免误写 NWP */
#endif
/* Watchdog enable 由 WMC 模式非 000 决定，无独立 EN 位（待手册确认） */

/* ===== REGU_CTRL (0x10) ===== */
/* V1RTC: 80% 复位阈值，bits[1:0] = 01 */
#ifndef V1RTC_80PCT
#define V1RTC_80PCT                (0x01u)
#endif
/* V2C/VEXTC: 仅 Normal 打开，bits[3:2] = 01 */
#ifndef VEXTC_ON_IN_NORMAL
#define VEXTC_ON_IN_NORMAL         (0x04u)
#endif
/* 假设:
 *  bits[1:0] V1RTC 阈值
 *  bits[3:2] VEXTC 工作策略
 */
#ifndef REGU_CTRL_V1RTC_MASK
#define REGU_CTRL_V1RTC_MASK       (0x03u)
#endif
#ifndef REGU_CTRL_VEXTC_MASK
#define REGU_CTRL_VEXTC_MASK       (0x0Cu)
#endif

/* ===== FAILSAFE_CTRL (0x02) ===== */
/* 若 FAILSAFE_CTRL_RCC 为 W1C 单 bit 则无需 MASK；如是 2-bit 字段另行定义:
 */
#ifndef FAILSAFE_CTRL_RCC
#define FAILSAFE_CTRL_RCC          (0x01u) /* 占位 */
#endif
/* 可选其它位: LHC (假设位2) */
#ifndef FAILSAFE_CTRL_LHC
/* #define FAILSAFE_CTRL_LHC       (0x04u) */
#endif

/* ===== CAN_CTRL (0x20) ===== */
/* CMC: Active（编码 01，bits[1:0]）—建议用“读改写+掩码”设置 */
#ifndef CAN_CTRL_CMC
#define CAN_CTRL_CMC               (0x01u)
#endif



#endif /* UJA1169_REGS_H */
