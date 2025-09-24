#ifndef UJA1169_H
#define UJA1169_H

#include <stdint.h>
#include <stdbool.h>
#include "UJA1169_Regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================
 * Status / Error Codes (缁熶竴鏋氫妇)
 * 璐熸暟淇濇寔涓庝綘鐜版湁瀹炵幇鍏煎
 * ================================ */
typedef enum
{
    UJA1169_STATUS_OK          = 0,   /* 鎴愬姛 */
    UJA1169_STATUS_SPI_FAIL    = -1,  /* SPI 浼犺緭澶辫触 */
    UJA1169_STATUS_NOT_INIT    = -2,  /* 鏈垵濮嬪寲 */
    UJA1169_STATUS_BAD_PARAM   = -3,  /* 鍦板潃鎴栧弬鏁伴潪娉� */
    UJA1169_STATUS_LEN_OVF     = -4,  /* 绐佸彂闀垮害瓒呴檺/缂撳啿涓嶈冻 */
    UJA1169_STATUS_ID_FAULT    = -5   /* 鏂板锛氳澶� ID 涓嶅湪鐧藉悕鍗曟垨闈炴硶(0x00/0xFF) */
} UJA1169_StatusType;

/* ================================
 * Operating mode 鏋氫妇锛堜笌瀵勫瓨鍣ㄤ綅鍖哄垎锛�
 * 涓婂眰浣跨敤璇ユ灇涓撅紝搴曞眰鏄犲皠涓� MODE_CTRL_xxx_MASK
 * ================================ */
typedef enum
{
    UJA1169_MODE_NORMAL  = 0,
    UJA1169_MODE_STANDBY = 1,
    UJA1169_MODE_SLEEP   = 2
} UJA1169_ModeType;

/* ================================
 * Public API
 * ================================ */

/* 椹卞姩鍒濆鍖栵細鎵ц涓�娆¤澶囧彲杈炬�ф娴�(璇诲彇璁惧 ID) */
int UJA1169_Init(void);

/* 鍗曞瘎瀛樺櫒璁块棶 */
int UJA1169_WriteReg(uint8_t reg, uint8_t value);
int UJA1169_ReadReg(uint8_t reg, uint8_t *value);

/* 鎵归噺绐佸彂璁块棶锛氳捣濮嬪湴鍧�+杩炵画鏁版嵁锛涘唴閮ㄨ嚜鍔ㄩ�掑鍦板潃 */
int UJA1169_WriteBurst(uint8_t startReg, const uint8_t *data, uint16_t len);
int UJA1169_ReadBurst(uint8_t startReg, uint8_t *data, uint16_t len);

/* 瀵勫瓨鍣ㄤ慨鏀癸細鍏堣鍚庡啓 (val = (val & ~clearMask) | setMask) */
int UJA1169_ModifyReg(uint8_t reg, uint8_t setMask, uint8_t clearMask);
int UJA1169_SetBits(uint8_t reg, uint8_t mask);
int UJA1169_ClearBits(uint8_t reg, uint8_t mask);

/* 鍔熻兘鎺ュ彛锛堣繑鍥炵姸鎬佺爜锛涜皟鐢ㄥ墠闇� Init 鎴愬姛锛� */
int UJA1169_SetMode(uint8_t modeVal);               /* 鍘熷鍐欏�兼帴鍙�(淇濈暀浣嶈皑鎱�) */
int UJA1169_SetModeEnum(UJA1169_ModeType mode);     /* 閫氳繃鏋氫妇璁剧疆妯″紡 */
int UJA1169_GetMainStatus(uint8_t *status);
int UJA1169_EnableWatchdog(bool enable);
int UJA1169_WatchdogRefresh(void);                  /* 鍐欏洖褰撳墠 WDOG_CTRL 瑙﹀彂鍒锋柊 */
int UJA1169_WatchdogConfigure(uint8_t val);         /* 淇敼 WDOG_CTRL锛堟ā寮�/鍛ㄦ湡锛夛紝浼氬悓鏃惰Е鍙戜竴娆″埛鏂� */
int UJA1169_ClearAllEvents(void);                   /* 璇诲洖鍐� 1 娓呴櫎鍏ㄩ儴浜嬩欢锛圵1C锛夛紝鐢ㄤ簬杩� Sleep 鍓� */
int UJA1169_SetCANMode(uint8_t mode);
int UJA1169_GetDeviceID(uint8_t *id);
int UJA1169_GetCachedDeviceID(uint8_t *id);

/* 浜嬩欢瀵勫瓨鍣ㄦ搷浣� */
int UJA1169_ReadAllEvents( uint8_t *globalEvt,
                           uint8_t *sysEvt,
                           uint8_t *supplyEvt,
                           uint8_t *transEvt,
                           uint8_t *wakeEvt );

int UJA1169_ClearEvents(uint8_t globalMask,
                        uint8_t sysMask,
                        uint8_t supplyMask,
                        uint8_t transMask,
                        uint8_t wakeMask);

/* 鍙�夛細鏌ヨ鏄惁宸插垵濮嬪寲 */
bool UJA1169_IsInitialized(void);

/* ================================
 * 绠�鏄撹緟鍔╁畯 (鍔犳嫭鍙烽槻姝㈠畯鍙備笌琛ㄨ揪寮忎紭鍏堢骇闂)
 * ================================ */
#define UJA1169_MODE_NORMAL()  (void)UJA1169_SetMode((uint8_t)MODE_CTRL_NORMAL_MASK)
#define UJA1169_MODE_STANDBY() (void)UJA1169_SetMode((uint8_t)MODE_CTRL_STBY_MASK)
#define UJA1169_MODE_SLEEP()   (void)UJA1169_SetMode((uint8_t)MODE_CTRL_SLEEP_MASK)

/* ================================
 * 寤鸿锛氫繚鐣欎綅鍐欏叆绛栫暐
 * - 鑻ュ悗缁疄鐜颁腑闇�瑕佸啓鍚繚鐣欎綅鐨勫瘎瀛樺櫒锛岃鍏� Read-Modify-Write
 * ================================ */

//bool UJA1169_ApplyDefaultConfig(void);


// === 单步封装接口：用于被 Init 组合调用 ===

// 0) 读取并校验 DeviceID（白名单：CF,C9,EF,E9,CE,EE）
int UJA1169_Step_CheckDeviceId(uint8_t *out_id);

// 1) 清事件（W1C）：对 0x61~0x64 读到的1按位写1；不写 0x60
int UJA1169_Step_ClearPendingEvents(void);

// 2) Fail-safe 复位计数器初始化：RCC=00；可选清 LHC
int UJA1169_Step_ResetCounterInit(void);

// 3) 在 Standby 下配置看门狗（传入“字段值”，已按位对齐/掩码）
//    wmc_field: 例如 WDOG_CTRL_WMC_TIMEOUT
//    nwp_field: 例如 WDOG_CTRL_NWP_128MS
int UJA1169_Step_ConfigWatchdogStandby(uint8_t wmc_field, uint8_t nwp_field);

// 4) 稳压器配置（读改写）：设置 V1RTC / VEXTC 字段
int UJA1169_Step_ConfigRegulators(uint8_t v1rtc_field, uint8_t vextc_field);

// 5) 事件捕获使能（按需设置掩码）
int UJA1169_Step_EnableEventCapture(uint8_t sys_en_mask,
                                    uint8_t supply_en_mask,
                                    uint8_t trans_en_mask,
                                    uint8_t wake_en_mask);

// 6) CAN 收发器基本配置（读改写）：setMask 置1，clrMask 清0
int UJA1169_Step_ConfigCan(uint8_t setMask, uint8_t clrMask);

// 7) 切换 Normal 并校验 MAIN_STATUS（可二次确认）
int UJA1169_Step_EnterNormalAndVerify(void);

// === 组合入口：保留一个 Init ===
int UJA1169_Init(void);


#ifdef __cplusplus
}
#endif

#endif /* UJA1169_H */


