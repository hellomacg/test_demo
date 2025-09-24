#include "UJA1169.h"
#include "Lpspi_Ip.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define UJA1169_ADDR_MAX        (0x7FU)
#define UJA1169_ADDR_MASK       (0x7FU)
#define UJA1169_LPSPI_INSTANCE   (0U)
#define UJA1169_LPSPI_TIMEOUT_US (5000U)
//MASTER_EXTERNAL_DEVICE
typedef struct {
    bool    initialized;
    uint8_t deviceId;
} UJA1169_Context_t;

static UJA1169_Context_t gUjaCtx = { false, 0u };

#define UJA1169_CHECK_INIT()  do{ if(!gUjaCtx.initialized){ return UJA1169_STATUS_NOT_INIT; } }while(0)

/* ========== SPI 閫傞厤鍗犱綅锛堥渶鐢ㄥ疄闄� LPSPI / SPI HAL 鏇挎崲锛� ==========
 * 杩斿洖 0 琛ㄧず鎴愬姛锛涢潪 0 澶辫触銆�
 */
static int UJA1169_SpiTransfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if ((tx == NULL) || (rx == NULL) || (len == 0U))
    {
        return -1;
    }
    Lpspi_Ip_StatusType st = Lpspi_Ip_SyncTransmit(
    		&Lpspi_Ip_DeviceAttributes_SpiExternalDevice_0_Instance_0,
                                tx,
                                rx,
                                (uint32)len,
                                UJA1169_LPSPI_TIMEOUT_US);
    return (st == LPSPI_IP_STATUS_SUCCESS) ? 0 : -1;
}

/* 浠呭湴鍧�锛屾棤 R/W 浣� */
static inline uint8_t UJA1169_AddressByte(uint8_t reg)
{
    return (uint8_t)(reg & UJA1169_ADDR_MASK);
}

/* 鍙傛暟涓庡垵濮嬪寲妫�鏌ュ畯 */
#define UJA1169_CHECK_ADDR(a)       do{ if((a) > UJA1169_ADDR_MAX){ return -3; } }while(0)

/* ========== 浣庡眰锛氬崟瀵勫瓨鍣ㄨ闂� ========== */
static int UJA1169_LL_Write(uint8_t reg, uint8_t val)
{
    UJA1169_CHECK_ADDR(reg);
    uint8_t tx[2];
    uint8_t rx[2] = {0};
    tx[0] = UJA1169_AddressByte(reg);
    tx[1] = val;
    return UJA1169_SpiTransfer(tx, rx, 2U);
}

static int UJA1169_LL_Read(uint8_t reg, uint8_t *val)
{
    if (val == NULL) return -1;
    UJA1169_CHECK_ADDR(reg);
    uint8_t tx[2];
    uint8_t rx[2] = {0};
    tx[0] = UJA1169_AddressByte(reg);
    tx[1] = 0x00U; /* dummy clock */
    if (0 == UJA1169_SpiTransfer(tx, rx, 2U))
    {
        *val = rx[1];
        return 0;
    }
    return -1;
}

/* ========== 绐佸彂璁块棶锛堣捣濮嬪湴鍧�鑷姩閫掑锛� ========== */
static int UJA1169_LL_WriteBurst(uint8_t startReg, const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) return -1;
    /* 鏈湴鍧�涓嶅緱瓒呰繃 0x7F */
    if ((startReg > UJA1169_ADDR_MAX) || ((uint16_t)startReg + (len - 1U) > UJA1169_ADDR_MAX))
        return -3;

    /* 棣栧瓧鑺傚湴鍧� + len 鏁版嵁瀛楄妭 */
    if (len > 63U) return -4; /* 闃叉鏍堣繃澶э紱闇�瑕佹洿闀垮彲鏀规垚鍔ㄦ�佹垨鍒嗘 */
    uint8_t tx[1 + 63];
    uint8_t rx[1 + 63];
    tx[0] = UJA1169_AddressByte(startReg);
    for (uint16_t i = 0; i < len; i++) tx[1U + i] = data[i];
    return UJA1169_SpiTransfer(tx, rx, (uint16_t)(1U + len));
}

static int UJA1169_LL_ReadBurst(uint8_t startReg, uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) return -1;
    if ((startReg > UJA1169_ADDR_MAX) || ((uint16_t)startReg + (len - 1U) > UJA1169_ADDR_MAX))
        return -3;

    if (len > 63U) return -4;
    uint8_t tx[1 + 63] = {0};
    uint8_t rx[1 + 63] = {0};
    tx[0] = UJA1169_AddressByte(startReg);
    /* 鍚庣画 dummy=0 */
    if (0 == UJA1169_SpiTransfer(tx, rx, (uint16_t)(1U + len)))
    {
        for (uint16_t i = 0; i < len; i++) data[i] = rx[1U + i];
        return 0;
    }
    return -1;
}

/* ========== 鍏叡 API ========== */
// uja1169.c
// 鍗曚竴鍏ュ彛锛氬畬鎴愯瘑鍒� + 娓呭満 + 鍔熻兘閰嶇疆 + 杩涘叆 Normal
static int UJA1169_LL_SetModeEnum(UJA1169_ModeType mode)
{
    uint8_t val;
    switch (mode)
    {
        case UJA1169_MODE_NORMAL:  val = MODE_CTRL_NORMAL_MASK; break;
        case UJA1169_MODE_STANDBY: val = MODE_CTRL_STBY_MASK;   break;
        case UJA1169_MODE_SLEEP:   val = MODE_CTRL_SLEEP_MASK;  break;
        default: return UJA1169_STATUS_BAD_PARAM;
    }
    return UJA1169_LL_Write(REG_MODE_CTRL, val);
}

// ---------------- 低层读改写工具改用 LL ----------------
static int UJA1169_LL_UpdateMasked(uint8_t reg, uint8_t clearMask, uint8_t setMask)
{
    uint8_t v;
    if (0 != UJA1169_LL_Read(reg, &v)) return UJA1169_STATUS_SPI_FAIL;
    v = (uint8_t)((v & (uint8_t)~clearMask) | setMask);
    return (0 == UJA1169_LL_Write(reg, v)) ? UJA1169_STATUS_OK : UJA1169_STATUS_SPI_FAIL;
}

// ---------------- Step 函数全部改用 LL ----------------

// 1) 清事件 (W1C) (LL)
int UJA1169_Step_ClearPendingEvents(void)
{
    uint8_t v;
    if (0 == UJA1169_LL_Read(REG_SYS_EVT_STAT, &v)    && v) (void)UJA1169_LL_Write(REG_SYS_EVT_STAT,    v);
    if (0 == UJA1169_LL_Read(REG_SUPPLY_EVT_STAT, &v) && v) (void)UJA1169_LL_Write(REG_SUPPLY_EVT_STAT, v);
    if (0 == UJA1169_LL_Read(REG_TRANS_EVT_STAT, &v)  && v) (void)UJA1169_LL_Write(REG_TRANS_EVT_STAT,  v);
    if (0 == UJA1169_LL_Read(REG_WAKE_EVT_STAT, &v)   && v) (void)UJA1169_LL_Write(REG_WAKE_EVT_STAT,   v);
    return UJA1169_STATUS_OK;
}

// 2) Fail-safe 复位计数器初始化 (假设 RCC 为 W1C 单 bit)
int UJA1169_Step_ResetCounterInit(void)
{
    uint8_t v;
    if (0 != UJA1169_LL_Read(REG_FAILSAFE_CTRL, &v)) return UJA1169_STATUS_SPI_FAIL;
#ifdef FAILSAFE_CTRL_RCC
    v |= FAILSAFE_CTRL_RCC; /* 写1清 */
#endif
#ifdef FAILSAFE_CTRL_LHC
    v &= (uint8_t)~FAILSAFE_CTRL_LHC;
#endif
    return (0 == UJA1169_LL_Write(REG_FAILSAFE_CTRL, v)) ? UJA1169_STATUS_OK : UJA1169_STATUS_SPI_FAIL;
}

// 3) 看门狗配置 (Standby) (LL)
int UJA1169_Step_ConfigWatchdogStandby(uint8_t wmc_field, uint8_t nwp_field)
{
    if (0 != UJA1169_LL_SetModeEnum(UJA1169_MODE_STANDBY)) return UJA1169_STATUS_SPI_FAIL;
    uint8_t v = 0u;
    if (0 != UJA1169_LL_Read(REG_WDOG_CTRL, &v)) return UJA1169_STATUS_SPI_FAIL;
#if defined(WDOG_CTRL_WMC_MASK) && defined(WDOG_CTRL_NWP_MASK)
    v &= (uint8_t)~(WDOG_CTRL_WMC_MASK | WDOG_CTRL_NWP_MASK);
    v |= (uint8_t)(wmc_field | nwp_field); /* 入参已对齐，不再二次 & mask */
#endif
    if (0 != UJA1169_LL_Write(REG_WDOG_CTRL, v)) return UJA1169_STATUS_SPI_FAIL;
    /* 刷新：再次写同值可选 */
    return UJA1169_STATUS_OK;
}

// 4) 稳压器配置
int UJA1169_Step_ConfigRegulators(uint8_t v1rtc_field, uint8_t vextc_field)
{
#if defined(REGU_CTRL_V1RTC_MASK) && defined(REGU_CTRL_VEXTC_MASK)
    uint8_t clear = (uint8_t)(REGU_CTRL_V1RTC_MASK | REGU_CTRL_VEXTC_MASK);
    uint8_t set   = (uint8_t)((v1rtc_field & REGU_CTRL_V1RTC_MASK) |
                              (vextc_field & REGU_CTRL_VEXTC_MASK));
    return UJA1169_LL_UpdateMasked(REG_REGU_CTRL, clear, set);
#else
    (void)v1rtc_field; (void)vextc_field;
    return UJA1169_STATUS_BAD_PARAM;
#endif
}

// 5) 事件捕获使能
int UJA1169_Step_EnableEventCapture(uint8_t sys_en_mask,
                                    uint8_t supply_en_mask,
                                    uint8_t trans_en_mask,
                                    uint8_t wake_en_mask)
{
    if (sys_en_mask)    (void)UJA1169_LL_UpdateMasked(REG_SYS_EVENT_EN,    0u, sys_en_mask);
    if (supply_en_mask) (void)UJA1169_LL_UpdateMasked(REG_SUPPLY_EVENT_EN, 0u, supply_en_mask);
    if (trans_en_mask)  (void)UJA1169_LL_UpdateMasked(REG_TRANS_EVENT_EN,  0u, trans_en_mask);
#ifdef REG_WAKE_EVENT_EN
    if (wake_en_mask)   (void)UJA1169_LL_UpdateMasked(REG_WAKE_EVENT_EN,   0u, wake_en_mask);
#endif
    return UJA1169_STATUS_OK;
}

// 6) CAN 配置
int UJA1169_Step_ConfigCan(uint8_t setMask, uint8_t clrMask)
{
    uint8_t v;
    if (0 != UJA1169_LL_Read(REG_CAN_CTRL, &v)) return UJA1169_STATUS_SPI_FAIL;
    v = (uint8_t)((v & (uint8_t)~clrMask) | setMask);
    return (0 == UJA1169_LL_Write(REG_CAN_CTRL, v)) ? UJA1169_STATUS_OK : UJA1169_STATUS_SPI_FAIL;
}

// 7) 切 Normal
int UJA1169_Step_EnterNormalAndVerify(void)
{
    if (0 != UJA1169_LL_SetModeEnum(UJA1169_MODE_NORMAL)) return UJA1169_STATUS_SPI_FAIL;
    uint8_t ms;
    if (0 == UJA1169_LL_Read(REG_MAIN_STATUS, &ms)) {
        /* 可加校验 MAIN_STATUS_NMS */
    }
    return UJA1169_STATUS_OK;
}

// 设备ID校验 (LL)
int UJA1169_Step_CheckDeviceId(uint8_t *out_id)
{
    uint8_t id = 0u;
    if (0 != UJA1169_LL_Read(REG_DEVICE_ID, &id)) return UJA1169_STATUS_SPI_FAIL;
    if (id == 0x00u || id == 0xFFu) return UJA1169_STATUS_ID_FAULT;
    static const uint8_t kIds[] = {0xCF,0xC9,0xEF,0xE9,0xCE,0xEE};
    bool ok = false;
    for (size_t i=0; i< (sizeof(kIds)/sizeof(kIds[0])); ++i) if (id == kIds[i]) { ok = true; break; }
    if (!ok) return UJA1169_STATUS_ID_FAULT;
    if (out_id) *out_id = id;
    gUjaCtx.deviceId = id;
    return UJA1169_STATUS_OK;
}

// Init：仅 LL；成功后恢复 API 保护
int UJA1169_Init(void)
{
    if (gUjaCtx.initialized) return UJA1169_STATUS_OK;
    int st;
#define RET_ON_ERR_LL(x) do{ st=(x); if(st) return st; }while(0)		//error stop next step
    RET_ON_ERR_LL( UJA1169_Step_CheckDeviceId(NULL) );
    RET_ON_ERR_LL( UJA1169_Step_ClearPendingEvents() );
    RET_ON_ERR_LL( UJA1169_Step_ResetCounterInit() );
    RET_ON_ERR_LL( UJA1169_Step_ConfigWatchdogStandby(WDOG_CTRL_WMC_TIMEOUT, WDOG_CTRL_NWP_128MS) );
    RET_ON_ERR_LL( UJA1169_Step_ConfigRegulators(V1RTC_80PCT, VEXTC_ON_IN_NORMAL) );
    RET_ON_ERR_LL( UJA1169_Step_EnableEventCapture(
                       SYS_EVENT_EN_SPIFE | SYS_EVENT_EN_OTWE,
                       SUPPLY_EVENT_EN_V1UE | SUPPLY_EVENT_EN_V2UE,
                       TRANS_EVENT_EN_CWE  | TRANS_EVENT_EN_CFE,
                       0) );
    RET_ON_ERR_LL( UJA1169_Step_ConfigCan(CAN_CTRL_CMC, 0) );
    RET_ON_ERR_LL( UJA1169_Step_EnterNormalAndVerify() );
    gUjaCtx.initialized = true;
    return UJA1169_STATUS_OK;
}

// ---------------- 公开 API 恢复初始化检查 ----------------
int UJA1169_WriteReg(uint8_t reg, uint8_t value)
{
    UJA1169_CHECK_INIT();
    return (0 == UJA1169_LL_Write(reg, value)) ? UJA1169_STATUS_OK : UJA1169_STATUS_SPI_FAIL;
}

int UJA1169_ReadReg(uint8_t reg, uint8_t *value)
{
    UJA1169_CHECK_INIT();
    if (!value) return UJA1169_STATUS_BAD_PARAM;
    return (0 == UJA1169_LL_Read(reg, value)) ? UJA1169_STATUS_OK : UJA1169_STATUS_SPI_FAIL;
}

int UJA1169_WriteBurst(uint8_t startReg, const uint8_t *data, uint16_t len)
{
    UJA1169_CHECK_INIT();
    return UJA1169_LL_WriteBurst(startReg, data, len);
}

int UJA1169_ReadBurst(uint8_t startReg, uint8_t *data, uint16_t len)
{
    UJA1169_CHECK_INIT();
    return UJA1169_LL_ReadBurst(startReg, data, len);
}

int UJA1169_ModifyReg(uint8_t reg, uint8_t setMask, uint8_t clearMask)
{
    UJA1169_CHECK_INIT();
    uint8_t val;
    int st = UJA1169_ReadReg(reg, &val);
    if (st != UJA1169_STATUS_OK) return st;
    val = (uint8_t)((val & (uint8_t)(~clearMask)) | setMask);
    return UJA1169_WriteReg(reg, val);
}

int UJA1169_SetBits(uint8_t reg, uint8_t mask)
{
    return UJA1169_ModifyReg(reg, mask, 0U);
}

int UJA1169_ClearBits(uint8_t reg, uint8_t mask)
{
    return UJA1169_ModifyReg(reg, 0U, mask);
}

int UJA1169_SetMode(uint8_t modeVal)
{
    return UJA1169_WriteReg(REG_MODE_CTRL, modeVal);
}

int UJA1169_GetMainStatus(uint8_t *status)
{
    return UJA1169_ReadReg(REG_MAIN_STATUS, status);
}

int UJA1169_EnableWatchdog(bool enable)
{
    return enable
        ? UJA1169_SetBits(REG_WDOG_CTRL, WDOG_CTRL_EN_MASK)
        : UJA1169_ClearBits(REG_WDOG_CTRL, WDOG_CTRL_EN_MASK);
}

int UJA1169_SetCANMode(uint8_t mode)
{
    return UJA1169_WriteReg(REG_CAN_CTRL, mode);
}

int UJA1169_GetDeviceID(uint8_t *id)
{
    return UJA1169_ReadReg(REG_DEVICE_ID, id);
}

bool UJA1169_IsInitialized(void)
{
    return gUjaCtx.initialized;
}

/* ========== 杩藉姞鍔熻兘瀹炵幇 ========== */

/* 鏋氫妇妯″紡鏄犲皠 */
int UJA1169_SetModeEnum(UJA1169_ModeType mode)
{
    uint8_t val;
    switch (mode)
    {
        case UJA1169_MODE_NORMAL:  val = MODE_CTRL_NORMAL_MASK; break;
        case UJA1169_MODE_STANDBY: val = MODE_CTRL_STBY_MASK;   break;
        case UJA1169_MODE_SLEEP:   val = MODE_CTRL_SLEEP_MASK;  break;
        default: return UJA1169_STATUS_BAD_PARAM;
    }
    return UJA1169_SetMode(val);
}

/*
 * Watchdog 鍒锋柊锛�
 * UJA1169 瑙勮寖锛氬 WDOG_CTRL(0x00) 鐨勪换浣曚竴娆℃湁鏁堝啓鍏ラ兘浼氳Е鍙戝埛鏂般��
 * 涓嶉渶瑕佺壒娈婂簭鍒�/榄旀暟銆備负閬垮厤鏃犳剰鏀瑰彉閰嶇疆锛屽仛娉曪細
 *  1) 璇诲綋鍓� WDOG_CTRL
 *  2) 鍘熷�煎啀鍐欏洖 = 浠呭埛鏂�
 */
int UJA1169_WatchdogRefresh(void)
{
    UJA1169_CHECK_INIT();
    uint8_t cur;
    if (0 != UJA1169_ReadReg(REG_WDOG_CTRL, &cur))
        return UJA1169_STATUS_SPI_FAIL;
    return UJA1169_WriteReg(REG_WDOG_CTRL, cur);
}

/*
 * Watchdog 閰嶇疆锛�
 * 鐩存帴鍐欏叆鏂� WDOG_CTRL 鍊硷紱鑻ユ敼鍙樻ā寮�/鍛ㄦ湡锛屾柊閰嶇疆绔嬪嵆鐢熸晥骞跺悓鏃跺畬鎴愪竴娆″埛鏂般��
 * 寤鸿锛氭寜鐓ф墜鍐屽湪鍏佽鐨勫姛鑰�/杩愯妯″紡涓嬭皟鐢紙閫氬父 Standby 涓嬫墠鏀规ā寮忥紝Normal 涓嬪彧鍒锋柊锛夈��
 * 涓嶅湪姝ゅ嚱鏁板唴寮哄埗妫�鏌ユā寮忥紝浜ょ敱涓婂眰绛栫暐鎺у埗銆�
 */
int UJA1169_WatchdogConfigure(uint8_t val)
{
    UJA1169_CHECK_INIT();
    return UJA1169_WriteReg(REG_WDOG_CTRL, val);
}

/* 璇诲彇鎵�鏈変簨浠跺瘎瀛樺櫒 */
int UJA1169_ReadAllEvents(uint8_t *globalEvt,
                          uint8_t *sysEvt,
                          uint8_t *supplyEvt,
                          uint8_t *transEvt,
                          uint8_t *wakeEvt)
{
    UJA1169_CHECK_INIT();
    if ((globalEvt==NULL) || (sysEvt==NULL) || (supplyEvt==NULL) ||
        (transEvt==NULL) || (wakeEvt==NULL))
    {
        return UJA1169_STATUS_BAD_PARAM;
    }

    if (0 != UJA1169_ReadReg(REG_GLOBAL_EVT_STAT, globalEvt))  return UJA1169_STATUS_SPI_FAIL;
    if (0 != UJA1169_ReadReg(REG_SYS_EVT_STAT,    sysEvt))     return UJA1169_STATUS_SPI_FAIL;
    if (0 != UJA1169_ReadReg(REG_SUPPLY_EVT_STAT, supplyEvt))  return UJA1169_STATUS_SPI_FAIL;
    if (0 != UJA1169_ReadReg(REG_TRANS_EVT_STAT,  transEvt))   return UJA1169_STATUS_SPI_FAIL;
    if (0 != UJA1169_ReadReg(REG_WAKE_EVT_STAT,   wakeEvt))    return UJA1169_STATUS_SPI_FAIL;

    return UJA1169_STATUS_OK;
}

int UJA1169_ClearEvents(uint8_t globalMask,
                        uint8_t sysMask,
                        uint8_t supplyMask,
                        uint8_t transMask,
                        uint8_t wakeMask)
{
    (void)globalMask; /* 0x60 只读汇总: 忽略 */
    UJA1169_CHECK_INIT();
    int st;

    if (sysMask)
    {
        st = UJA1169_WriteReg(REG_SYS_EVT_STAT, sysMask);
        if (st) return st;
    }
    if (supplyMask)
    {
        st = UJA1169_WriteReg(REG_SUPPLY_EVT_STAT, supplyMask);
        if (st) return st;
    }
    if (transMask)
    {
        st = UJA1169_WriteReg(REG_TRANS_EVT_STAT, transMask);
        if (st) return st;
    }
    if (wakeMask)
    {
        st = UJA1169_WriteReg(REG_WAKE_EVT_STAT, wakeMask);
        if (st) return st;
    }
    return UJA1169_STATUS_OK;
}

/* 鍙�夛細娓呴櫎鎵�鏈夊綋鍓嶄簨浠讹紙灏嗗悇浜嬩欢鐘舵�佸瘎瀛樺櫒璇诲嚭鍚庡洖鍐欙紝鎴栫洿鎺� 0xFF锛�
 * 杩欓噷閲囩敤鈥滆鍑哄啀鍥炲啓鈥濇柟寮忥紝閬垮厤娓呴櫎淇濈暀浣嶉闄┿��
 */
int UJA1169_ClearAllEvents(void)
{
    UJA1169_CHECK_INIT();
    uint8_t v;
    if (0 == UJA1169_ReadReg(REG_SYS_EVT_STAT, &v)    && v) (void)UJA1169_WriteReg(REG_SYS_EVT_STAT,    v);
    if (0 == UJA1169_ReadReg(REG_SUPPLY_EVT_STAT, &v) && v) (void)UJA1169_WriteReg(REG_SUPPLY_EVT_STAT, v);
    if (0 == UJA1169_ReadReg(REG_TRANS_EVT_STAT, &v)  && v) (void)UJA1169_WriteReg(REG_TRANS_EVT_STAT,  v);
    if (0 == UJA1169_ReadReg(REG_WAKE_EVT_STAT, &v)   && v) (void)UJA1169_WriteReg(REG_WAKE_EVT_STAT,   v);
    return UJA1169_STATUS_OK;
}

int UJA1169_GetCachedDeviceID(uint8_t *id)
{
    UJA1169_CHECK_INIT();
    if (!id) return UJA1169_STATUS_BAD_PARAM;
    *id = gUjaCtx.deviceId;
    return UJA1169_STATUS_OK;
}
