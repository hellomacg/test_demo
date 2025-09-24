#include "UJA1169.h"
#include "UJA1169_Regs.h"
#include <stdio.h>

// 说明：底层芯片探测使用 uja1169.c 中的 int UJA1169_Init(void)
// 这里提供“功能/默认配置”封装：在底层 Init 成功后调用
// 若你仍需要原先 bool UJA1169_Init(void) 版本，请重命名为 UJA1169_ApplyDefaultConfig

/* 辅助：打印并返回失败 */
#define UJA1169_CFG_CHECK(callDesc, expr)                \
    do {                                                 \
        int _st = (expr);                                \
        if (_st != 0) {                                  \
            printf("UJA1169 CFG ERROR: %s failed (%d)\n", callDesc, _st); \
            return false;                                \
        }                                                \
    } while(0)


