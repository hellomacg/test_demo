
// filepath: i:\S32dsWorkspace_1\CPM_beiqi\UJA1169_Driver\src\uja1169_test.c
#include "UJA1169.h"
#include <stdio.h>
#include <string.h>

static void test_mem_block(void)
{
    uint8_t wr[4] = {0x11,0x22,0x33,0x44};
    uint8_t rd[4] = {0};
    if (UJA1169_WriteBurst(REG_MEM0, wr, 4)==0 &&
        UJA1169_ReadBurst(REG_MEM0, rd, 4)==0 &&
        memcmp(wr, rd, 4)==0)
        printf("[OK] MEM burst\n");
    else
        printf("[FAIL] MEM burst\n");
}

void UJA1169_RunSmoke(void)
{
    if (UJA1169_Init()!=0) { printf("Init fail\n"); return; }
//    if (!UJA1169_ApplyDefaultConfig()) { printf("Config fail\n"); return; }

    uint8_t id=0;
    (void)UJA1169_GetDeviceID(&id);
    printf("ID=0x%02X\n", id);

    test_mem_block();

    uint8_t g,s,sp,t,w;
    if (UJA1169_ReadAllEvents(&g,&s,&sp,&t,&w)==0)
        printf("Evt g=%02X s=%02X sp=%02X t=%02X w=%02X\n", g,s,sp,t,w);

    (void)UJA1169_ClearAllEvents();
    (void)UJA1169_WatchdogRefresh();
}

