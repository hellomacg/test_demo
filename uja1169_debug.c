#include "UJA1169.h"
#include <stdio.h>

/**
 * @brief  Dump all UJA1169A registers (0x00 ~ 0x7F)
 * @note   调试用，打印到串口或日志
 */
void UJA1169_DumpAllRegs(void)
{
    printf("===== UJA1169 Register Dump =====\n");
    for (uint8_t addr = 0x00; addr <= 0x7F; addr++)
    {
        uint8_t val = 0x00U;
        int st = UJA1169_ReadReg(addr, &val);
        if (st == 0)
        {
            printf("Reg[0x%02X] = 0x%02X\n", addr, val);
        }
        else
        {
            printf("Reg[0x%02X] = <ERR %d>\n", addr, st);
        }
    }
    printf("=================================\n");
}
