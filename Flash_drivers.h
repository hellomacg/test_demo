/*
 * Flash_drivers.h
 *
 *  Created on: 2025Äê9ÔÂ1ÈÕ
 *      Author: maode1
 */

#ifndef FLASH_DRIVERS_H_
#define FLASH_DRIVERS_H_
#include "C40_Ip.h"
#include "cache_Ip.h"
#include "stdlib.h"

void User_FlexCAN_Init(void);
C40_Ip_StatusType Flash_EraseRange(uint32 startAddress, uint32 size);
C40_Ip_StatusType Flash_WriteRange(uint32 LogicalAddress,uint32 Length, uint8 *SourceAddressPtr);
C40_Ip_StatusType Flash_ReadRange(uint32 LogicalAddress,uint32 Length, uint8 *SourceAddressPtr);


#endif /* FLASH_DRIVERS_H_ */
