/*
 * Flash_drivers.c
 *
 *  Created on: 2025年9月1日
 *      Author: maode1
 */

#include "Flash_drivers.h"

C40_Ip_VirtualSectorsType vSector ;
C40_Ip_StatusType C40_Ip_Status ;

#define FLS_MASTER_ID                (0U)
#define C40_DATA_SIZE_BYTES           16
#define MAX_ERASE_TIMEOUT_MS          5000

uint32_t timeout = MAX_ERASE_TIMEOUT_MS; // 定义一个最大超时时间

/**
 * @brief 擦除一个指定的 Flash 扇区
 * @param address: 目标扇区内的任何一个地址
 * @return 执行状态：STATUS_SUCCESS 或 STATUS_ERROR
 */
C40_Ip_StatusType DFlash_Erase(uint32_t address)
{
    C40_Ip_StatusType status;
    C40_Ip_VirtualSectorsType vSector;
    uint32_t flashAddr = address; // 使用一个明确的变量名

    /* 1. 根据地址获取扇区号 */
    vSector = C40_Ip_GetSectorNumberFromAddress(flashAddr);
    /* 可以在这里添加检查：if (vSector == INVALID_SECTOR) ... */

    /* 2. 检查并解除扇区写保护 */
    status = C40_Ip_GetLock(vSector);
    if (status == C40_IP_STATUS_SECTOR_PROTECTED)
    {
        status = C40_Ip_ClearLock(vSector, FLS_MASTER_ID);
        if (status != C40_IP_STATUS_SUCCESS)
        {
            /* 解除保护失败，返回错误 */
            return C40_IP_STATUS_ERROR;
        }
    }
    else if (status != C40_IP_STATUS_SUCCESS)
    {
        /* 获取锁状态失败，返回错误 */
        return C40_IP_STATUS_ERROR;
    }

    /* 3. 执行扇区擦除命令 */
    status = C40_Ip_MainInterfaceSectorErase(vSector, FLS_MASTER_ID);
    if (status != C40_IP_STATUS_SUCCESS)
    {
        /* 启动擦除命令失败 */
        return C40_IP_STATUS_ERROR;
    }

    /* 4. 等待擦除操作完成 */
    do {
        //TestDelay(50000);
        status = C40_Ip_MainInterfaceSectorEraseStatus();
        if (timeout-- == 0) {
            return C40_IP_STATUS_ERROR_TIMEOUT; // 超时错误
        }
    } while (status == C40_IP_STATUS_BUSY);

    /* 5. 检查最终擦除状态 */
    if (status != C40_IP_STATUS_SUCCESS)
    {
        /* 擦除操作失败 */
        return C40_IP_STATUS_ERROR;
    }

    /* 6. 清除数据缓存 (Critical Step!) */
    /* 确保CPU后续读取的是Flash里的新数据，而不是缓存里的旧数据 */
    /*
    Cache_Ip_CleanByAddr(CACHE_IP_CORE,
                         CACHE_IP_DATA,
                         true, // 可能是失效(Invalidate)而非清理(Clean)，需查文档
                         flashAddr,
                         C40_DATA_SIZE_BYTES); // 最好使用实际的扇区大小
     */

    /* 7. (可选) 重新保护扇区（如果需要）*/
    // status = C40_Ip_SetLock(vSector, FLS_MASTER_ID);

    return C40_IP_STATUS_ERROR;
}

__attribute__((section(".ramcode")))C40_Ip_StatusType PFlash_Erase(uint32_t address)
{
	FLASH_Type *base = C40_Ip_pFlashBaseAddress;

	//获取扇区地址
	vSector = C40_Ip_GetSectorNumberFromAddress(address);

	/* Unlock sector */
	if (C40_IP_STATUS_SECTOR_PROTECTED == C40_Ip_GetLock(vSector))
	{
		C40_Ip_Status = C40_Ip_ClearLock(vSector, FLS_MASTER_ID);
		while(C40_IP_STATUS_SUCCESS != C40_Ip_Status);
	}

	 C40_Ip_MainInterfaceSectorErase(vSector,FLS_MASTER_ID);

	do
	{
		/*wait*/
//		TestDelay(5000);
	}
	while(0 == (base->MCRS & FLASH_MCRS_DONE_MASK));
    /* terminate erase operation */
	base->MCR &= ~FLASH_MCR_EHV_MASK;
    /* Terminate erase operation */
	base->MCR &= ~FLASH_MCR_ERS_MASK;

	/*清除缓存*/
	//Cache_Ip_CleanByAddr(CACHE_IP_CORE,CACHE_IP_DATA,true,flashAddr,C40_DATA_SIZE_BYTES);
	return C40_IP_STATUS_SUCCESS;
}

__attribute__((section(".ramcode")))C40_Ip_StatusType PFlash_Write(uint32 LogicalAddress,uint32 Length, uint8 *SourceAddressPtr)
{
	FLASH_Type *base = C40_Ip_pFlashBaseAddress;
	C40_Ip_MainInterfaceWrite( LogicalAddress,Length,SourceAddressPtr, FLS_MASTER_ID);

	do
	{
		/*wait*/
//		TestDelay(5000);
	}
	while(0 == (base->MCRS & FLASH_MCRS_DONE_MASK));
    /* terminate erase operation */
	base->MCR &= ~FLASH_MCR_EHV_MASK;
    /* Terminate erase operation */
	base->MCR &= ~FLASH_MCR_ERS_MASK;

	/*清除缓存*/
	//Cache_Ip_CleanByAddr(CACHE_IP_CORE,CACHE_IP_DATA,true,flashAddr,C40_DATA_SIZE_BYTES);
	return C40_IP_STATUS_SUCCESS;
}

C40_Ip_StatusType DFlash_Write(uint32 flashAddr,uint32 Length,uint8 *SourceAddressPtr)
{
	C40_Ip_Status = C40_Ip_MainInterfaceWrite(flashAddr,Length,SourceAddressPtr,FLS_MASTER_ID);
	while(C40_IP_STATUS_SUCCESS != C40_Ip_Status);
	do
	{
		//TestDelay(50000);
		C40_Ip_Status = C40_Ip_MainInterfaceWriteStatus();
	}
	while (C40_IP_STATUS_BUSY == C40_Ip_Status);
	return C40_IP_STATUS_SUCCESS;
}

/**
 * @brief 擦除从指定地址开始的一定大小的 Flash 区域
 * @param startAddress: 起始地址
 * @param size: 要擦除的区域总大小（字节）
 * @return 执行状态：STATUS_SUCCESS 或 STATUS_ERROR
 */
C40_Ip_StatusType Flash_EraseRange(uint32_t startAddress, uint32_t size)
{
	C40_Ip_StatusType status;
    uint32_t endAddress = startAddress + size;
    uint32_t currentAddress = startAddress;
    uint32_t sectorSize = 0x2000; // 8KB

    /* 遍历所有需要擦除的扇区 */
    while (currentAddress < endAddress)
    {
    	if(startAddress > 0x600000)
    	{
    		status = DFlash_Erase(currentAddress);
    	}
    	else
        {
    		status = PFlash_Erase(currentAddress);
        }

        if (status != C40_IP_STATUS_SUCCESS) {
            return C40_IP_STATUS_ERROR;
        }

        currentAddress += sectorSize;
    }

    return C40_IP_STATUS_ERROR;
}

C40_Ip_StatusType Flash_WriteRange(uint32 LogicalAddress,uint32 Length, uint8 *SourceAddressPtr)
{
	C40_Ip_StatusType status;
	if(LogicalAddress > 0x600000)
	{
		status = DFlash_Write(LogicalAddress,Length,SourceAddressPtr);
	}
	else
    {
		status = PFlash_Write(LogicalAddress,Length,SourceAddressPtr);
    }
	return status;
}

C40_Ip_StatusType Flash_ReadRange(uint32 LogicalAddress,uint32 Length, uint8 *SourceAddressPtr)
{

	C40_Ip_Status = C40_Ip_Read(LogicalAddress,Length,SourceAddressPtr);
			while(C40_IP_STATUS_SUCCESS != C40_Ip_Status);
	return C40_IP_STATUS_SUCCESS;
}

void Flash_drivers_Init(void)
{
	//初始化fls
	C40_Ip_Status = C40_Ip_Init(&C40_Ip_InitCfg);
	while(C40_IP_STATUS_SUCCESS != C40_Ip_Status);
}

