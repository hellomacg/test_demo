/*
 * Flash_drivers.c
 *
 *  Created on: 2025��9��1��
 *      Author: maode1
 */

#include "Flash_drivers.h"

C40_Ip_VirtualSectorsType vSector ;
C40_Ip_StatusType C40_Ip_Status ;

#define FLS_MASTER_ID                (0U)
#define C40_DATA_SIZE_BYTES           16
#define MAX_ERASE_TIMEOUT_MS          5000

uint32_t timeout = MAX_ERASE_TIMEOUT_MS; // ����һ�����ʱʱ��

/**
 * @brief ����һ��ָ���� Flash ����
 * @param address: Ŀ�������ڵ��κ�һ����ַ
 * @return ִ��״̬��STATUS_SUCCESS �� STATUS_ERROR
 */
C40_Ip_StatusType DFlash_Erase(uint32_t address)
{
    C40_Ip_StatusType status;
    C40_Ip_VirtualSectorsType vSector;
    uint32_t flashAddr = address; // ʹ��һ����ȷ�ı�����

    /* 1. ���ݵ�ַ��ȡ������ */
    vSector = C40_Ip_GetSectorNumberFromAddress(flashAddr);
    /* ������������Ӽ�飺if (vSector == INVALID_SECTOR) ... */

    /* 2. ��鲢�������д���� */
    status = C40_Ip_GetLock(vSector);
    if (status == C40_IP_STATUS_SECTOR_PROTECTED)
    {
        status = C40_Ip_ClearLock(vSector, FLS_MASTER_ID);
        if (status != C40_IP_STATUS_SUCCESS)
        {
            /* �������ʧ�ܣ����ش��� */
            return C40_IP_STATUS_ERROR;
        }
    }
    else if (status != C40_IP_STATUS_SUCCESS)
    {
        /* ��ȡ��״̬ʧ�ܣ����ش��� */
        return C40_IP_STATUS_ERROR;
    }

    /* 3. ִ�������������� */
    status = C40_Ip_MainInterfaceSectorErase(vSector, FLS_MASTER_ID);
    if (status != C40_IP_STATUS_SUCCESS)
    {
        /* ������������ʧ�� */
        return C40_IP_STATUS_ERROR;
    }

    /* 4. �ȴ������������ */
    do {
        //TestDelay(50000);
        status = C40_Ip_MainInterfaceSectorEraseStatus();
        if (timeout-- == 0) {
            return C40_IP_STATUS_ERROR_TIMEOUT; // ��ʱ����
        }
    } while (status == C40_IP_STATUS_BUSY);

    /* 5. ������ղ���״̬ */
    if (status != C40_IP_STATUS_SUCCESS)
    {
        /* ��������ʧ�� */
        return C40_IP_STATUS_ERROR;
    }

    /* 6. ������ݻ��� (Critical Step!) */
    /* ȷ��CPU������ȡ����Flash��������ݣ������ǻ�����ľ����� */
    /*
    Cache_Ip_CleanByAddr(CACHE_IP_CORE,
                         CACHE_IP_DATA,
                         true, // ������ʧЧ(Invalidate)��������(Clean)������ĵ�
                         flashAddr,
                         C40_DATA_SIZE_BYTES); // ���ʹ��ʵ�ʵ�������С
     */

    /* 7. (��ѡ) ���±��������������Ҫ��*/
    // status = C40_Ip_SetLock(vSector, FLS_MASTER_ID);

    return C40_IP_STATUS_ERROR;
}

__attribute__((section(".ramcode")))C40_Ip_StatusType PFlash_Erase(uint32_t address)
{
	FLASH_Type *base = C40_Ip_pFlashBaseAddress;

	//��ȡ������ַ
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

	/*�������*/
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

	/*�������*/
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
 * @brief ������ָ����ַ��ʼ��һ����С�� Flash ����
 * @param startAddress: ��ʼ��ַ
 * @param size: Ҫ�����������ܴ�С���ֽڣ�
 * @return ִ��״̬��STATUS_SUCCESS �� STATUS_ERROR
 */
C40_Ip_StatusType Flash_EraseRange(uint32_t startAddress, uint32_t size)
{
	C40_Ip_StatusType status;
    uint32_t endAddress = startAddress + size;
    uint32_t currentAddress = startAddress;
    uint32_t sectorSize = 0x2000; // 8KB

    /* ����������Ҫ���������� */
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
	//��ʼ��fls
	C40_Ip_Status = C40_Ip_Init(&C40_Ip_InitCfg);
	while(C40_IP_STATUS_SUCCESS != C40_Ip_Status);
}

