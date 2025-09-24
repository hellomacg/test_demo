/*==================================================================================================
* Project : RTD AUTOSAR 4.7
* Platform : CORTEXM
* Peripheral : S32K3XX
* Dependencies : none
*
* Autosar Version : 4.7.0
* Autosar Revision : ASR_REL_4_7_REV_0000
* Autosar Conf.Variant :
* SW Version : 4.0.0
* Build Version : S32K3_RTD_4_0_0_HF02_D2407_ASR_REL_4_7_REV_0000_20240725
*
* Copyright 2020 - 2024 NXP
*
* NXP Confidential. This software is owned or controlled by NXP and may only be
* used strictly in accordance with the applicable license terms. By expressly
* accepting such terms or by downloading, installing, activating and/or otherwise
* using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms. If you do not agree to be
* bound by the applicable license terms, then you may not retain, install,
* activate or otherwise use the software.
==================================================================================================*/

/**
*   @file main.c
*
*   @addtogroup main_module main module documentation
*   @{
*/

/* Including necessary configuration files. */
#include "Mcal.h"
#include "Clock_Ip.h"
#include "Siul2_Port_Ip.h"
#include "Siul2_Dio_Ip.h"
#include "AppTaskManager.h"
#include "uds.h"

volatile uint16_t exit_code = 0;
volatile uint16_t exit_code1[7] = {0};
volatile uint16_t exit_code2 = 0;
extern uint8_t pit_test;
/* User includes */
void test_delay(uint32_t count)
{
    volatile uint32_t i = 0;
    for (i = 0; i < count; ++i)
    {
        __asm("NOP"); /* 调用nop空指令 */
         __asm("NOP"); /* 调用nop空指令 */
          __asm("NOP"); /* 调用nop空指令 */
           __asm("NOP"); /* 调用nop空指令 */
            __asm("NOP"); /* 调用nop空指令 */
    }
}
/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
uint16 RxData_r = 0;
uint16 RxData = 0;
uint16 RxData_12 = 0;
uint8 afe_diag;
uint8 StatusA = 0;
uint8 StatusB = 0;
uint8 AlertA = 0;
uint8 AlertB = 0;
uint16 Status = 0;
uint16 Raw_Status = 0;
uint16 Battery = 0;
uint8 Charging = 0;
uint16 Measurement = 0;
uint8 SEPIC_EN = 0;
uint8 Balancing = 0;
uint8 Balancing_cbk = 0;
int main(void)
{
    /* Write your code here */
	/* Initialize Clock */

	Clock_Ip_StatusType Status_Init_Clock = CLOCK_IP_ERROR;
	Status_Init_Clock = Clock_Ip_Init(Clock_Ip_aClockConfig);

	if (Status_Init_Clock != CLOCK_IP_SUCCESS)
	{
		while(1); /* Error during initialization. */
	}

	/* Initialize all pins using the Port driver */
	Siul2_Port_Ip_PortStatusType Status_Init_Port = SIUL2_PORT_ERROR;
	Status_Init_Port = Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals, g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);

	if(Status_Init_Port != SIUL2_PORT_SUCCESS)
	{
		while(1); /* Error during initialization. */
	}
	IP_DCM_GPR->DCMRWF4 |= (1<<12);
	Siul2_Dio_Ip_WritePin(LED_PORT, LED_PIN, 0);
//	Siul2_Dio_Ip_WritePin(SEPIC_EN_PORT, SEPIC_EN_PIN, 1);
	Bq76907_Init();

	Spi_Init();

	//DR7808_init();
	CAN0_Init();
	ADC0_Init();
	PIT0_Init();
	//Sbc_fs23_ReadRegister(SBC_FS23_M_STATUS_ADDR,&RxData_r);
	//TaskInit();
	//vTaskStartScheduler();
	//exit_code=Read_Int_Temperature();
	exit_code1[0]=ReadCellVoltage(1);
	//Sleep_ctrl(0);
	test_delay(2000);
	//Sleep_enable(0);
	//RxData_12 = BQ76907_ReadReg_test(0x12 ,2);
	Data_Init();
    for(;;)
    {
    	APP_Program();
    	Diagnostic_MainProc();
    	/*
    	if(Charging!=0x04)
    	{
    		OnOff_Charging(Charging);
    		Charging=0x04;
    	}
    	if(SEPIC_EN==0x01)
    		Siul2_Dio_Ip_WritePin(SEPIC_EN_PORT, SEPIC_EN_PIN, 1);
    	else if(SEPIC_EN==0x02)
    		Siul2_Dio_Ip_WritePin(SEPIC_EN_PORT, SEPIC_EN_PIN, 0);
    	if(Balancing!=0x08)
    	{
    	Cell_Balancing(Balancing);
    	test_delay(2000);
		Balancing=0x08;
    	}
    	test_delay(2000);
    	Balancing_cbk = Cell_Balancing_cbk();
    	*/
    	test_delay(2000);
    	exit_code=Read_CC1_Current();
    	//OnOff_Charging(Charging);
    	/*
    	test_delay(2000);
    	exit_code=Read_CC2_Current();
    	test_delay(2000);
    	exit_code1[0]=ReadCellVoltage(0);
    	exit_code1[1]=ReadCellVoltage(1);
    	exit_code1[2]=ReadCellVoltage(2);
    	exit_code1[3]=ReadCellVoltage(3);
    	exit_code1[4]=ReadCellVoltage(4);
    	exit_code1[5]=ReadCellVoltage(5);
    	exit_code1[6]=ReadCellVoltage(6);
    	test_delay(2000);
    	exit_code2=Stack_Voltage();
    	test_delay(2000);
    	StatusA=Safety_StatusA();
    	test_delay(2000);
    	StatusB=Safety_StatusB();
    	test_delay(2000);
    	AlertA=Safety_AlertA();
    	test_delay(2000);
    	AlertB=Safety_AlertB();
    	test_delay(2000);
    	Status=Alarm_Status();
    	test_delay(2000);
    	Raw_Status=Alarm_Raw_Status();
    	test_delay(2000);
    	Battery=Battery_Status();
		test_delay(2000);
		*/
    	afe_diag = Siul2_Dio_Ip_ReadPin(AFE_ALERT_PORT, AFE_ALERT_PIN);
    	if(afe_diag)
    	{
//    	Siul2_Dio_Ip_WritePin(LED_PORT, LED_PIN, 1);

    	//Sleep_ctrl(1);
    	//RxData_12 = BQ76907_ReadReg_test(0x12 ,2);
    	}
    	else
    	{
//        	Siul2_Dio_Ip_WritePin(LED_PORT, LED_PIN, 0);
        	//Sleep_enable(0);
    		//Siul2_Dio_Ip_WritePin(LED_PORT, LED_PIN, 1);
    		//Sleep_ctrl(0);
    		//RxData_12 = BQ76907_ReadReg_test(0x12 ,2);
    	}
    	//exit_code=Read_Int_Temperature();
    	//test_delay(2000);




    	//exit_code1=ReadCellVoltage(1);
        //if(exit_code != 0)
        //{
        //    break;
       // }
    }
    return exit_code;
}

/** @} */
