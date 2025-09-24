/*
 * pit_timer.c
 *
 *  Created on: 2025年8月26日
 *      Author: mengfa3
 */
#include "pit_timer.h"
#include "Pit_Ip.h"
#include "IntCtrl_Ip.h"
#include "uds.h"

/* PIT instance used - 0 */
#define PIT_INST_0 0U
/* PIT Channel used - 0 */
#define CH_0 0U
/* PIT time-out period - equivalent to 1s */
#define PIT_PERIOD 40000

uint16_t Balance_Count = 0;
uint16_t Drive_Count = 0;//驱动时间
uint8_t CRASH_Count = 0;
uint16_t Sleep_Count = 0;
uint16_t cnt_500ms = 0;
uint16_t cnt_1000ms = 0;
uint8_t cnt_50ms = 0;
uint8_t pit_test = 0;
void PIT0_Init(void)//定时器0初始化
{
	/* set PIT 0 interrupt */
	IntCtrl_Ip_Init(&IntCtrlConfig_0);
	IntCtrl_Ip_EnableIrq(PIT0_IRQn);

	/* Initialize PIT instance 0 - Channel 0 */
	Pit_Ip_Init(PIT_INST_0, &PIT_0_InitConfig_PB);
	/* Initialize channel 0 */
	Pit_Ip_InitChannel(PIT_INST_0, PIT_0_CH_0);
	/* Enable channel interrupt PIT_0 - CH_0 */
	Pit_Ip_EnableChannelInterrupt(PIT_INST_0, CH_0);
	/* Start channel CH_0 */
	Pit_Ip_StartChannel(PIT_INST_0, CH_0, PIT_PERIOD);
}

void PIT0_IRQn_ISR(void)
{
	if(Balance_Count<25000)
	{
		Balance_Count++;
	}
	if(Drive_Count<20000)
	{
		Drive_Count++;
	}
	if(CRASH_Count<100)
	{
		CRASH_Count++;
	}
	if(cnt_500ms<1000)
	{
		cnt_500ms++;
	}
	if(cnt_1000ms<1500)
	{
		cnt_1000ms++;
	}
	if(cnt_50ms<100)
	{
		cnt_50ms++;
	}
	if(Sleep_Count<5)
	{
		Sleep_Count++;
	}
	else
	{

		if (pit_test == 0) {
			Sleep_Count = 0;
		    pit_test = 1;
		} else {
		    //pit_test = 0;
		}
	}
	PitRtiNotification();
}
