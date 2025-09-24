/*
 * AdcService.c
 *
 *  Created on: 2025Äê8ÔÂ22ÈÕ
 *      Author: mengfa3
 */
#include "Mcal.h"
#include "AdcService.h"
#include "IntCtrl_Ip.h"

#define ADC_SAR_USED_CH                 41U /* Internal Bandgap Channel */

extern void Adc_Sar_0_Isr(void);

uint8_t u8Count = 0;
uint32_t AD1_SumV[2] = {0};
uint16_t ADC_convert_buff[2] = {0};
volatile boolean notif_triggered = FALSE;

void ADC0_Init(void)
{
	StatusType status;
	uint8 Index;
	status = (StatusType) Adc_Sar_Ip_Init(ADCHWUNIT_0_INSTANCE, &AdcHwUnit_0);
	while (status != E_OK);
	/* Install and enable interrupt handlers */
	IntCtrl_Ip_InstallHandler(ADC0_IRQn, Adc_Sar_0_Isr, NULL_PTR);
	IntCtrl_Ip_EnableIrq(ADC0_IRQn);
	for(Index = 0; Index <= 5; Index++)
	{
		status = (StatusType) Adc_Sar_Ip_DoCalibration(ADCHWUNIT_0_INSTANCE);
		if(status == E_OK)
		{
			break;
		}
	}
	Adc_Sar_Ip_EnableNotifications(ADCHWUNIT_0_INSTANCE, ADC_SAR_IP_NOTIF_FLAG_NORMAL_ENDCHAIN);
	/* Start a SW triggered normal conversion on ADC_SAR */
	Adc_Sar_Ip_StartConversion(ADCHWUNIT_0_INSTANCE, ADC_SAR_IP_CONV_CHAIN_NORMAL);
}

void AdcEndOfChainNotif(void)
{
	//	if((Adc_Sar_Ip_GetStatusFlags(0u) & ADC_SAR_IP_NOTIF_FLAG_NORMAL_ENDCHAIN))
	{
		if(u8Count<20)
		{
			AD1_SumV[0] += Adc_Sar_Ip_GetConvData(ADCHWUNIT_0_INSTANCE, ADC_SAR_USED_CH);
			u8Count++;
		}
		else
		{
			ADC_convert_buff[0] = AD1_SumV[0]/20;
			AD1_SumV[0] = 0;
			notif_triggered = TRUE;
			u8Count=0;
		}
		Adc_Sar_Ip_ClearStatusFlags(ADCHWUNIT_0_INSTANCE,ADC_SAR_IP_NOTIF_FLAG_NORMAL_ENDCHAIN);
	}
	/* Checks the measured ADC data conversion */
    // while (ADC_TOLERANCE(data, ADC_BANDGAP));
	/* Start a SW triggered normal conversion on ADC_SAR */
	Adc_Sar_Ip_StartConversion(ADCHWUNIT_0_INSTANCE, ADC_SAR_IP_CONV_CHAIN_NORMAL);
}
