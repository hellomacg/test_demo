/*
 * AdcService.h
 *
 *  Created on: 2025Äê8ÔÂ22ÈÕ
 *      Author: mengfa3
 */

#ifndef SERVICE_ADCSERVICE_H_
#define SERVICE_ADCSERVICE_H_

#include "Adc_Sar_Ip.h"

extern uint16_t ADC_convert_buff[2];
extern volatile boolean notif_triggered;

void ADC0_Init(void);
void AdcEndOfChainNotif(void);

#endif /* SERVICE_ADCSERVICE_H_ */
