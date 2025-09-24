/*
 * pit_timer.h
 *
 *  Created on: 2025Äê8ÔÂ26ÈÕ
 *      Author: mengfa3
 */

#ifndef SERVICE_PIT_TIMER_H_
#define SERVICE_PIT_TIMER_H_

#include "IntCtrl_Ip.h"

extern uint16_t Balance_Count;
extern uint16_t Drive_Count;
extern uint8_t CRASH_Count;
extern uint16_t cnt_500ms;
extern uint16_t cnt_1000ms;
extern uint8_t cnt_50ms;
extern uint16_t Sleep_Count;

void PIT0_Init(void);

#endif /* SERVICE_PIT_TIMER_H_ */
