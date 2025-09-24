/*
 * AppTaskManager.h
 *
 *  Created on: 2025Äê8ÔÂ21ÈÕ
 *      Author: mengfa3
 */

#ifndef APPTASKMANAGER_H_
#define APPTASKMANAGER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "CanService.h"
#include "AdcService.h"
#include "pit_timer.h"
#include "BQ76907.h"
#include "DR7808.h"

void Data_Init(void);
void APP_Program(void);
void TaskInit(void);

#endif /* APPTASKMANAGER_H_ */
