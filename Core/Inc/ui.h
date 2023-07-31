/*
 * ui.h
 *
 *  Created on: Jul 28, 2023
 *      Author: Zain Irsyad
 */

#ifndef INC_UI_H_
#define INC_UI_H_

#include "main.h"

#define SW_UP_PRESS	0x01
#define SW_RG_PRESS	0x02
#define SW_DW_PRESS	0x04
#define SW_BK_PRESS	0x08
#define SW_LF_PRESS	0x10
#define SW_OK_PRESS	0x20
#define SW_HM_PRESS	0x40

#define SW_DELAY 	70
#define SW_DELAY_FAST 	25

typedef enum {
	UI_IDLE,
	UI_STARTUP,
	UI_HOMESCREEN,
	UI_SPEED_SET,
	UI_RUN,
	UI_PLAN_SET,
	UI_PLAN_SELECT,
	UI_SET,
	UI_SET_ADC,
	UI_SET_CALIB,
	UI_SET_CP,
	UI_SET_PID,
	UI_SET_MIRR,
	UI_SET_MEM,
	UI_SET_HW,
	UI_SET_WIM
} UI_Menu_Lists;

void UI_Init();
void UI_Main();
uint8_t UI_ShowSensor();

#endif /* INC_UI_H_ */
