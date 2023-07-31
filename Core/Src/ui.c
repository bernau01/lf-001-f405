/*
 * ui.c
 *
 *  Created on: Jul 28, 2023
 *      Author: Zain Irsyad
 */

#include "ui.h"
#include "display.h"
#include "sensor.h"
#include "storage.h"
#include "stdio.h"
#include "run.h"
#include "plan.h"

uint8_t sen_xy[2] = {16, 16};

#define SENSOR_NUMBER SENSOR_NUM
#define SENSOR_ORIGIN_X sen_xy[0]
#define SENSOR_ORIGIN_Y sen_xy[1]
#define SENSOR_WIDTH_SEN 6
#define SENSOR_HEIGHT_PX 15

uint8_t UI_ShowSensor() {
	if((sensor_flag & 1) != 1) return 0;
	uint8_t* value = sensor_disp_status;
	DISP_FillRect(
			SENSOR_ORIGIN_X,
			SENSOR_ORIGIN_Y,
			SENSOR_ORIGIN_X+(SENSOR_WIDTH_SEN+1)*14,
			SENSOR_ORIGIN_Y+SENSOR_HEIGHT_PX,
			DISP_COLOR_BLACK);
	DISP_Shape_Typedef rect = {0,0,DISP_COLOR_WHITE,SENSOR_WIDTH_SEN-1,1};
	for(int s=0; s<SENSOR_NUMBER; s++) {
		uint8_t status = value[SENSOR_NUMBER-s-1];
		if(status>=SENSOR_HEIGHT_PX) status=SENSOR_HEIGHT_PX-1;
		else if(status==1 || status==2) status=3;
		uint8_t y_rect = status;

		rect.origin_x = (s*(SENSOR_WIDTH_SEN+1))+SENSOR_ORIGIN_X;
		rect.origin_y = SENSOR_ORIGIN_Y+SENSOR_HEIGHT_PX-y_rect;
		rect.size_y = y_rect;
		DISP_FillRect2(&rect);
		DISP_OLEDUpdate(
				rect.origin_x,
				SENSOR_ORIGIN_Y,
				rect.origin_x + SENSOR_WIDTH_SEN,
				SENSOR_ORIGIN_Y+SENSOR_HEIGHT_PX);
	}
	sensor_flag &=~1;
	return 1;
}

uint8_t UI_ReadButton() {
	uint8_t temp = 0;
	temp |= !HAL_GPIO_ReadPin(SW_UP_GPIO_Port, SW_UP_Pin) * SW_UP_PRESS;
	temp |= !HAL_GPIO_ReadPin(SW_RG_GPIO_Port, SW_RG_Pin) * SW_RG_PRESS;
	temp |= !HAL_GPIO_ReadPin(SW_DW_GPIO_Port, SW_DW_Pin) * SW_DW_PRESS;
	temp |= !HAL_GPIO_ReadPin(SW_BK_GPIO_Port, SW_BK_Pin) * SW_BK_PRESS;
	temp |= !HAL_GPIO_ReadPin(SW_LF_GPIO_Port, SW_LF_Pin) * SW_LF_PRESS;
	temp |= !HAL_GPIO_ReadPin(SW_OK_GPIO_Port, SW_OK_Pin) * SW_OK_PRESS;
	temp |= !HAL_GPIO_ReadPin(SW_HM_GPIO_Port, SW_HM_Pin) * SW_HM_PRESS;
	return temp;
}

DISP_Text_Typedef short_message;

DISP_Text_Typedef home_menu[4];
DISP_Text_Typedef home_status;

char* home_screen_menus[4] = {
		"RUN",
		"PATH",
		"PLAN",
		"SET"
};

void UI_HomeScreenInit() {

	short_message.origin_x = 0;
	short_message.origin_y = 27;
	short_message.color = DISP_COLOR_BLACK;
	short_message.size_x = 127;
	short_message.size_y = 10;
	short_message.align_x = DISP_ALIGN_CENTER;
	short_message.align_y = DISP_ALIGN_CENTER;

	home_menu[0].origin_x = 14;
	home_menu[0].origin_y = 38;
	home_menu[0].color = DISP_COLOR_WHITE;
	home_menu[0].size_x = 40;
	home_menu[0].align_x = DISP_ALIGN_CENTER;

	home_menu[1].origin_x = 14;
	home_menu[1].origin_y = 51;
	home_menu[1].color = DISP_COLOR_WHITE;
	home_menu[1].size_x = 40;
	home_menu[1].align_x = DISP_ALIGN_CENTER;

	home_menu[2].origin_x = 73;
	home_menu[2].origin_y = 38;
	home_menu[2].color = DISP_COLOR_WHITE;
	home_menu[2].size_x = 40;
	home_menu[2].align_x = DISP_ALIGN_CENTER;

	home_menu[3].origin_x = 73;
	home_menu[3].origin_y = 51;
	home_menu[3].color = DISP_COLOR_WHITE;
	home_menu[3].size_x = 40;
	home_menu[3].align_x = DISP_ALIGN_CENTER;

	home_status.origin_x = 0;
	home_status.origin_y = 0;
	home_status.size_x = 127;
	home_status.size_y = 10;
	home_status.color = DISP_COLOR_BLACK;
	home_status.align_x = DISP_ALIGN_CENTER;
	home_status.align_y = DISP_ALIGN_CENTER;
}


void UI_HomeScreenDisplay(uint8_t num_menu, uint8_t cp) {
	char buff[22];
	sprintf(buff, "i:%03i  cp:%02i  Plan%i", plan.checkpoint[cp], cp, num_plan);
	DISP_DisplayText(&home_status, buff);
	for(int i=0; i<4; i++) {
		if(i == num_menu) home_menu[i].color = DISP_COLOR_BLACK;
		else home_menu[i].color = DISP_COLOR_WHITE;
		DISP_DisplayText(&home_menu[i], home_screen_menus[i]);
	}
}

UI_Menu_Lists UI_HomeScreen() {
	static uint8_t num_menu = 0;
	static uint32_t tick = 0;
	uint8_t checkpoint = 0;
	ui_home_screen:
	UI_HomeScreenDisplay(num_menu, checkpoint);
	tick = HAL_GetTick();
	while(1) {
		UI_ShowSensor();

		static uint8_t bt = 0;

		if(HAL_GetTick() - tick < SW_DELAY) continue;
		if(bt == UI_ReadButton()) continue;
		bt = UI_ReadButton();
//		if(bt & (SW_RG_PRESS | SW_LF_PRESS)) {
//			num_menu = (num_menu+2)%4;
//			goto ui_home_screen;
//		}
		if(bt & SW_UP_PRESS) {
			if(num_menu > 0) num_menu--; else num_menu=3;
			goto ui_home_screen;
		}
		else if(bt & SW_DW_PRESS) {
			if(num_menu < 3) num_menu++; else return UI_SPEED_SET;
			goto ui_home_screen;
		}
		else if(bt & SW_RG_PRESS) {
			if(checkpoint < MAX_CHECKPOINT-1) checkpoint++; else checkpoint=0;
			goto ui_home_screen;
		}
		else if(bt & SW_LF_PRESS) {
			if(checkpoint > 0) checkpoint--; else checkpoint=MAX_CHECKPOINT-1;
			goto ui_home_screen;
		}
		else if(bt & SW_HM_PRESS) {
			num_menu = 0;
			goto ui_home_screen;
		}
		switch(num_menu) {
		case 0:
			if(bt & SW_OK_PRESS) {
				num_checkpoint = checkpoint;
				return UI_RUN;
			}
			break;
		case 1:
			if(bt & SW_OK_PRESS) {
				num_checkpoint = checkpoint;
				return UI_PLAN_SET;
			}
			break;
		case 2:
			if(bt & SW_OK_PRESS) {
				return UI_PLAN_SELECT;
			}
			break;
		case 3:
			if(bt & SW_OK_PRESS) {
				return UI_SET;
			}
			break;
		default: num_menu = 0;
		}
	}
	return UI_IDLE;
}

DISP_Text_Typedef setting_menu[4];
DISP_Text_Typedef setting_head;
char* setting_menus[] = {
		" ADC Threshold",
		" Sensor Calibration",
		" Checkpoint",
		" PID",
		" Mirroring (NOP)",
		" Memory",
		" Hardware (NOP)",
		" Who Am I?"
};

void UI_SettingDisplayInit() {
	setting_menu[0].origin_x = 3;
	setting_menu[0].origin_y = 13;
	setting_menu[0].size_x = 120;
	setting_menu[0].size_y = 11;
	setting_menu[0].color = DISP_COLOR_WHITE;
	setting_menu[0].align_y = DISP_ALIGN_CENTER;
	for(int i=1; i<4; i++) {

		setting_menu[i].origin_x = setting_menu[0].origin_x;
		setting_menu[i].origin_y = setting_menu[i-1].origin_y+setting_menu[0].size_y+2;
		setting_menu[i].size_x = setting_menu[0].size_x;
		setting_menu[i].size_y = setting_menu[0].size_y;
		setting_menu[i].color = setting_menu[0].color;
		setting_menu[i].align_y = setting_menu[0].align_y;
	}

	setting_head.origin_x = 0;
	setting_head.origin_y = 0;
	setting_head.size_x = 127;
	setting_head.size_y = 10;
	setting_head.color = DISP_COLOR_BLACK;
	setting_head.align_x = DISP_ALIGN_CENTER;
	setting_head.align_y = DISP_ALIGN_CENTER;
}

void UI_SettingDisplay(uint8_t num) {
	static int8_t first_list = 0;
	if(num - first_list >= 4) first_list = (num-3);
	else if(num < first_list) first_list = num;

	for(int i=0; i<4; i++) {
		if(i + first_list == num) setting_menu[i].color = DISP_COLOR_BLACK;
		else setting_menu[i].color = DISP_COLOR_WHITE;
		DISP_DisplayText(&setting_menu[i], setting_menus[i+first_list]);
	}
}

UI_Menu_Lists UI_Setting() {
	uint8_t bt;
	static uint8_t num_menu = 0;
	uint32_t tick = 0;
	DISP_DisplayText(&home_status, "SETTING");
	ui_setting:
	UI_SettingDisplay(num_menu);
	tick = HAL_GetTick();
	while(1) {
		if(HAL_GetTick() - tick < SW_DELAY) continue;
		if(bt == UI_ReadButton()) continue;
		bt = UI_ReadButton();

		if(bt & SW_DW_PRESS) {
			if(num_menu==7) num_menu = 0; else num_menu++;
			goto ui_setting;
		}
		else if(bt & SW_UP_PRESS) {
			if(num_menu==0) num_menu = 7; else num_menu--;
			goto ui_setting;
		}
		else if(bt & (SW_HM_PRESS|SW_BK_PRESS)) {
			num_menu = 0;
			return UI_HOMESCREEN;
		}
		switch(num_menu) {
		case 0:
			if(bt & SW_OK_PRESS) {
				return UI_SET_ADC;
			}
			break;
		case 1:
			if(bt & SW_OK_PRESS) {
				return UI_SET_CALIB;
			}
			break;
		case 2:
			if(bt & SW_OK_PRESS) {
				return UI_SET_CP;
			}
			break;
		case 3:
			if(bt & SW_OK_PRESS) {
				return UI_SET_PID;
			}
			break;
		case 5:
			if(bt & SW_OK_PRESS) {
				return UI_SET_MEM;
			}
			break;
		}
	}
	return UI_IDLE;
}



DISP_Text_Typedef planset_list_name[4];
DISP_Text_Typedef planset_list_value[4];
DISP_Text_Typedef planset_head;

char* plan_act_lists[] = {
		"Stop",
		"Left",
		"Right",
		"Forward",
		"Follow",
		"Follow L",
		"Follow R",
		"Invert",
		"Next Plan",
		"Null",
};

char* plan_mode_lists[] = {
		"Time",
		"Sen",
		"Enc"
};

char* plan_act_value_lists[] = {
		" Delay",
		" Delay",
		" Enc Val"
};

#define ACT_LIST	10

void UI_PlanSetDisplayInit() {
	planset_list_name[0].origin_x = 6;
	planset_list_name[0].origin_y = 16;
	planset_list_name[0].size_x = 54;
	planset_list_name[0].size_y = 11;
	planset_list_name[0].color = DISP_COLOR_WHITE;
	planset_list_name[0].align_y = DISP_ALIGN_CENTER;

	planset_list_value[0] = planset_list_name[0];
	planset_list_value[0].origin_x = 62;
	planset_list_value[0].size_x = 58;
	planset_list_value[0].align_x = DISP_ALIGN_CENTER;


	for(int i=1; i<4; i++) {
		planset_list_name[i] = planset_list_name[0];
		planset_list_name[i].origin_y = planset_list_name[i-1].origin_y+planset_list_name[0].size_y+1;

		planset_list_value[i] = planset_list_value[0];
		planset_list_value[i].origin_y = planset_list_value[i-1].origin_y+planset_list_value[0].size_y+1;
	}

	planset_head.origin_x = 0;
	planset_head.origin_y = 0;
	planset_head.size_x = 127;
	planset_head.size_y = 13;
	planset_head.color = DISP_COLOR_BLACK;
	planset_head.align_x = DISP_ALIGN_LEFT;
	planset_head.align_y = DISP_ALIGN_CENTER;
}

void UI_PlanSetDisplay(int8_t num, Action_typedef* plan, uint8_t num_index) {
	char buff[22];
	static int8_t first_list = 0;
	uint8_t temp = (num == -1);
	sprintf(buff, " %03i ~ %s", num_index, plan_act_lists[plan->act]);
	if(temp) planset_head.color = DISP_COLOR_BLACK; else planset_head.color = DISP_COLOR_WHITE;
	DISP_DisplayText(&planset_head, buff);
	DISP_DrawLineSimple(0, 14, 127, 14, DISP_COLOR_WHITE);
	DISP_OLEDUpdate(0, 13, 127, 13);
	if(num==-1) num = 0;
	if(num - first_list >= 4) first_list = (num-3);
	else if(num < first_list) first_list = num;

	for(int i=0; i<4; i++) {
		if(i + first_list == num && !temp) {
			planset_list_name[i].color = DISP_COLOR_BLACK;
			planset_list_value[i].color = DISP_COLOR_BLACK;
		}
		else {
			planset_list_name[i].color = DISP_COLOR_WHITE;
			planset_list_value[i].color = DISP_COLOR_WHITE;
		}
		switch(first_list + i) {
		case 0:
			DISP_DisplayText(&planset_list_name[i], " Sensor");
			sprintf(buff, "%i", plan->sen_trig);
			DISP_DisplayText(&planset_list_value[i], buff);
			break;
		case 1:
			DISP_DisplayText(&planset_list_name[i], " Mode");
			DISP_DisplayText(&planset_list_value[i], plan_mode_lists[plan->act_mode]);
			break;
		case 2:
			DISP_DisplayText(&planset_list_name[i], plan_act_value_lists[plan->act_mode]);
			sprintf(buff, "%i", plan->act_value);
			DISP_DisplayText(&planset_list_value[i], buff);
			break;
		case 3:
			DISP_DisplayText(&planset_list_name[i], " Brake");
			sprintf(buff, "%i", plan->brake);
			DISP_DisplayText(&planset_list_value[i], buff);
			break;
		case 4:
			DISP_DisplayText(&planset_list_name[i], " Fw Sp");
			sprintf(buff, "%i", plan->forward_speed);
			DISP_DisplayText(&planset_list_value[i], buff);
			break;
		case 5:
			DISP_DisplayText(&planset_list_name[i], " Rv Sp");
			sprintf(buff, "%i", plan->reverse_speed);
			DISP_DisplayText(&planset_list_value[i], buff);
			break;
		case 6:
			DISP_DisplayText(&planset_list_name[i], " Wait");
			sprintf(buff, "%i", plan->wait_time);
			DISP_DisplayText(&planset_list_value[i], buff);
			break;
		case 7:
			DISP_DisplayText(&planset_list_name[i], " Bst Tim");
			sprintf(buff, "%i", plan->boost_time);
			DISP_DisplayText(&planset_list_value[i], buff);
			break;
		case 8:
			DISP_DisplayText(&planset_list_name[i], " Bst Sp");
			sprintf(buff, "%i", plan->boost_speed);
			DISP_DisplayText(&planset_list_value[i], buff);
			break;
		case 9:
			DISP_DisplayText(&planset_list_name[i], " Insert");
			DISP_DisplayText(&planset_list_value[i], " ");
			break;
		case 10:
			DISP_DisplayText(&planset_list_name[i], " Delete");
			DISP_DisplayText(&planset_list_value[i], " ");
			break;
		}
	}
}

UI_Menu_Lists UI_PlanSet() {
	uint8_t num_index = 0;
	Action_typedef plan_set = GetAction(num_index);
	int8_t num = -1;
	uint32_t tick;
	uint8_t bt, bt2;
	uint8_t bt_status = 0;
	plan_set:
	UI_PlanSetDisplay(num, &plan_set, num_index);
	tick = HAL_GetTick();
	while(1) {
		if(HAL_GetTick() - tick < ((bt_status == 1)?SW_DELAY_FAST:SW_DELAY)) continue;

		bt2 = UI_ReadButton();

		if(bt == bt2 && bt_status == 0) continue;
		bt = bt2;
//
//		if(bt & SW_DW_PRESS) {
//			if(num==8) num = 0; else num++;
//			goto ui_setting;
//		}
//		else if(bt & SW_UP_PRESS) {
//			if(num==0) num = 8; else num--;
//			goto ui_setting;
//		}
		if(bt & SW_HM_PRESS) {
			num = -1;
			return UI_HOMESCREEN;
		}
		switch(num) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
			if(bt & SW_UP_PRESS) {
				if(num != 0) num--; else num=10;
				bt_status = 0;
				goto plan_set;
			}
			else if(bt & SW_DW_PRESS) {
				if(num != 10) num++; else num=0;
				bt_status = 0;
				goto plan_set;
			}
			else if(bt & SW_BK_PRESS) {
				bt_status = 0;
				num = -1;
				goto plan_set;
			}
		}
		switch(num) {
		case -1:
			if(bt & SW_DW_PRESS) {
				if(plan_set.act != 0) plan_set.act--; else plan_set.act=ACT_LIST-1;
				bt_status = 0;
				goto plan_set;
			}
			else if(bt & SW_UP_PRESS) {
				if(plan_set.act != ACT_LIST-1) plan_set.act++; else plan_set.act=0;
				bt_status = 0;
				goto plan_set;
			}
			else if(bt & SW_OK_PRESS) {
				num = 0;
				bt_status = 0;
				goto plan_set;
			}
			else if(bt & SW_BK_PRESS) {
				SetAction(plan_set, num_index);
				num = -1;
				bt_status = 0;
				return UI_HOMESCREEN;
			}
			else if(bt2 & SW_RG_PRESS) {
				SetAction(plan_set, num_index);
				if(num_index != max_index) num_index++; else num_index=0;
				plan_set = GetAction(num_index);
				bt_status = 1;
				goto plan_set;
			}
			else if(bt2 & SW_LF_PRESS) {
				SetAction(plan_set, num_index);
				if(num_index != 0) num_index--; else num_index=max_index;
				plan_set = GetAction(num_index);
				bt_status = 1;
				goto plan_set;
			}
			break;
		case 0:
			if(bt & SW_LF_PRESS) {
				if(plan_set.sen_trig != 0) plan_set.sen_trig--; else plan_set.sen_trig=SENSOR_NUM;
				bt_status = 0;
				goto plan_set;
			}
			if(bt & SW_RG_PRESS) {
				if(plan_set.sen_trig != SENSOR_NUM) plan_set.sen_trig++; else plan_set.sen_trig=0;
				bt_status = 0;
				goto plan_set;
			}
			break;
		case 1:
			if(bt & SW_LF_PRESS) {
				if(plan_set.act_mode != 0) plan_set.act_mode--; else plan_set.act_mode=2;
				bt_status = 0;
				goto plan_set;
			}
			if(bt & SW_RG_PRESS) {
				if(plan_set.act_mode != 2) plan_set.act_mode++; else plan_set.act_mode=0;
				bt_status = 0;
				goto plan_set;
			}
			break;
		case 2:
			if(bt & SW_LF_PRESS) {
				if(plan_set.act_value != 0) plan_set.act_value--; else plan_set.act_value=255;
				bt_status = 1;
				goto plan_set;
			}
			if(bt & SW_RG_PRESS) {
				if(plan_set.act_value != 255) plan_set.act_value++; else plan_set.act_value=0;
				bt_status = 1;
				goto plan_set;
			}
			break;
		case 3:
			if(bt & SW_LF_PRESS) {
				if(plan_set.brake != 0) plan_set.brake--; else plan_set.brake=255;
				bt_status = 1;
				goto plan_set;
			}
			if(bt & SW_RG_PRESS) {
				if(plan_set.brake != 255) plan_set.brake++; else plan_set.brake=0;
				bt_status = 1;
				goto plan_set;
			}
			break;
		case 4:
			if(bt & SW_LF_PRESS) {
				if(plan_set.forward_speed != -99) plan_set.forward_speed--; else plan_set.forward_speed=99;
				bt_status = 1;
				goto plan_set;
			}
			if(bt & SW_RG_PRESS) {
				if(plan_set.forward_speed != 99) plan_set.forward_speed++; else plan_set.forward_speed=-99;
				bt_status = 1;
				goto plan_set;
			}
			break;
		case 5:
			if(bt & SW_LF_PRESS) {
				if(plan_set.reverse_speed != -99) plan_set.reverse_speed--; else plan_set.reverse_speed=99;
				bt_status = 1;
				goto plan_set;
			}
			if(bt & SW_RG_PRESS) {
				if(plan_set.reverse_speed != 99) plan_set.reverse_speed++; else plan_set.reverse_speed=-99;
				bt_status = 1;
				goto plan_set;
			}
			break;
		case 6:
			if(bt & SW_LF_PRESS) {
				if(plan_set.wait_time != 0) plan_set.wait_time--; else plan_set.wait_time=255;
				bt_status = 1;
				goto plan_set;
			}
			if(bt & SW_RG_PRESS) {
				if(plan_set.wait_time != 255) plan_set.wait_time++; else plan_set.wait_time=0;
				bt_status = 1;
				goto plan_set;
			}
			break;
		case 7:
			if(bt & SW_LF_PRESS) {
				if(plan_set.boost_time != 0) plan_set.boost_time--; else plan_set.boost_time=255;
				bt_status = 1;
				goto plan_set;
			}
			if(bt & SW_RG_PRESS) {
				if(plan_set.boost_time != 255) plan_set.boost_time++; else plan_set.boost_time=0;
				bt_status = 1;
				goto plan_set;
			}
			break;
		case 8:
			if(bt & SW_LF_PRESS) {
				if(plan_set.boost_speed != 0) plan_set.boost_speed--; else plan_set.boost_speed=255;
				bt_status = 1;
				goto plan_set;
			}
			if(bt & SW_RG_PRESS) {
				if(plan_set.boost_speed != 255) plan_set.boost_speed++; else plan_set.boost_speed=0;
				bt_status = 1;
				goto plan_set;
			}
			break;
		}
	}
	return UI_IDLE;
}




DISP_Text_Typedef adc_menu[2];
DISP_Text_Typedef adc_head;
DISP_Text_Typedef adc_point;

void UI_ADCThresholdDisplayInit() {
	adc_menu[0].origin_x = 4;
	adc_menu[0].origin_y = 50;
	adc_menu[0].size_x = 58;
	adc_menu[0].size_y = 11;
	adc_menu[0].color = DISP_COLOR_BLACK;
	adc_menu[0].align_y = DISP_ALIGN_CENTER;
	adc_menu[0].align_x = DISP_ALIGN_CENTER;

	adc_menu[1] = adc_menu[0];
	adc_menu[1].origin_x = 63;
	adc_menu[1].size_x = 60;
	adc_menu[1].color = DISP_COLOR_WHITE;

	adc_head.origin_x = 0;
	adc_head.origin_y = 0;
	adc_head.size_x = 127;
	adc_head.size_y = 10;
	adc_head.color = DISP_COLOR_BLACK;
	adc_head.align_x = DISP_ALIGN_CENTER;
	adc_head.align_y = DISP_ALIGN_CENTER;

	adc_point.origin_y = 32;
	adc_point.size_x = SENSOR_WIDTH_SEN;
	adc_point.align_x = DISP_ALIGN_CENTER;

}

void UI_ADCThresholdDisplay(uint8_t num) {
	static uint8_t last_num = 1;
	char buff[22];
	if(last_num != num) {
		sprintf(buff, "ADC ~ %i", num+1);
		DISP_DisplayText(&adc_head, buff);

		uint8_t point_x = (last_num*(SENSOR_WIDTH_SEN+1))+SENSOR_ORIGIN_X-1;
		char point[] = " ";
		adc_point.origin_x = point_x;
		DISP_DisplayText(&adc_point, point);

		point[0] = '~'+2;
		adc_point.origin_x = (num*(SENSOR_WIDTH_SEN+1))+SENSOR_ORIGIN_X-1;
		DISP_DisplayText(&adc_point, point);

		last_num = num;
	}
	sprintf(buff, "Tr %4i", threshold_sensor[SENSOR_NUM-num-1]);
	DISP_DisplayText(&adc_menu[0], buff);

}

UI_Menu_Lists UI_ADCThreshold() {
	uint32_t tick;
	uint8_t bt, bt2;
	uint8_t bt_status = 0;
	uint8_t num_sen = 0;
	uint8_t actual_num_sen = 0;
	char buff[22];
	Storage_GetADCThreshold();
	UI_ADCThresholdDisplay(num_sen+1);
	adctreshold:
	UI_ADCThresholdDisplay(num_sen);
	tick = HAL_GetTick();
	while(1) {
		actual_num_sen = SENSOR_NUM-num_sen-1;
		if(UI_ShowSensor()) {
			sprintf(buff, "ADC %4i", adc_sensor_arranged[actual_num_sen]);
			DISP_DisplayText(&adc_menu[1], buff);
		}

		if(HAL_GetTick() - tick < ((bt_status == 1)?20:SW_DELAY)) continue;

		bt2 = UI_ReadButton();

		if(bt == bt2 && bt_status == 0) continue;
		bt = bt2;

		if(bt & SW_HM_PRESS) {
			num_sen = 0;
			return UI_HOMESCREEN;
		}
		else if(bt & (SW_BK_PRESS|SW_OK_PRESS)) {
			Storage_SetADCThreshold();
			num_sen = 0;
			return UI_SET;
		}
		else if(bt & SW_LF_PRESS) {
			if(num_sen != 0) num_sen--; else num_sen=SENSOR_NUM-1;
			bt_status = 0;
			goto adctreshold;
		}
		else if(bt & SW_RG_PRESS) {
			if(num_sen != SENSOR_NUM-1) num_sen++; else num_sen=0;
			bt_status = 0;
			goto adctreshold;
		}
		else if(bt & SW_DW_PRESS) {
			if(threshold_sensor[actual_num_sen] != 0) threshold_sensor[actual_num_sen]--; else threshold_sensor[actual_num_sen]=ADC_MAX;
			bt_status = 1;
			goto adctreshold;
		}
		else if(bt & SW_UP_PRESS) {
			if(threshold_sensor[actual_num_sen] != ADC_MAX) threshold_sensor[actual_num_sen]++; else threshold_sensor[actual_num_sen]=0;
			bt_status = 1;
			goto adctreshold;
		}
	}
}



UI_Menu_Lists UI_AutoCalib() {
	uint8_t bt = UI_ReadButton();
	SEN_StartAutoCalibration();
	while(1) {
		UI_ShowSensor();
		if(bt == UI_ReadButton()) continue;
		bt = UI_ReadButton();
		if(bt & SW_HM_PRESS) {
			SEN_EndAutoCalibration(0);
			return UI_HOMESCREEN;
		}
		else if(bt & SW_BK_PRESS) {
			SEN_EndAutoCalibration(0);
			return UI_SET;
		}
		else if(bt & SW_OK_PRESS) {
			SEN_EndAutoCalibration(1);
			DISP_DisplayText(&short_message, "Saving...");
			Storage_SetADCThreshold();
			HAL_Delay(200);
			return UI_SET;
		}
	}
}


DISP_Text_Typedef startup;

UI_Menu_Lists UI_Startup() {
	char buff[22];
	startup.size_y = 9;
	sprintf(buff, "[%5i]Check memory", (uint16_t)HAL_GetTick());
	DISP_DisplayText(&startup, buff);
	startup.origin_y = startup.origin_y+startup.size_y;
	HAL_Delay(1);
	if(!Storage_Init()) {
		sprintf(buff, "[%5i]Mem initialize", (uint16_t)HAL_GetTick());
		DISP_DisplayText(&startup, buff);
		startup.origin_y = startup.origin_y+startup.size_y;
		do {
			Storage_ResetMemory();
		} while(!Storage_Init());
	}
	sprintf(buff, "[%5i]Memory OK", (uint16_t)HAL_GetTick());
	DISP_DisplayText(&startup, buff);
	startup.origin_y = startup.origin_y+startup.size_y;

	sprintf(buff, "[%5i]Checking plan", (uint16_t)HAL_GetTick());
	DISP_DisplayText(&startup, buff);
	startup.origin_y = startup.origin_y+startup.size_y;
	HAL_Delay(1);
	if(!Storage_PlanInit()) {
		sprintf(buff, "[%5i]Plan initializ", (uint16_t)HAL_GetTick());
		DISP_DisplayText(&startup, buff);
		startup.origin_y = startup.origin_y+startup.size_y;

		do {
			Storage_ResetPlan();
		} while(!Storage_PlanInit());
	}
	Storage_GetPlan(num_plan);
	sprintf(buff, "[%5i]Plan OK", (uint16_t)HAL_GetTick());
	DISP_DisplayText(&startup, buff);
	startup.origin_y = startup.origin_y+startup.size_y;

	while(UI_ReadButton() & SW_HM_PRESS);

	return UI_HOMESCREEN;
}



DISP_Text_Typedef cp_menu[4];
DISP_Text_Typedef cp_head;

void UI_CPDisplayInit() {
	cp_menu[0].origin_x = 3;
	cp_menu[0].origin_y = 13;
	cp_menu[0].size_x = 120;
	cp_menu[0].size_y = 11;
	cp_menu[0].color = DISP_COLOR_WHITE;
	cp_menu[0].align_x = DISP_ALIGN_CENTER;
	cp_menu[0].align_y = DISP_ALIGN_CENTER;
	for(int i=1; i<4; i++) {
		cp_menu[i] = cp_menu[0];
		cp_menu[i].origin_y = cp_menu[i-1].origin_y+cp_menu[0].size_y+2;
	}

	cp_head.origin_x = 0;
	cp_head.origin_y = 0;
	cp_head.size_x = 127;
	cp_head.size_y = 10;
	cp_head.color = DISP_COLOR_BLACK;
	cp_head.align_x = DISP_ALIGN_CENTER;
	cp_head.align_y = DISP_ALIGN_CENTER;
}

void UI_CPDisplay(uint8_t num, uint8_t* num_value) {
	static int8_t first_list = 0;
	if(num - first_list >= 4) first_list = (num-3);
	else if(num < first_list) first_list = num;

	for(int i=0; i<4; i++) {
		if(i + first_list == num) cp_menu[i].color = DISP_COLOR_BLACK;
		else cp_menu[i].color = DISP_COLOR_WHITE;
		char buff[22];
		sprintf(buff, "CP%-2i  ~  %03i", i + first_list, num_value[i + first_list]);
		DISP_DisplayText(&cp_menu[i], buff);
	}
}

UI_Menu_Lists UI_CP() {
	uint8_t bt;
	uint8_t num_menu = 0;
	uint32_t tick = 0;
	DISP_DisplayText(&cp_head, "CHECKPOINT");
	ui_cp:
	UI_CPDisplay(num_menu, plan.checkpoint);
	tick = HAL_GetTick();
	while(1) {
		if(HAL_GetTick() - tick < SW_DELAY) continue;
		if(bt == UI_ReadButton()) continue;
		bt = UI_ReadButton();

		if(bt & SW_DW_PRESS) {
			if(num_menu==MAX_CHECKPOINT-1) num_menu = 0; else num_menu++;
			goto ui_cp;
		}
		else if(bt & SW_UP_PRESS) {
			if(num_menu==0) num_menu = MAX_CHECKPOINT-1; else num_menu--;
			goto ui_cp;
		}
		else if(bt & SW_RG_PRESS) {
			if(plan.checkpoint[num_menu]==DEF_ACTION_PER_PLAN-1) plan.checkpoint[num_menu] = 0; else plan.checkpoint[num_menu]++;
			goto ui_cp;
		}
		else if(bt & SW_LF_PRESS) {
			if(plan.checkpoint[num_menu]==0) plan.checkpoint[num_menu] = DEF_ACTION_PER_PLAN-1; else plan.checkpoint[num_menu]--;
			goto ui_cp;
		}
		else if(bt & SW_HM_PRESS) {
			return UI_HOMESCREEN;
		}
		else if(bt & (SW_BK_PRESS|SW_OK_PRESS)) {
			DISP_DisplayText(&short_message, "Saving...");
			Storage_SetCheckpoint();
			HAL_Delay(200);
			return UI_SET;
		}
	}
	return UI_IDLE;
}




void UI_PlanSelectDisplay(uint8_t num) {
	static int8_t first_list = 0;
	if(num - first_list >= 4) first_list = (num-3);
	else if(num < first_list) first_list = num;

	for(int i=0; i<4; i++) {
		if(i + first_list == num) cp_menu[i].color = DISP_COLOR_BLACK;
		else cp_menu[i].color = DISP_COLOR_WHITE;
		char buff[22];
		sprintf(buff, "Plan%i", i + first_list);
		DISP_DisplayText(&cp_menu[i], buff);
	}
}

UI_Menu_Lists UI_PlanSelect() {
	uint8_t bt;
	uint8_t num_menu = 0;
	uint32_t tick = 0;
	DISP_DisplayText(&cp_head, "SELECT PLAN");
	ui_cp:
	UI_PlanSelectDisplay(num_menu);
	tick = HAL_GetTick();
	while(1) {
		if(HAL_GetTick() - tick < SW_DELAY) continue;
		if(bt == UI_ReadButton()) continue;
		bt = UI_ReadButton();

		if(bt & SW_DW_PRESS) {
			if(num_menu==MAX_PLAN-1) num_menu = 0; else num_menu++;
			goto ui_cp;
		}
		else if(bt & SW_UP_PRESS) {
			if(num_menu==0) num_menu = MAX_PLAN-1; else num_menu--;
			goto ui_cp;
		}
		else if(bt & (SW_HM_PRESS|SW_BK_PRESS)) {
			return UI_HOMESCREEN;
		}
		else if(bt & SW_OK_PRESS) {
			DISP_DisplayText(&short_message, "Loading...");
			if(!Storage_GetPlan(num_menu)) {
				DISP_DisplayText(&short_message, "Reseting...");
				do {
					Storage_ResetPlan();
				} while(!Storage_PlanInit());
			}
			DISP_DisplayText(&short_message, "Done!");
			HAL_Delay(200);
			return UI_HOMESCREEN;
		}
	}
	return UI_IDLE;
}




char* memory_menus[] = {
		" Reset Plan",
		" Reset All",
		" ",
		" "
};

void UI_MemSettingDisplay(uint8_t num) {
	static int8_t first_list = 0;
	if(num - first_list >= 4) first_list = (num-3);
	else if(num < first_list) first_list = num;

	for(int i=0; i<4; i++) {
		if(i + first_list == num) setting_menu[i].color = DISP_COLOR_BLACK;
		else setting_menu[i].color = DISP_COLOR_WHITE;
		DISP_DisplayText(&setting_menu[i], memory_menus[i+first_list]);
	}
}

UI_Menu_Lists UI_MemSetting() {
	uint8_t bt;
	static uint8_t num_menu = 0;
	uint32_t tick = 0;
	DISP_DisplayText(&home_status, "MEMORY");
	ui_mem:
	UI_MemSettingDisplay(num_menu);
	tick = HAL_GetTick();
	while(1) {
		if(HAL_GetTick() - tick < SW_DELAY) continue;
		if(bt == UI_ReadButton()) continue;
		bt = UI_ReadButton();

		if(bt & SW_DW_PRESS) {
			if(num_menu==1) num_menu = 0; else num_menu++;
			goto ui_mem;
		}
		else if(bt & SW_UP_PRESS) {
			if(num_menu==0) num_menu = 1; else num_menu--;
			goto ui_mem;
		}
		else if(bt & SW_HM_PRESS) {
			num_menu = 0;
			return UI_HOMESCREEN;
		}
		else if(bt & SW_BK_PRESS) {
			num_menu = 0;
			return UI_SET;
		}
		switch(num_menu) {
		case 0:
			if(bt & SW_OK_PRESS) {
				DISP_DisplayText(&short_message, "Reseting...");
				do {
					Storage_ResetPlan();
				} while(!Storage_PlanInit());
				DISP_DisplayText(&short_message, "Done!");
				HAL_Delay(200);
				goto ui_mem;
			}
			break;
		case 1:
			if(bt & SW_OK_PRESS) {
				num_menu = 3;
			}
			break;
		case 3:
			DISP_DisplayText(&setting_menu[3], "Are you sure?");
			while(1) {
				bt = UI_ReadButton();
				if(bt & SW_BK_PRESS) {
					num_menu = 1;
					goto ui_mem;
				}
				if(bt & SW_OK_PRESS) {
					DISP_DisplayText(&setting_menu[3], "Wait...");
					Storage_CleanInit();
					HAL_Delay(1000);
					HAL_NVIC_SystemReset();
				}
			}
		}
	}
	return UI_IDLE;
}



void UI_PIDSettingDisplay(uint8_t num) {
	static int8_t first_list = 0;
	if(num - first_list >= 4) first_list = (num-3);
	else if(num < first_list) first_list = num;

	for(int i=0; i<4; i++) {
		if(i + first_list == num) cp_menu[i].color = DISP_COLOR_BLACK;
		else cp_menu[i].color = DISP_COLOR_WHITE;
		char buff[22];
		switch(i + first_list) {
		case 0:
			sprintf(buff, "KP ~ %3.2f", plan.kpid[0]);
			break;
		case 1:
			sprintf(buff, "KI ~ %3.2f", plan.kpid[1]);
			break;
		case 2:
			sprintf(buff, "KD ~ %3.2f", plan.kpid[2]);
			break;
		case 3:
			sprintf(buff, "%s", plan.status_pid?"AUTO":"MANUAL");
			break;
		}
		DISP_DisplayText(&cp_menu[i], buff);
	}
}

UI_Menu_Lists UI_PIDSetting() {
	uint8_t bt, bt2;
	uint8_t bt_status = 0;
	static uint8_t num_menu = 0;
	uint32_t tick = 0;
	DISP_DisplayText(&home_status, "PID");
	ui_mem:
	UI_PIDSettingDisplay(num_menu);
	tick = HAL_GetTick();
	while(1) {
		if(HAL_GetTick() - tick < ((bt_status == 1)?25:SW_DELAY)) continue;
		bt2 = UI_ReadButton();
		if(bt == bt2 && bt_status == 0) continue;
		bt = bt2;

		if(bt & SW_DW_PRESS) {
			if(num_menu==3) num_menu = 0; else num_menu++;
			bt_status = 0;
			goto ui_mem;
		}
		else if(bt & SW_UP_PRESS) {
			if(num_menu==0) num_menu = 3; else num_menu--;
			bt_status = 0;
			goto ui_mem;
		}
		else if(bt & SW_HM_PRESS) {
			num_menu = 0;
			return UI_HOMESCREEN;
		}
		else if(bt & SW_BK_PRESS) {
			num_menu = 0;
			DISP_DisplayText(&short_message, "Saving..");
			Storage_SetPID();
			return UI_SET;
		}
		switch(num_menu) {
		case 0:
		case 1:
		case 2:
			if((bt & (SW_LF_PRESS|SW_OK_PRESS)) == SW_LF_PRESS) {
				if(plan.kpid[num_menu]==0) plan.kpid[num_menu] = 99.0; else plan.kpid[num_menu]-=1.0;
				bt_status = 1;
				goto ui_mem;
			}
			else if((bt & (SW_RG_PRESS|SW_OK_PRESS)) == SW_RG_PRESS) {
				if(plan.kpid[num_menu]==99.0) plan.kpid[num_menu] = 0.0; else plan.kpid[num_menu]+=1.0;
				bt_status = 1;
				goto ui_mem;
			}
			else if((bt & (SW_LF_PRESS|SW_OK_PRESS)) == (SW_LF_PRESS|SW_OK_PRESS)) {
				if(plan.kpid[num_menu]==0) plan.kpid[num_menu] = 99.0; else plan.kpid[num_menu]-=0.10;
				bt_status = 1;
				goto ui_mem;
			}
			else if((bt & (SW_RG_PRESS|SW_OK_PRESS)) == (SW_RG_PRESS|SW_OK_PRESS)) {
				if(plan.kpid[num_menu]==99.0) plan.kpid[num_menu] = 0.0; else plan.kpid[num_menu]+=0.1;
				bt_status = 1;
				goto ui_mem;
			}
			break;
		case 3 :
			if(bt & (SW_LF_PRESS|SW_RG_PRESS)) {
				plan.status_pid = !plan.status_pid;
				bt_status = 0;
				goto ui_mem;
			}
		}
	}
	return UI_IDLE;
}



void UI_SpeedSetDisplay(uint8_t num) {
	for(int i=2; i<4; i++) {
		if(i == num) cp_menu[i].color = DISP_COLOR_BLACK;
		else cp_menu[i].color = DISP_COLOR_WHITE;
		char buff[22];
		switch(i) {
		case 2:
			sprintf(buff, "Speed ~ %-3i", plan.speed);
			break;
		case 3:
			sprintf(buff, "Tr Speed ~ %-3i", plan.turn_speed);
			break;
		}
		DISP_DisplayText(&cp_menu[i], buff);
	}
}

UI_Menu_Lists UI_SpeedSetting() {
	uint8_t bt, bt2;
	uint8_t num_menu = 2;
	uint32_t tick = 0;
	uint8_t bt_status = 0;
	DISP_DisplayText(&home_status, "SPEED");
	ui_mem:
	UI_SpeedSetDisplay(num_menu);
	tick = HAL_GetTick();
	while(1) {
		if(HAL_GetTick() - tick < ((bt_status == 1)?30:SW_DELAY)) continue;
		bt2 = UI_ReadButton();
		if(bt == bt2 && bt_status == 0) continue;
		bt = bt2;

		if(bt & SW_DW_PRESS) {
			if(num_menu!=3) num_menu++;
			bt_status = 0;
			Storage_SetSpeed();
			goto ui_mem;
		}
		else if(bt & SW_UP_PRESS) {
			if(num_menu==2) {
				Storage_SetSpeed();
				return UI_HOMESCREEN;
			}
			else num_menu--;
			bt_status = 0;
			Storage_SetSpeed();
			goto ui_mem;
		}
		else if(bt & (SW_HM_PRESS|SW_BK_PRESS)) {
			num_menu = 0;
			Storage_SetSpeed();
			return UI_HOMESCREEN;
		}
		switch(num_menu) {
		case 2 :
			if(bt & SW_LF_PRESS) {
				if(plan.speed==0) plan.speed = 99; else plan.speed-=1;
				bt_status = 1;
				goto ui_mem;
			}
			else if(bt & SW_RG_PRESS) {
				if(plan.speed==99) plan.speed = 0; else plan.speed+=1;
				bt_status = 1;
				goto ui_mem;
			}
		case 3 :
			if(bt & SW_LF_PRESS) {
				if(plan.turn_speed==0) plan.turn_speed = 99; else plan.turn_speed-=1;
				bt_status = 1;
				goto ui_mem;
			}
			else if(bt & SW_RG_PRESS) {
				if(plan.turn_speed==99) plan.turn_speed = 0; else plan.turn_speed+=1;
				bt_status = 1;
				goto ui_mem;
			}

		}
	}
	return UI_IDLE;
}



UI_Menu_Lists UI_Run() {
	static uint8_t bt;

	char buff[22];
	Plan_Start();
	ui_run:
	Plan_UIRoutine();
	sprintf(buff, "i:%03i  cp:%02i  Plan%i", plan.checkpoint[num_checkpoint], num_checkpoint, num_plan);
	DISP_DisplayText(&home_status, buff);
	sprintf(buff, "Act ~ %s", plan_act_lists[plan_active.act]);
	DISP_DisplayText(&cp_menu[2], buff);
	sprintf(buff, "%3i | %-3i", (int16_t)motor[1].pwm, (int16_t)motor[0].pwm);
	DISP_DisplayText(&cp_menu[3], buff);
	while(1) {
		if((main_flag & MAIN_FLAG_RUN) == 0) break;
		Plan_UIRoutine();
		UI_ShowSensor();
		bt = UI_ReadButton();
		if(bt & SW_HM_PRESS) {
			main_flag &=~0x01;
			Plan_Stop();
			return UI_HOMESCREEN;
		}
		if(main_flag & 0x80) {
			main_flag &=~0x80;
			goto ui_run;
		}
	}
	DISP_DisplayText(&short_message, "FINISH!");
	HAL_Delay(500);
//	num_checkpoint;
	return UI_HOMESCREEN;
}


void UI_Init() {
	UI_HomeScreenInit();
	UI_SettingDisplayInit();
	UI_PlanSetDisplayInit();
	UI_ADCThresholdDisplayInit();
	UI_CPDisplayInit();
}

void UI_Main() {
	static UI_Menu_Lists retval = UI_STARTUP;
	DISP_ClearAll();
	switch(retval) {
	case UI_STARTUP:
		retval = UI_Startup();
		break;
	case UI_HOMESCREEN:
		retval = UI_HomeScreen();
		break;
	case UI_SPEED_SET:
		retval = UI_SpeedSetting();
		break;
	case UI_RUN:
		retval = UI_Run();
		break;
	case UI_PLAN_SET:
		retval = UI_PlanSet();
		break;
	case UI_PLAN_SELECT:
		retval = UI_PlanSelect();
		break;
	case UI_SET:
		retval = UI_Setting();
		break;
	case UI_SET_ADC:
		retval = UI_ADCThreshold();
		break;
	case UI_SET_CALIB:
		retval = UI_AutoCalib();
		break;
	case UI_SET_CP:
		retval = UI_CP();
		break;
	case UI_SET_PID:
		retval = UI_PIDSetting();
		break;
	case UI_SET_MEM:
		retval = UI_MemSetting();
		break;
	default:
		;
	}
}
