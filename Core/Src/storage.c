/*
 * storage.c
 *
 *  Created on: Jul 29, 2023
 *      Author: Zain Irsyad
 */

#include "storage.h"
#include "eeprom.h"
#include "sensor.h"
#include "memory.h"

uint8_t num_plan;
uint16_t num_plan_addr;
uint8_t num_checkpoint;
uint8_t num_index;

uint8_t max_index = DEF_ACTION_PER_PLAN-1;

uint8_t init_buff[PAGE_SIZE];

uint8_t last_index = 0;
Action_typedef plan_buffer[BUFFER_SIZE];
uint8_t buff_index = 0;
uint8_t plan_buffer_f = 0;
uint8_t plan_buffer_i = 0;
Action_typedef plan_active;
const Action_typedef plan_default = {
		.act = 0,
		.sen_trig = 2,
		.act_mode = 0,
		.act_value = 5,
		.brake = 0,
		.forward_speed = 60,
		.reverse_speed = -20,
		.wait_time = 0,
		.boost_time = 0,
		.boost_speed = 50,
		.unit_speed = 0,
		.status = 0,
};

uint8_t storage_flag = 0;

Plan_typedef plan;

Action_typedef GetAction(uint8_t index) {
	index = index%DEF_ACTION_PER_PLAN;
	uint8_t buff[ACTION_SIZE];
	uint16_t act_addr = num_plan_addr + PLAN_INIT_SIZE + (index * ACTION_SIZE) ;
	while(!EEP_ReadyToWrite());
	EEP_ReadMem(act_addr, buff, ACTION_SIZE);
	memcpy((uint8_t*)&plan_active, buff, sizeof(Action_typedef));
	return plan_active;
}

void SetAction(Action_typedef act, uint8_t index) {
	index = index%DEF_ACTION_PER_PLAN;
	uint8_t buff[ACTION_SIZE];
	plan_active = act;
	uint16_t act_addr = num_plan_addr + PLAN_INIT_SIZE + (index * ACTION_SIZE);
	memcpy(buff, (uint8_t*)&plan_active, sizeof(Action_typedef));
	while(!EEP_ReadyToWrite());
	EEP_WriteMem(act_addr, buff, ACTION_SIZE);
}

void Storage_ResetPlan() {
	Plan_typedef temp;
	temp.id = num_plan | 0x80;
	temp.speed = 40;
	temp.num_action = 100;
	temp.turn_speed = 10;
	memset(temp.checkpoint, 0, NUM_CHECKPOINT);
	temp.kpid[0] = 4.71428;
	temp.kpid[1] = 0;
	temp.kpid[2] = 0;
	temp.status_pid = 0;
	temp.acc = 1;

	uint8_t buff[PAGE_SIZE];
	memcpy(buff, (uint8_t*)&temp, sizeof(Plan_typedef));

	uint16_t page_plan = num_plan_addr/PAGE_SIZE;
//	uint16_t page_offset = DEF_ACTION_PER_PLAN/ACTION_PER_PAGE+1;

	while(!EEP_ReadyToWrite());
	EEP_WritePage(page_plan, buff);

	Action_typedef action_temp;
	uint8_t action_buff[PAGE_SIZE];

	for(int i=0; i<DEF_ACTION_PER_PLAN; i++) {

		action_temp = plan_default;

		memcpy(action_buff, (uint8_t*)&action_temp, sizeof(Action_typedef));

		SetAction(action_temp, i);
	}
}

void Storage_ResetMemory() {
	uint8_t init_buff[PAGE_SIZE];
	memset(init_buff, 0, PAGE_SIZE);
	init_buff[INIT_MEM_ID_ADDR] = INIT_MEM_ID;
	init_buff[INIT_MEM_NUM_PLAN_ADDR] = INIT_MEM_NUM_PLAN;
	init_buff[INIT_MEM_LAST_PLAN_ADDR] = 0;
	uint16_t* temp = (uint16_t*)&init_buff[PTR_MEMORY_ADDR_START];
	for(int i=0; i<INIT_MEM_NUM_PLAN; i++) {
		temp[i] = (((i*ACTION_SIZE*(DEF_ACTION_PER_PLAN+4))/PAGE_SIZE))*PAGE_SIZE+DATA_ADDR_START;
	}
	temp = (uint16_t*)&init_buff[ADC_TR_ADDR_START];
	for(int i=0; i<SENSOR_NUM; i++) {
		temp[i] = ADC_MAX/2;
	}
	while(!EEP_ReadyToWrite());
	EEP_WritePage(0x0000, init_buff);
}

uint8_t Storage_Init() {
	while(!EEP_ReadyToWrite());
	EEP_ReadPage(INIT_ADDR_START/PAGE_SIZE, init_buff);
	if(init_buff[INIT_MEM_ID_ADDR] != INIT_MEM_ID) return 0;
	num_plan = init_buff[INIT_MEM_LAST_PLAN_ADDR];
	num_plan_addr = *(uint16_t*)&init_buff[PTR_MEMORY_ADDR_START+num_plan*2];
	memcpy((uint8_t*)threshold_sensor, &init_buff[ADC_TR_ADDR_START], SENSOR_NUM*2);
	return 1;
}

uint8_t Storage_PlanInit() {
	uint8_t buff[PAGE_SIZE];
	uint8_t plan_page = (num_plan_addr)/PAGE_SIZE;
	while(!EEP_ReadyToWrite());
	EEP_ReadPage(plan_page, buff);
	memcpy((uint8_t*)&plan, buff, sizeof(Plan_typedef));
	if(plan.id>>7 != 1) return 0;
	return 1;
}

void Storage_GetADCThreshold() {
	while(!EEP_ReadyToWrite());
	EEP_ReadMem(ADC_TR_ADDR_START, &init_buff[ADC_TR_ADDR_START], SENSOR_NUM*2);
	memcpy((uint8_t*)threshold_sensor, &init_buff[ADC_TR_ADDR_START], SENSOR_NUM*2);
}

void Storage_SetADCThreshold() {
	memcpy(&init_buff[ADC_TR_ADDR_START], (uint8_t*)threshold_sensor, SENSOR_NUM*2);
	while(!EEP_ReadyToWrite());
	EEP_WriteMem(ADC_TR_ADDR_START, &init_buff[ADC_TR_ADDR_START], SENSOR_NUM*2);
}

uint8_t Storage_GetPlan(uint8_t n_plan) {
	num_plan = n_plan;
	init_buff[INIT_MEM_LAST_PLAN_ADDR] = num_plan;
	num_plan_addr = *(uint16_t*)&init_buff[PTR_MEMORY_ADDR_START+num_plan*2];
	while(!EEP_ReadyToWrite());
	EEP_WriteMem(INIT_MEM_LAST_PLAN_ADDR, &num_plan, 1);

	uint8_t buff[PAGE_SIZE];
	uint8_t plan_page = (num_plan_addr)/PAGE_SIZE;
	while(!EEP_ReadyToWrite());
	EEP_ReadPage(plan_page, buff);
	memcpy((uint8_t*)&plan, buff, sizeof(Plan_typedef));
	if(plan.id>>7 != 1) return 0;
	return 1;
}

void Storage_SetPlanOther(Plan_typedef* ptr_plan, uint8_t n_plan) {
	uint16_t n_plan_addr = *(uint16_t*)&init_buff[PTR_MEMORY_ADDR_START+n_plan*2];
	uint8_t plan_page = (n_plan_addr)/PAGE_SIZE;
	uint8_t buff[PAGE_SIZE];
	memcpy(buff, (uint8_t*)ptr_plan, sizeof(Plan_typedef));
	while(!EEP_ReadyToWrite());
	EEP_WritePage(plan_page, buff);
}

void Storage_GetCheckpoint() {
	uint8_t offset = plan.checkpoint - (uint8_t*)&plan;
	uint8_t plan_page = (num_plan_addr)/PAGE_SIZE;
	while(!EEP_ReadyToWrite());
	EEP_ReadMem(plan_page+offset, plan.checkpoint, NUM_CHECKPOINT);
}

void Storage_SetCheckpoint() {
	uint8_t offset = (uint8_t*)&plan.checkpoint - (uint8_t*)&plan;
	while(!EEP_ReadyToWrite());
	EEP_WriteMem(num_plan_addr+offset, plan.checkpoint, NUM_CHECKPOINT);
}

void Storage_SetPID() {
	uint8_t offset = (uint8_t*)&plan.kpid - (uint8_t*)&plan;
	while(!EEP_ReadyToWrite());
	EEP_WriteMem(num_plan_addr+offset, (uint8_t*)plan.kpid, 3*sizeof(float));

	offset = (uint8_t*)&plan.status_pid - (uint8_t*)&plan;
	while(!EEP_ReadyToWrite());
	EEP_WriteMem(num_plan_addr+offset, (uint8_t*)plan.status_pid, 1);
}

void Storage_SetSpeed() {
	uint8_t offset = (uint8_t*)&plan.speed - (uint8_t*)&plan;
	while(!EEP_ReadyToWrite());
	EEP_WriteMem(num_plan_addr+offset, (uint8_t*)&plan.speed, 2);
}

Action_typedef GetActionOther(uint8_t plan, uint8_t index) {
	index = index%DEF_ACTION_PER_PLAN;
	uint16_t addr = *(uint16_t*)&init_buff[PTR_MEMORY_ADDR_START+plan*2];
	uint8_t buff[ACTION_SIZE];
	uint16_t act_addr = addr + PLAN_INIT_SIZE + (index * ACTION_SIZE) ;
	while(!EEP_ReadyToWrite());
	EEP_ReadMem(act_addr, buff, ACTION_SIZE);
	return *(Action_typedef*)&buff;
}

void SetActionOther(Action_typedef act, uint8_t index) {
	index = index%DEF_ACTION_PER_PLAN;
	uint8_t buff[ACTION_SIZE];
	uint16_t act_addr = num_plan_addr + PLAN_INIT_SIZE + (index * ACTION_SIZE);
	memcpy(buff, (uint8_t*)&act, sizeof(Action_typedef));
	while(!EEP_ReadyToWrite());
	EEP_WriteMem(act_addr, buff, ACTION_SIZE);
}

void SetActionOther2(uint8_t n_plan, Action_typedef act, uint8_t index) {
	index = index%DEF_ACTION_PER_PLAN;
	uint8_t buff[ACTION_SIZE];
	uint16_t n_plan_addr = *(uint16_t*)&init_buff[PTR_MEMORY_ADDR_START+n_plan*2];
	uint16_t act_addr = n_plan_addr + PLAN_INIT_SIZE + (index * ACTION_SIZE);
	memcpy(buff, (uint8_t*)&act, sizeof(Action_typedef));
	while(!EEP_ReadyToWrite());
	EEP_WriteMem(act_addr, buff, ACTION_SIZE);
}

uint8_t GetActionSequence() {
	uint8_t retval;
	if(!(storage_flag & STO_FLAG_BUFF_EMPTY)) {

		if(plan_buffer_i>=BUFFER_SIZE-1) plan_buffer_i=0; else plan_buffer_i++;

		plan_active = plan_buffer[plan_buffer_i];

		if(plan_buffer_i == plan_buffer_f) {
			storage_flag |= STO_FLAG_BUFF_EMPTY;
		}
		storage_flag &=~STO_FLAG_BUFF_FULL;
		retval = 1;
	} else {
		retval = 0;
	}

	return retval;
}

void GetActionSequenceRoutine() {
	if( (plan_buffer_i > 0 && (plan_buffer_f != plan_buffer_i-1)) ||
		(plan_buffer_i == 0 && (plan_buffer_f != BUFFER_SIZE-1)) ) {
		if(buff_index < plan.num_action-2) {
			buff_index++;
			if(plan_buffer_f>=BUFFER_SIZE-1) plan_buffer_f=0; else plan_buffer_f++;
			plan_buffer[plan_buffer_f] = GetActionOther(num_plan, buff_index);
			if(plan_buffer[plan_buffer_f].act == 11) {
				buff_index = plan_buffer[plan_buffer_f].act_value-1;
			}
			storage_flag &=~STO_FLAG_BUFF_EMPTY;
		}
		else {
			storage_flag |= STO_FLAG_BUFF_END;
		}
	}
	else {
		storage_flag |= STO_FLAG_BUFF_FULL;
	}
}

void GetActionSequenceInit(uint8_t num) {
	GetAction(num);
	buff_index = num;
	plan_buffer_i = 0;
	plan_buffer_f = 0;
	storage_flag = 0;
	memset((uint8_t*)&plan_buffer, 0, sizeof(Action_typedef)*BUFFER_SIZE);
	storage_flag |= STO_FLAG_BUFF_EMPTY;
}

void Storage_InsertAction(uint8_t index) {
	Action_typedef temp;
	for(int i=plan.num_action-1; i>index; i--) {
		temp = GetActionOther(num_plan, i-1);
		SetActionOther(temp, i);
	}
}

void Storage_DeleteAction(uint8_t index) {
	Action_typedef temp;
	for(int i=index; i<plan.num_action-1; i++) {
		temp = GetActionOther(num_plan, i+1);
		SetActionOther(temp, i);
	}
	SetActionOther(plan_default, plan.num_action-1);
}

void Storage_CopyPlan(uint8_t plan_dest) {
	Storage_SetPlanOther(&plan, plan_dest);
	Action_typedef temp;
	for(int i=0; i<plan.num_action; i++) {
		temp = GetActionOther(num_plan, i);
		SetActionOther2(plan_dest, temp, i);
	}
}

void Storage_CleanInit() {
	uint8_t ok[PAGE_SIZE];
	memset(ok, 0, PAGE_SIZE);
	for(int i=0; i<PAGE_NUM; i++) {
		while(!EEP_ReadyToWrite());
		EEP_WritePage(i, ok);
	}
}

