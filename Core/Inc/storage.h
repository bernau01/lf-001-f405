/*
 * storage.h
 *
 *  Created on: Jul 29, 2023
 *      Author: Zain Irsyad
 */

#ifndef INC_STORAGE_H_
#define INC_STORAGE_H_

#include "main.h"

#define MAX_PLAN				8
#define MAX_ACTION_PER_PLAN		200
#define DEF_ACTION_PER_PLAN		400
#define MAX_CHECKPOINT			20
#define BUFFER_SIZE				50

#define INIT_ADDR_START				0x0000
#define INIT_ADDR_END				0x000F
#define PTR_MEMORY_ADDR_START		0x0010
#define PTR_MEMORY_ADDR_END			0x001F
#define ADC_TR_ADDR_START			0x0020
#define ADC_TR_ADDR_END				0x003F
#define DATA_ADDR_START				0x0040
#define DATA_ADDR_END				0x7000

#define INIT_MEM_ID_ADDR		0x0001
#define INIT_MEM_ID 			0xAB
#define INIT_MEM_NUM_PLAN_ADDR	0x0003
#define INIT_MEM_NUM_PLAN		MAX_PLAN
#define INIT_MEM_LAST_PLAN_ADDR	0x0004

#define ACTION_PER_PAGE 4
#define ACTION_SIZE		16
#define PLAN_INIT_SIZE	64

#define NUM_CHECKPOINT 	15

#define STO_FLAG_BUFF_FULL 	0x01
#define STO_FLAG_BUFF_EMPTY 0x02
#define STO_FLAG_BUFF_END	0x04

#define GET_LINE_COLOR_STAT(x) (x&0x03)
#define SET_LINE_COLOR_STAT(x,val) {x &=~0x03; x |= (val&0x03);}
#define ADD_LINE_COLOR_STAT(x,val) {x=(x+val)&0x03;}

#define LINE_COLOR_STAT_IDEM 0
#define LINE_COLOR_STAT_BLACK 1
#define LINE_COLOR_STAT_WHITE 2

typedef struct {
	uint8_t act;
	uint8_t sen_trig;
	uint8_t act_mode;
	uint8_t act_value;
	uint8_t brake;
	int8_t forward_speed;
	int8_t reverse_speed;
	uint8_t wait_time;
	uint8_t boost_time;
	uint8_t boost_speed;
	uint8_t unit_speed;
	uint8_t status;
	uint8_t kp;
} Action_typedef;

typedef struct {
	uint8_t id;
	uint16_t num_action;
	uint16_t checkpoint[NUM_CHECKPOINT];
	uint8_t speed;
	uint8_t turn_speed;
	float kpid[3];
	uint8_t status_pid;
	float alpha;
	float acc;
} Plan_typedef;

//typedef struct {
//	Action_typedef action[150];
//	uint8_t checkpoint[20];
//	uint8_t code[4];
//	uint8_t master_speed;
//	uint8_t master_turn;
//	float kpid[3];
//} Plan_typedef;
extern uint8_t num_plan;
extern int num_index;
extern uint8_t num_checkpoint;
extern int max_index;
extern uint8_t plan_speed;
extern uint8_t plan_turn_speed;
extern Action_typedef plan_active;
extern Plan_typedef plan;

void Storage_ResetPlan();
void Storage_ResetMemory();
uint8_t Storage_Init();
uint8_t Storage_PlanInit();
Action_typedef GetAction(int index);
void SetAction(Action_typedef act, int index);
void Storage_GetADCThreshold();
void Storage_SetADCThreshold();
uint8_t Storage_GetPlan(uint8_t n_plan);
void Storage_SetCheckpoint();
void Storage_GetCheckpoint();
void Storage_CleanInit();
void Storage_SetPID();
void Storage_SetSpeed();
uint8_t GetActionSequence();
void GetActionSequenceRoutine();
void GetActionSequenceInit(int num);
Action_typedef GetActionOther(uint8_t plan, int index);
void SetActionOther(Action_typedef act, int index);
void SetActionOther2(uint8_t n_plan, Action_typedef act, int index);
void Storage_CopyPlan(uint8_t plan_dest);
void Storage_InsertAction(int index);
void Storage_DeleteAction(int index);

#endif /* INC_STORAGE_H_ */
