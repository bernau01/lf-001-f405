/*
 * plan.c
 *
 *  Created on: Jul 31, 2023
 *      Author: Zain Irsyad
 */

#include "plan.h"
#include "storage.h"
#include "run.h"
#include "sensor.h"
#include <math.h>

#define LEFT_SENSOR(x)  SENSOR_NUM-1-x
#define RIGHT_SENSOR(x) x

#define LEFT_ALIGN_SEN 0x00
#define RIGHT_ALIGN_SEN 0x01
#define LR_ALIGN_SEN 0x02

#define BRAKE_MAX_DELAY 3

uint32_t plan_counter;
uint32_t plan_last_counter;
uint32_t plan_last_counter2;
int32_t plan_last_enc;

uint8_t counter_status = 0;
uint8_t action_status = 0;
//uint8_t acc_status = 0;
uint8_t autoturn_stat = 0;
uint8_t speed_now;

void Plan_SetCounter() {
	plan_last_counter = plan_counter;
}

uint8_t Plan_CheckCounterValue(uint32_t value) {
	return (plan_counter - plan_last_counter) >= value*DELAY_FACTOR;
}

uint32_t Plan_CounterValue() {
	return (plan_counter - plan_last_counter);
}

/*******************************COUNTER**********************************/

void Plan_StartCounter(PlanCounter_t* c) {
    c->status = COUNTER_RUN;
    c->counter_stamp = c->counter;
}

void Plan_Counter(PlanCounter_t* c) {
    c->counter+=1;
}

uint8_t Plan_CheckCounterVal(PlanCounter_t* c, uint32_t value) {
    return (c->counter - c->counter_stamp) >= value*DELAY_FACTOR;
}


PlanCounter_t action_delay_counter;
PlanCounter_t duration_counter;


/**************************************CHECK SENSOR****************************************/

uint8_t Plan_CheckSensor(uint8_t s, uint8_t align) {
	if(s == 0) return 1;
	else {
		s-=1;
		switch(align) {
		case RIGHT_ALIGN_SEN:
			return sensor_disp_status[RIGHT_SENSOR(s)] > 0;
		case LR_ALIGN_SEN:
			return (sensor_disp_status[RIGHT_SENSOR(s)] > 0) || (sensor_disp_status[LEFT_SENSOR(s)] > 0);
		case LEFT_ALIGN_SEN:
		default:
			return sensor_disp_status[LEFT_SENSOR(s)] > 0;
		}
	}
}




/*********************************ACTION***********************************/

//#define ACTION_IDLE			0
//#define ACTION_WAIT_TIME	1
//#define ACTION_BOOST_TIME 	2
//#define ACTION_FIND_SENSOR	3
//#define ACTION_EXECUTE_1	4
//#define ACTION_EXECUTE_2	5
//#define ACTION_EXECUTE_3	6


PlanStatus_t plan_status;
ActionInitStatus_t action_init_status;
ActionStatus_t action_status_;

PlanStatus_t Plan_Left2(Action_typedef action) {
	switch (action_status_) {

	case ACTION_STATUS_IDLE :
		action_status_ = ACTION_STATUS_FIND;

	case ACTION_STATUS_FIND :
		if(Plan_CheckSensor(action.sen_trig, LEFT_ALIGN_SEN)) {
			action_status_ = ACTION_STATUS_BRAKE_1;
		}
		else return PLAN_STATUS_IDLE;

	case ACTION_STATUS_BRAKE_1 :
		Run_SetReverseSpeed(0.5);
		if(Plan_CheckCounterValue((action.brake<BRAKE_MAX_DELAY)?action.brake:BRAKE_MAX_DELAY)) {
			action_status_ = ACTION_STATUS_BRAKE_2;
		}
		else return PLAN_STATUS_EXECUTE;

	case ACTION_STATUS_BRAKE_2 :
		Run_SetReverseSpeed(0.5);
		if(Plan_CheckCounterValue((action.brake<BRAKE_MAX_DELAY)?action.brake:BRAKE_MAX_DELAY)) {
			action_status_ = ACTION_STATUS_BRAKE_2;
		}
		else return PLAN_STATUS_EXECUTE;

	case ACTION_STATUS_EXECUTE_1 :

	}
}

uint8_t Plan_Left(Action_typedef a) {
	if(Plan_CheckSensor(a.sen_trig, LEFT_ALIGN_SEN) && counter_status == 0) {
			Plan_SetCounter();
			plan_last_enc = MOTOR_R.enc_cnt;
			counter_status = 8;
			Run_SetReverseSpeed(0.5);
	}
	if(counter_status == 8) {
		if(Plan_CheckCounterValue((a.brake<BRAKE_MAX_DELAY)?a.brake:BRAKE_MAX_DELAY)) {
			Plan_SetCounter();
			counter_status = 1;
		}
		return 1;
	}
	else if(counter_status == 1) {
		if(Plan_CheckCounterValue(a.brake)) {
			Plan_SetCounter();
			counter_status = 2;
		}
		return 1;
	}
	else if(counter_status == 2) {
		if(autoturn_stat == 0) {
			Run_SetMotorSpeed(a.reverse_speed*plan.turn_speed*0.1, a.forward_speed*plan.turn_speed*0.1);
		}
		else {
			Run_SetMotorSpeed(a.reverse_speed*plan.turn_speed*0.05, a.forward_speed*plan.turn_speed*0.05);
		}
		switch(a.act_mode) {
		case 0:
			if(Plan_CheckCounterValue(a.act_value)) return 2;
			break;
		case 1:
			if(Plan_CheckCounterValue(a.act_value)) {
				if(Plan_CheckSensor(3, LEFT_ALIGN_SEN)) {autoturn_stat = 1;}
				if(Plan_CheckSensor(5, LEFT_ALIGN_SEN)) {autoturn_stat = 0; return 2;}
			}
			break;
		case 2:
			if(MOTOR_R.enc_cnt - plan_last_enc > (int32_t)a.act_value*ENC_FACTOR) return 2;
			break;
		}
		return 1;
	}
	return 0;
}

uint8_t Plan_Right(Action_typedef a) {
	if(Plan_CheckSensor(a.sen_trig, RIGHT_ALIGN_SEN) && counter_status == 0) {
			Plan_SetCounter();
			plan_last_enc = MOTOR_L.enc_cnt;
			counter_status = 8;
			Run_SetReverseSpeed(0.5);
	}
	if(counter_status == 8) {
		if(Plan_CheckCounterValue((a.brake<BRAKE_MAX_DELAY)?a.brake:BRAKE_MAX_DELAY)) {
			Plan_SetCounter();
			counter_status = 1;
		}
		return 1;
	}
	else if(counter_status == 1) {
		if(Plan_CheckCounterValue(a.brake)) {
			Plan_SetCounter();
			counter_status = 2;
		}
		return 1;
	}
	if(counter_status == 2) {
		if(autoturn_stat == 0) {
			Run_SetMotorSpeed(a.forward_speed*plan.turn_speed*0.1, a.reverse_speed*plan.turn_speed*0.1);
		}
		else {
			Run_SetMotorSpeed(a.forward_speed*plan.turn_speed*0.05, a.reverse_speed*plan.turn_speed*0.05);
		}
		switch(a.act_mode) {
		case 0:
			if(Plan_CheckCounterValue(a.act_value)) return 2;
			break;
		case 1:
			if(Plan_CheckCounterValue(a.act_value)) {
				if(Plan_CheckSensor(3, RIGHT_ALIGN_SEN)) {autoturn_stat = 1;}
				if(Plan_CheckSensor(5, RIGHT_ALIGN_SEN)) {autoturn_stat = 0; return 2;}
			}
			break;
		case 2:
			if(MOTOR_L.enc_cnt - plan_last_enc > (int32_t)a.act_value*ENC_FACTOR) return 2;
			break;
		}
		return 1;
	}
	return 0;
}

uint8_t Plan_Forward(Action_typedef a, uint8_t _speed) {
	if(Plan_CheckSensor(a.sen_trig, LR_ALIGN_SEN) && counter_status == 0) {
			Plan_SetCounter();
			plan_last_enc = robot_enc_pos;
			counter_status = 8;
			Run_SetReverseSpeed(0.5);
	}
	if(counter_status == 8) {
		if(Plan_CheckCounterValue((a.brake<BRAKE_MAX_DELAY)?a.brake:BRAKE_MAX_DELAY)) {
			Plan_SetCounter();
			counter_status = 1;
		}
		return 1;
	}
	else if(counter_status == 1) {
		if(Plan_CheckCounterValue(a.brake)) {
			Plan_SetCounter();
			counter_status = 2;
		}
		return 1;
	}
	if(counter_status == 2) {
		Run_SetMotorSpeed(_speed, _speed);
		switch(a.act_mode) {
		case 0:
			if(Plan_CheckCounterValue(a.act_value)) return 2;
			break;
		case 1:
			if(Plan_CheckCounterValue(a.act_value))
				if(Plan_CheckSensor(7, LR_ALIGN_SEN)) return 2;
			break;
		case 2:
			if(robot_enc_pos - plan_last_enc > (int32_t)a.act_value*ENC_FACTOR) return 2;
			break;
		}
		return 1;
	}
	return 0;
}

uint8_t Plan_Backward(Action_typedef a, uint8_t _speed) {
	if(Plan_CheckSensor(a.sen_trig, LR_ALIGN_SEN) && counter_status == 0) {
			Plan_SetCounter();
			plan_last_enc = robot_enc_pos;
			counter_status = 8;
			Run_SetReverseSpeed(0.5);
	}
	if(counter_status == 8) {
		if(Plan_CheckCounterValue((a.brake<BRAKE_MAX_DELAY)?a.brake:BRAKE_MAX_DELAY)) {
			Plan_SetCounter();
			counter_status = 1;
		}
		return 1;
	}
	else if(counter_status == 1) {
		if(Plan_CheckCounterValue(a.brake)) {
			Plan_SetCounter();
			counter_status = 2;
		}
		return 1;
	}
	if(counter_status == 2) {
		Run_SetMotorSpeed(-_speed, -_speed);
		switch(a.act_mode) {
		case 0:
			if(Plan_CheckCounterValue(a.act_value)) return 2;
			break;
		case 1:
			if(Plan_CheckCounterValue(a.act_value))
				if(Plan_CheckSensor(7, LR_ALIGN_SEN)) return 2;
			break;
		case 2:
			if(plan_last_enc - robot_enc_pos > (int32_t)a.act_value*ENC_FACTOR) return 2;
			break;
		}
		return 1;
	}
	return 0;
}

uint8_t Plan_Idle(Action_typedef a) {
	if(Plan_CheckSensor(a.sen_trig, LR_ALIGN_SEN) && counter_status == 0) {
			Plan_SetCounter();
			plan_last_enc = robot_enc_pos;
			counter_status = 8;
			Run_SetReverseSpeed(0.5);
	}
	if(counter_status == 8) {
		if(Plan_CheckCounterValue((a.brake<BRAKE_MAX_DELAY)?a.brake:BRAKE_MAX_DELAY)) {
			Plan_SetCounter();
			counter_status = 1;
		}
		return 1;
	}
	else if(counter_status == 1) {
		if(Plan_CheckCounterValue(a.brake)) {
			Plan_SetCounter();
			counter_status = 2;
		}
		return 1;
	}
	if(counter_status == 2) {
		Run_SetMotorSpeed(0, 0);
		switch(a.act_mode) {
		case 0:
			if(Plan_CheckCounterValue(a.act_value)) return 2;
			break;
//		case 1:
//			if(Plan_CheckCounterValue(a.act_value))
//				if(Plan_CheckSensor(7, LR_ALIGN_SEN)) return 2;
//			break;
//		case 2:
//			if(robot_enc_pos - plan_last_enc > (int32_t)a.act_value*ENC_FACTOR) return 2;
//			break;
		}
		return 1;
	}
	return 0;
}

uint8_t Plan_Follow(Action_typedef a) {
	if(counter_status == 0) {
		Plan_SetCounter();
		robot_enc_pos = MOTOR_L.enc_cnt;
		counter_status = 1;
	}
	switch(a.act_mode) {
	case 0:
		if(Plan_CheckCounterValue(a.act_value) && a.act_value!=0) return 2;
		break;
	case 2:
		if(robot_enc_pos - plan_last_enc > (int32_t)a.act_value*ENC_FACTOR) return 2;
		break;
	default: return 2;
	}
	return 0;
}

uint8_t Plan_FollowLeft(Action_typedef a) {
	if(counter_status == 0) {
		Plan_SetCounter();
		plan_last_enc = robot_enc_pos;
		counter_status = 1;
	}
	if(Plan_CheckSensor(a.sen_trig, LEFT_ALIGN_SEN) && counter_status == 1) {
		counter_status = 3;
		Run_SetReverseSpeed(0.5);
	}
	if(counter_status == 3) {
		if(Plan_CheckCounterValue((a.brake<BRAKE_MAX_DELAY)?a.brake:BRAKE_MAX_DELAY)) {
			Plan_SetCounter();
			counter_status = 2;
		}
		return 1;
	}
	else if(counter_status == 2) {
		Run_SetMotorSpeed(a.reverse_speed*plan.turn_speed*0.1, a.forward_speed*plan.turn_speed*0.1);
		if(Plan_CheckSensor(3, LEFT_ALIGN_SEN) > 0) {
			counter_status = 4;
		}
		return 1;
	}
	else if(counter_status == 4) {
		Run_SetMotorSpeed(a.reverse_speed*plan.turn_speed*0.05, a.forward_speed*plan.turn_speed*0.05);
		if(Plan_CheckSensor(5, LEFT_ALIGN_SEN) > 0) {
			counter_status = 1;
		}
		return 1;
	}
	switch(a.act_mode) {
	case 0:
		if(Plan_CheckCounterValue(a.act_value) && a.act_value!=0) return 2;
		break;
	case 2:
		if(robot_enc_pos - plan_last_enc > (int32_t)a.act_value*ENC_FACTOR) return 2;
		break;
	default: return 2;
	}
	return 0;
}

uint8_t Plan_FollowRight(Action_typedef a) {
	if(counter_status == 0) {
		Plan_SetCounter();
		plan_last_enc = robot_enc_pos;
		counter_status = 1;
	}
	if(Plan_CheckSensor(a.sen_trig, RIGHT_ALIGN_SEN) && counter_status == 1) {
		counter_status = 3;
		Run_SetReverseSpeed(0.5);
	}
	if(counter_status == 3) {
		if(Plan_CheckCounterValue((a.brake<BRAKE_MAX_DELAY)?a.brake:BRAKE_MAX_DELAY)) {
			Plan_SetCounter();
			counter_status = 2;
		}
		return 1;
	}
	else if(counter_status == 2) {
		Run_SetMotorSpeed(a.forward_speed*plan.turn_speed*0.1, a.reverse_speed*plan.turn_speed*0.1);
		if(Plan_CheckSensor(3, RIGHT_ALIGN_SEN) > 0) {
			counter_status = 4;
		}
		return 1;
	}
	else if(counter_status == 4) {
		Run_SetMotorSpeed(a.forward_speed*plan.turn_speed*0.05, a.reverse_speed*plan.turn_speed*0.05);
		if(Plan_CheckSensor(5, RIGHT_ALIGN_SEN) > 0) {
			counter_status = 1;
		}
		return 1;
	}
	switch(a.act_mode) {
	case 0:
		if(Plan_CheckCounterValue(a.act_value) && a.act_value!=0) return 2;
		break;
	case 2:
		if(robot_enc_pos - plan_last_enc > (int32_t)a.act_value*ENC_FACTOR) return 2;
		break;
	default: return 2;
	}
	return 0;
}


uint8_t Plan_Invert(Action_typedef a) {
	if(Plan_CheckSensor(a.sen_trig, LR_ALIGN_SEN)) {
		SEN_Invert();
		return 2;
	}
	return 0;
}

uint8_t Plan_NextPlan(Action_typedef a) {
	return 2;
}


uint8_t Plan_Null(Action_typedef a) {
	return 2;
}

uint8_t Plan_Jump(Action_typedef a) {
	if(a.act_value > 0) num_index = a.act_value-1;
	return 2;
}

uint8_t Plan_ActionInit(Action_typedef a, float period, uint8_t sp) {
	static uint8_t stat = 0;
	static uint8_t _speed = 0;
	if(counter_status == 0) {
		Plan_SetCounter();
		counter_status = 1;
	}

	if(Plan_CheckCounterValue(a.wait_time) && stat == 0) {
		counter_status = 0;
		stat = 1;
		_speed = sp;
		return 0;
	}

	if(stat == 0) return 0;

	if(Plan_CheckCounterValue(a.boost_time) && stat == 1) {
		counter_status = 0;
//		acc_status = 0;
		stat = 0;
		return 3;
	}

//	if(Plan_CheckCounterValue(plan.acc) && stat == 1 && acc_status == 1) {
//		acc_status = 2;
//	}

//	if(acc_status == 0) {
//		_speed = sp;
//		acc_status = 1;
//	}
//	else if(acc_status == 1) {
//		if(sp > a.boost_speed) {
//			_speed = (sp-a.boost_speed)*(float)(Plan_CounterValue()/(float)plan.acc);
//			_speed += a.boost_speed;
//		}
//		else if(sp < a.boost_speed){
//			_speed = (a.boost_speed-sp)*(float)(Plan_CounterValue()/(float)plan.acc);
//			_speed += sp;
//		}
//	}
//	else if(acc_status == 2) {

		float range;
		if(fabsf(a.boost_speed - _speed) < plan.acc) range = fabsf(a.boost_speed - _speed);
		else range = plan.acc;

		if(a.boost_speed > _speed) _speed += range;
		else if(a.boost_speed < _speed) _speed -= range;

//		_speed = a.boost_speed;
//	}
	Run_LineTracing(_speed, period, plan.status_pid);
//	Run_LineTracing(a.boost_speed, period, 0);
	return 1;
}

uint8_t Plan_Brake(Action_typedef a) {
	if(counter_status == 0) {
		Plan_SetCounter();
		counter_status = 1;
	}

	Run_SetMotorSpeed(0, 0);
	if(Plan_CheckCounterValue(a.brake)) {
		counter_status = 0;
		return 2;
	}
	return 0;
}


uint8_t Plan_Stop() {
	Run_SetMotorSpeed(0, 0);
	SEN_BlackLine();
	num_checkpoint = 0;
	num_index = plan.checkpoint[num_checkpoint];
	main_flag &=~ MAIN_FLAG_RUN;
	run_counter++;
	if(run_counter > 30) {
		HAL_NVIC_SystemReset();
	}
	return 1;
}

void Plan_Start() {
	num_index = plan.checkpoint[num_checkpoint];
	GetActionSequenceInit(num_index);
	action_status = 0;
	plan_counter = 0;
	plan_last_counter = 0;
	sum_error = 0;
	kp = plan.kpid[0];
	ki = plan.kpid[1];
	kd = plan.kpid[2];
	alpha = plan.alpha;
//	Run_SetMotorAccl(100);
	speed_now = plan.speed;
	main_flag |= MAIN_FLAG_RUN;
}

void Plan_UIRoutine() {
	if(main_flag & MAIN_FLAG_RUN) {
		if(main_flag & MAIN_FLAG_NEXT) {
			GetAction(num_index);
			main_flag &= ~MAIN_FLAG_NEXT;
		}
		else {
			GetActionSequenceRoutine();
		}
	}
}

uint8_t retval;
void Plan_Main(float period) {
	if(main_flag & MAIN_FLAG_RUN) {
		if(main_flag & MAIN_FLAG_NEXT) {
//			Run_LineTracing(speed_now, period, plan.status_pid);

		}
		else {

			if(retval == 0) {
				Run_LineTracing(speed_now, period, plan.status_pid);
			}
			else if(retval == 2){
				sum_error = 0;
				counter_status = 0;
				action_status = 0;
				autoturn_stat = 0;
				speed_now = plan.speed;
				num_index++;
				if(num_index == plan.num_action) {
					Plan_Stop();
					return;
				}
				if(num_index >= plan.checkpoint[num_checkpoint+1]) {
					num_checkpoint++;
				}
				if(!GetActionSequence())
					main_flag |= MAIN_FLAG_NEXT;
			}
			else if(retval == 3){
				counter_status = 0;
				action_status = 2;
			}

			speed_now = (plan_active.unit_speed==0)?plan.speed:plan_active.unit_speed;
			if(action_status == 0) {
				switch(GET_LINE_COLOR_STAT(plan_active.status)) {
				case LINE_COLOR_STAT_BLACK: SEN_BlackLine(); break;
				case LINE_COLOR_STAT_WHITE: SEN_WhiteLine(); break;
				}
				action_status = 1;
			}

			if(action_status <= 1 && 				 \
					(plan_active.act != PLAN_STOP && \
					 plan_active.act != PLAN_NULL && \
					 plan_active.act != PLAN_JUMP)) {
				retval = Plan_ActionInit(plan_active, period, speed_now);
			}
			else {
				switch(plan_active.act) {
				case PLAN_STOP: 	retval = Plan_Stop(); 					break;
				case PLAN_LEFT: 	retval = Plan_Left(plan_active); 		break;
				case PLAN_RIGHT: 	retval = Plan_Right(plan_active); 		break;
				case PLAN_FORWARD: 	retval = Plan_Forward(plan_active, speed_now); 	break;
				case PLAN_FOLLOW: 	retval = Plan_Follow(plan_active); 		break;
				case PLAN_FOLLOW_L: retval = Plan_FollowLeft(plan_active); 	break;
				case PLAN_FOLLOW_R: retval = Plan_FollowRight(plan_active); break;
				case PLAN_INVERT: 	retval = Plan_Invert(plan_active); 		break;
				case PLAN_NEXT: 	retval = Plan_NextPlan(plan_active); 	break;
				case PLAN_NULL: 	retval = Plan_Null(plan_active); 		break;
				case PLAN_BACKWARD:	retval = Plan_Backward(plan_active, speed_now);	break;
				case PLAN_JUMP: 	retval = Plan_Jump(plan_active); 		break;
				case PLAN_IDLE: 	retval = Plan_Idle(plan_active); 		break;
				default: retval = 2;
				}
			}

		}
		plan_counter++;
	}
}

//typedef enum {
//	PLAN_INIT,
//	PLAN_WAIT,
//	PLAN_BOOST,
//	PLAN_EXEC,
//	PLAN_FINAL
//} PlanStatus_t;
//
//PlanStatus_t plan_status;
//
//void Plan_Main2(float period) {
//	if(main_flag & MAIN_FLAG_RUN) {
//		if(main_flag & MAIN_FLAG_NEXT) {
//		//			Run_LineTracing(speed_now, period, plan.status_pid);
//
//		}
//		else {
//			switch(plan_status) {
//			case PLAN_INIT: {
//
//			} break;
//			case PLAN_WAIT: {
//
//			} break;
//			case PLAN_BOOST: {
//
//			} break;
//			case PLAN_EXEC: {
//				Run_LineTracing(speed_now, period, plan.status_pid);
//			} break;
//			case PLAN_FINAL: {
//
//			} break;
//			}
//		}
//	}
//}
