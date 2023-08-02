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

#define LEFT_SENSOR(x)  SENSOR_NUM-1-x
#define RIGHT_SENSOR(x) x

#define LEFT_ALIGN_SEN 0x00
#define RIGHT_ALIGN_SEN 0x01
#define LR_ALIGN_SEN 0x02

uint32_t plan_counter;
uint32_t plan_last_counter;
uint32_t plan_last_counter2;
int16_t plan_last_enc;

uint8_t counter_status = 0;
uint8_t action_status = 0;
//uint8_t acc_status = 0;

void Plan_SetCounter() {
	plan_last_counter = plan_counter;
}

uint8_t Plan_CheckCounterValue(uint32_t value) {
	return (plan_counter - plan_last_counter) >= value*DELAY_FACTOR;
}

uint32_t Plan_CounterValue() {
	return (plan_counter - plan_last_counter);
}

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

uint8_t Plan_Left(Action_typedef a) {
	if(Plan_CheckSensor(a.sen_trig, LEFT_ALIGN_SEN) && counter_status == 0) {
			Plan_SetCounter();
			plan_last_enc = MOTOR_L.enc_cnt;
			counter_status = 1;
	}
	if(counter_status == 1) {
		Run_SetMotorSpeed(a.reverse_speed*plan.turn_speed*0.1, a.forward_speed*plan.turn_speed*0.1);
		switch(a.act_mode) {
		case 0:
			if(Plan_CheckCounterValue(a.act_value)) return 2;
			break;
		case 1:
			if(Plan_CheckCounterValue(a.act_value))
				if(Plan_CheckSensor(5, LEFT_ALIGN_SEN)) return 2;
			break;
		case 2:
			if(MOTOR_L.enc_cnt - plan_last_enc > a.act_value) return 2;
			break;
		}
		return 1;
	}
	return 0;
}

uint8_t Plan_Right(Action_typedef a) {
	if(Plan_CheckSensor(a.sen_trig, RIGHT_ALIGN_SEN) && counter_status == 0) {
			Plan_SetCounter();
			plan_last_enc = MOTOR_R.enc_cnt;
			counter_status = 1;
	}
	if(counter_status == 1) {
		Run_SetMotorSpeed(a.forward_speed*plan.turn_speed*0.1, a.reverse_speed*plan.turn_speed*0.1);
		switch(a.act_mode) {
		case 0:
			if(Plan_CheckCounterValue(a.act_value)) return 2;
			break;
		case 1:
			if(Plan_CheckCounterValue(a.act_value))
				if(Plan_CheckSensor(5, RIGHT_ALIGN_SEN)) return 2;
			break;
		case 2:
			if(MOTOR_R.enc_cnt - plan_last_enc > a.act_value) return 2;
			break;
		}
		return 1;
	}
	return 0;
}

uint8_t Plan_Forward(Action_typedef a, uint8_t _speed) {
	if(Plan_CheckSensor(a.sen_trig, LR_ALIGN_SEN) && counter_status == 0) {
			Plan_SetCounter();
			plan_last_enc = MOTOR_L.enc_cnt;
			counter_status = 1;
	}
	if(counter_status == 1) {
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
			if(MOTOR_L.enc_cnt - plan_last_enc > a.act_value) return 2;
			break;
		}
		return 1;
	}
	return 0;
}

uint8_t Plan_Backward(Action_typedef a, uint8_t _speed) {
	if(Plan_CheckSensor(a.sen_trig, LR_ALIGN_SEN) && counter_status == 0) {
			Plan_SetCounter();
			plan_last_enc = MOTOR_L.enc_cnt;
			counter_status = 1;
	}
	if(counter_status == 1) {
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
			if(MOTOR_L.enc_cnt - plan_last_enc > a.act_value) return 2;
			break;
		}
		return 1;
	}
	return 0;
}

uint8_t Plan_Follow(Action_typedef a) {
	if(counter_status == 0) {
		Plan_SetCounter();
		plan_last_enc = MOTOR_L.enc_cnt;
		counter_status = 1;
	}
	switch(a.act_mode) {
	case 0:
		if(Plan_CheckCounterValue(a.act_value) && a.act_value!=0) return 2;
		break;
	case 2:
		if(MOTOR_L.enc_cnt - plan_last_enc > a.act_value) return 2;
		break;
	default: return 2;
	}
	return 0;
}

uint8_t Plan_FollowLeft(Action_typedef a) {
	if(counter_status == 0) {
		Plan_SetCounter();
		plan_last_enc = MOTOR_L.enc_cnt;
		counter_status = 1;
	}
	if(Plan_CheckSensor(a.sen_trig, LEFT_ALIGN_SEN) && counter_status == 1) {
		counter_status = 2;
	}
	if(counter_status == 2) {
		Run_SetMotorSpeed(a.reverse_speed*plan.turn_speed*0.1, a.forward_speed*plan.turn_speed*0.1);
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
		if(MOTOR_L.enc_cnt - plan_last_enc > a.act_value) return 2;
		break;
	default: return 2;
	}
	return 0;
}

uint8_t Plan_FollowRight(Action_typedef a) {
	if(counter_status == 0) {
		Plan_SetCounter();
		plan_last_enc = MOTOR_R.enc_cnt;
		counter_status = 1;
	}
	if(Plan_CheckSensor(a.sen_trig, RIGHT_ALIGN_SEN) && counter_status == 1) {
		counter_status = 2;
	}
	if(counter_status == 2) {
		Run_SetMotorSpeed(a.forward_speed*plan.turn_speed*0.1, a.reverse_speed*plan.turn_speed*0.1);
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
		if(MOTOR_R.enc_cnt - plan_last_enc > a.act_value) return 2;
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

uint8_t Plan_ActionInit(Action_typedef a, float period, uint8_t sp) {
	static uint8_t stat = 0;
	uint8_t _speed = 0;
	if(counter_status == 0) {
		Plan_SetCounter();
		counter_status = 1;
	}

	if(Plan_CheckCounterValue(a.wait_time) && stat == 0) {
		counter_status = 0;
		stat = 1;
		return 0;
	}

	if(Plan_CheckCounterValue(a.boost_time) && stat == 1) {
		counter_status = 0;
		acc_status = 0;
		stat = 0;
		return 3;
	}

//	if(Plan_CheckCounterValue(plan.acc) && stat == 1 && acc_status == 1) {
//		acc_status = 2;
//	}
//
//	if(acc_status == 0) {
//		_speed = sp;
//		acc_status = 1;
//	}
//	else if(acc_status == 1) {
//		if(sp > a.boost_speed) {
//			_speed = (sp-a.boost_speed)*(float)(Plan_CounterValue()/(float)plan.acc);
//		}
//		else if(sp < a.boost_speed){
//			_speed = (a.boost_speed-sp)*(float)(Plan_CounterValue()/(float)plan.acc);
//		}
//	}
//	else if(acc_status == 2) {
//		_speed = a.boost_speed;
//	}
//	Run_LineTracing(_speed, period, 0);
	Run_LineTracing(a.boost_speed, period, 0);
	return 1;
}


uint8_t Plan_Stop() {
	Run_SetMotorSpeed(0, 0);
	SEN_BlackLine();
	num_checkpoint = 0;
	num_index = plan.checkpoint[num_checkpoint];
	main_flag &=~ MAIN_FLAG_RUN;
	return 1;
}

void Plan_Start() {
	num_index = plan.checkpoint[num_checkpoint];
	GetActionSequenceInit(num_index);
	action_status = 0;
	plan_counter = 0;
	plan_last_counter = 0;
	kp = plan.kpid[0];
	ki = plan.kpid[1];
	kd = plan.kpid[2];
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

void Plan_Main(float period) {
	if(main_flag & MAIN_FLAG_RUN) {
		if(main_flag & MAIN_FLAG_NEXT) {
			Run_LineTracing(plan.speed, period, 0);
		}
		else {
			uint8_t retval;
			uint8_t speed_now = (plan_active.unit_speed==0)?plan.speed:plan_active.unit_speed;
			if(action_status == 0 && (plan_active.act != PLAN_STOP || plan_active.act != PLAN_NULL)) {
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
				default: retval = 2;
				}
			}
			if(retval == 0) {
				Run_LineTracing(speed_now, period, 0);
			}
			else if(retval == 2){
				counter_status = 0;
				action_status = 0;
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
				action_status = 1;
			}
		}
		plan_counter++;
	}
}
