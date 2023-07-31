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

uint32_t plan_counter;
uint32_t plan_last_counter;
uint32_t plan_last_counter2;
int16_t plan_last_enc;

uint8_t counter_status = 0;
uint8_t action_status = 0;

void Plan_SetCounter() {
	plan_last_counter = plan_counter;
}

uint8_t Plan_CheckCounterValue(uint32_t value) {
	return (plan_counter - plan_last_counter) >= value*DELAY_FACTOR;
}

uint8_t Plan_CheckSensor(uint8_t s) {
	if(s == 0) return 1;
	else {
		s-=1;
		return sensor_disp_status[s] > 0;
	}
}

uint8_t Plan_Left(Action_typedef a) {
	if(Plan_CheckSensor(LEFT_SENSOR(a.sen_trig)) && counter_status == 0) {
			Plan_SetCounter();
			plan_last_enc = MOTOR_L.enc_cnt;
			counter_status = 1;
	}
	if(counter_status == 1) {
		Run_SetMotorSpeed(a.reverse_speed*plan.turn_speed, a.forward_speed*plan.turn_speed);
		switch(a.act_mode) {
		case 0:
			if(Plan_CheckCounterValue(a.act_value)) return 2;
			break;
		case 1:
			if(Plan_CheckCounterValue(a.act_value))
				if(Plan_CheckSensor(LEFT_SENSOR(5))) return 2;
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
	if(Plan_CheckSensor(RIGHT_SENSOR(a.sen_trig)) && counter_status == 0) {
			Plan_SetCounter();
			plan_last_enc = MOTOR_R.enc_cnt;
			counter_status = 1;
	}
	if(counter_status == 1) {
		Run_SetMotorSpeed(a.forward_speed*plan.turn_speed, a.reverse_speed*plan.turn_speed);
		switch(a.act_mode) {
		case 0:
			if(Plan_CheckCounterValue(a.act_value)) return 2;
			break;
		case 1:
			if(Plan_CheckCounterValue(a.act_value))
				if(Plan_CheckSensor(RIGHT_SENSOR(5))) return 2;
			break;
		case 2:
			if(MOTOR_R.enc_cnt - plan_last_enc > a.act_value) return 2;
			break;
		}
		return 1;
	}
	return 0;
}

uint8_t Plan_Forward(Action_typedef a) {
	if(Plan_CheckSensor(LEFT_SENSOR(a.sen_trig)) && counter_status == 0) {
			Plan_SetCounter();
			plan_last_enc = MOTOR_L.enc_cnt;
			counter_status = 1;
	}
	if(counter_status == 1) {
		Run_SetMotorSpeed(plan.speed, plan.speed);
		switch(a.act_mode) {
		case 0:
			if(Plan_CheckCounterValue(a.act_value)) return 2;
			break;
		case 1:
			if(Plan_CheckCounterValue(a.act_value))
				if(Plan_CheckSensor(LEFT_SENSOR(7))) return 2;
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
	if(Plan_CheckSensor(LEFT_SENSOR(a.sen_trig)) && counter_status == 1) {
		counter_status = 2;
	}
	if(counter_status == 2) {
		Run_SetMotorSpeed(a.reverse_speed*plan.turn_speed, a.forward_speed*plan.turn_speed);
		if(Plan_CheckSensor(LEFT_SENSOR(5)) > 0) {
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
	if(Plan_CheckSensor(RIGHT_SENSOR(a.sen_trig)) && counter_status == 1) {
		counter_status = 2;
	}
	if(counter_status == 2) {
		Run_SetMotorSpeed(a.forward_speed*plan.turn_speed, a.reverse_speed*plan.turn_speed);
		if(Plan_CheckSensor(RIGHT_SENSOR(5)) > 0) {
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
	if(Plan_CheckSensor(LEFT_SENSOR(a.sen_trig))) {
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

uint8_t Plan_ActionInit(Action_typedef a, float period) {
	static uint8_t stat = 0;
	if(counter_status == 0) {
		Plan_SetCounter();
		counter_status = 1;
	}

	if(Plan_CheckCounterValue(a.wait_time) && stat == 0) {
		counter_status = 0;
		stat = 1;
		return 0;
	}

	Run_LineTracing(a.boost_speed, period, 0);

	if(Plan_CheckCounterValue(a.boost_time) && stat == 1) {
		counter_status = 0;
		stat = 0;
		return 3;
	}
	return 0;
}


uint8_t Plan_Stop() {
	Run_SetMotorSpeed(0, 0);
	SEN_BlackLine();
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
	}
}

void Plan_Main(float period) {
	if(main_flag & MAIN_FLAG_RUN) {
		if(main_flag & MAIN_FLAG_NEXT) {
			Run_LineTracing(plan.speed, period, 0);
		}
		else {
			uint8_t retval;
			if(action_status == 0 && plan_active.act != 0) {
				retval = Plan_ActionInit(plan_active, period);
			}
			else {
				switch(plan_active.act) {
				case PLAN_STOP: 	retval = Plan_Stop(); 					break;
				case PLAN_LEFT: 	retval = Plan_Left(plan_active); 		break;
				case PLAN_RIGHT: 	retval = Plan_Right(plan_active); 		break;
				case PLAN_FORWARD: 	retval = Plan_Forward(plan_active); 	break;
				case PLAN_FOLLOW: 	retval = Plan_Follow(plan_active); 		break;
				case PLAN_FOLLOW_L: retval = Plan_FollowLeft(plan_active); 	break;
				case PLAN_FOLLOW_R: retval = Plan_FollowRight(plan_active); break;
				case PLAN_INVERT: 	retval = Plan_Invert(plan_active); 		break;
				case PLAN_NEXT: 	retval = Plan_NextPlan(plan_active); 	break;
				case PLAN_NULL: 	retval = Plan_Null(plan_active); 		break;
				default: retval = 2;
				}
			}
			if(retval == 0) {
				Run_LineTracing(plan.speed, period, 0);
			}
			else if(retval == 2){
				counter_status = 0;
				action_status = 0;
				num_index++;
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
