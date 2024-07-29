/*
 * plan.h
 *
 *  Created on: Jul 31, 2023
 *      Author: Zain Irsyad
 */

#include "main.h"

#ifndef INC_PLAN_H_
#define INC_PLAN_H_

#define DELAY_FACTOR	10
#define ENC_FACTOR	2000

#define PLAN_STOP		0
#define PLAN_LEFT		1
#define PLAN_RIGHT		2
#define PLAN_FORWARD	3
#define PLAN_FOLLOW		4
#define PLAN_FOLLOW_L	5
#define PLAN_FOLLOW_R	6
#define PLAN_INVERT		7
#define PLAN_NEXT		8
#define PLAN_NULL		9
#define PLAN_BACKWARD	10
#define PLAN_JUMP		11
#define PLAN_IDLE		12

/**
 * @brief	Ini adalah enumerasi untuk status counter
 */
typedef enum {
    COUNTER_STOP,
    COUNTER_RUN
} PlanCounterStatus_t;

/**
 * @brief	Ini adalah struktur data untuk couter yang akan digunakan di plan
 */
typedef struct {
    uint32_t counter;
    uint32_t counter_stamp;
    PlanCounterStatus_t status;
} PlanCounter_t;

/**
 * @brief	Ini adalah enumerasi untuk status plan secara global
 */
typedef enum {
	PLAN_STATUS_IDLE,
	PLAN_STATUS_INIT_RUN,
	PLAN_STATUS_EXECUTE,
	PLAN_STATUS_NEXT
} PlanStatus_t;

/**
 * @brief	Ini adalah enumerasi untuk status inisialisasi sebelum aksi
 */
typedef enum {
	ACTION_INIT_WAIT_TIME,
	ACTION_INIT_BOOST_TIME
} ActionInitStatus_t;

/**
 * @brief	Ini adalah enumerasi untuk status aksi saat ini
 */
typedef enum {
	ACTION_STATUS_IDLE,
	ACTION_STATUS_FIND,
	ACTION_STATUS_BRAKE_1,
	ACTION_STATUS_BRAKE_2,
	ACTION_STATUS_EXECUTE_1,
	ACTION_STATUS_EXECUTE_2,
	ACTION_STATUS_EXECUTE_3,
} ActionStatus_t;



void Plan_Start();
void Plan_UIRoutine();
void Plan_Main(float period);
uint8_t Plan_Stop();

#endif /* INC_PLAN_H_ */
