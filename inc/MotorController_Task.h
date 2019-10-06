/*
 * MotorController_Task.h
 *
 *  Created on: Oct 1, 2019
 *      Author: Zoltan Gere
 */

#ifndef MOTORCONTROLLER_TASK_H_
#define MOTORCONTROLLER_TASK_H_

typedef enum {
	M10, M11, M2, M1, M5, M4, G28, G1
} Instruction_E;

typedef struct {
	Instruction_E cmd;
	uint16_t arg1, arg2, arg3, arg4, arg5;
	int arg6, arg7;
	float Xcoord, Ycoord;
} Instruction_t;

/**
 * MotorController_Task .h
 * <pre> void MotorControllerTask(void *pvParameters);
 *
 * MotorController_task: control the plotter hardware
 *
 * @brief	Reads 1 line from instruction queue and process it.
 *
 * @return    Nothing, function should not exit
 */
void MotorControllerTask(void *pvParameters);

#endif /* MOTORCONTROLLER_TASK_H_ */
