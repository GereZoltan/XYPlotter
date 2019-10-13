/*
 * MotorController_Task.h
 *
 *  Created on: Oct 1, 2019
 *      Author: Zoltan Gere
 */

#ifndef MOTORCONTROLLER_TASK_H_
#define MOTORCONTROLLER_TASK_H_

#include "FreeRTOS.h"

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
