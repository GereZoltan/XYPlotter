/*
 * MotorController_Task.cpp
 *
 *  Created on: Oct 1, 2019
 *      Author: Zoltan Gere
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "DigitalIoPin.h"

#include "ITM_write.h"
#include "MotorController_Task.h"

QueueHandle_t instructionQueue;

/**
 * MotorController_Task .cpp
 * <pre> void MotorControllerTask(void *pvParameters);
 *
 * MotorController_task: control the plotter hardware
 *
 * @brief	Reads 1 line from instruction queue and process it.
 *
 * @return    Nothing, function should not exit
 */
void MotorControllerTask(void *pvParameters) {
	Instruction_t command;

	vTaskDelay(100); /* wait until semaphores are created */

	while (1) {
		xQueueReceive(instructionQueue, (void *) &command, portMAX_DELAY);
		switch (command.cmd) {
		case M10:
			break;
		default:
			break;
		}
	}	// End infinite loop
}
