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
#include "Servo.h"
#include "Globals.h"
#include "Limits.h"

extern QueueHandle_t instructionQueue;
extern QueueHandle_t outputQueue;

volatile uint32_t RIT_count;
//volatile uint32_t RIT_countX;
//volatile uint32_t RIT_countY;
SemaphoreHandle_t sbRIT;

volatile bool directionX = false;
volatile bool phaseX = false;
volatile int32_t stepX;
volatile bool directionY = false;
volatile bool phaseY = false;
volatile int32_t stepY;

extern DigitalIoPin limit1;
extern DigitalIoPin limit2;
extern DigitalIoPin limit3;
extern DigitalIoPin limit4;

extern DigitalIoPin * LimitswitchYPos;
extern DigitalIoPin * LimitswitchYNeg;
extern DigitalIoPin * LimitswitchXPos;
extern DigitalIoPin * LimitswitchXNeg;

extern DigitalIoPin stepPinX;	// D10 - P0.27
extern DigitalIoPin dirPinX;	// D11 - P0.28
extern DigitalIoPin stepPinY;	// D8 - P0.24
extern DigitalIoPin dirPinY;	// D9 - P1.0

/**
 * @brief	RIT interrupt handler
 * @return	Nothing
 */
extern "C" {
void RIT_IRQHandler(void) {
// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
// Tell timer that we have processed the interrupt.
// Timer then removes the IRQ until next match occurs
	Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag

	if (RIT_count == 0) {
		Chip_RIT_Disable(LPC_RITIMER); // disable timer
// Give semaphore and set context switch flag if a higher priority task was woken up
		xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
	} else {

		if ((RIT_count % stepX) == 0) {
			dirPinX.write(directionX);
			stepPinX.write(TRUE);
//			RIT_countX--;
		} else {
			stepPinX.write(FALSE);
		}
		if ((RIT_count % stepY) == 0) {
			dirPinY.write(directionY);
			stepPinY.write(TRUE);
//			RIT_countY--;
		} else {
			stepPinY.write(FALSE);
		}
		RIT_count--;
	}
// End the ISR and (possibly) do a context switch
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}
}

/*
 * The following function sets up RIT interrupt at given interval and waits until count RIT interrupts have
 * occurred. Note that the actual counting is performed by the ISR and this function just waits on the semaphore.
 *
 * count: How many times the ISR runs
 * us = pps: Delta time between ISR runs
 * usec = 500000 / pps
 */
void RIT_start(int countX, int countY, int usX, int usY) {
	uint64_t cmp_value;
// Determine approximate compare value based on clock rate and passed interval
	cmp_value = (uint64_t) Chip_Clock_GetSystemClockRate() * (uint64_t) 10	//us
	/ 1000000;
// disable timer during configuration
	Chip_RIT_Disable(LPC_RITIMER);
//	RIT_countX = countX;
//	RIT_countY = countY;
	stepX = usX / 10;
	stepY = usY / 10;
	if (countX > countY) {
		RIT_count = countX * stepX;
	} else {
		RIT_count = countY * stepY;
	}
// enable automatic clear on when compare value==timer value
// this makes interrupts trigger periodically
	Chip_RIT_EnableCompClear(LPC_RITIMER);
// reset the counter
	Chip_RIT_SetCounter(LPC_RITIMER, 0);
	Chip_RIT_SetCompareValue(LPC_RITIMER, cmp_value);
// start counting
	Chip_RIT_Enable(LPC_RITIMER);
// Enable the interrupt signal in NVIC (the interrupt controller)
	NVIC_EnableIRQ(RITIMER_IRQn);
// wait for ISR to tell that we're done
	if (xSemaphoreTake(sbRIT, portMAX_DELAY) == pdTRUE) {
// Disable the interrupt signal in NVIC (the interrupt controller)
		NVIC_DisableIRQ(RITIMER_IRQn);
	} else {
// unexpected error
	}
}

void MoveToXY(float finishX, float finishY, bool relative) {


	float newPosX;
	float newPosY;
//	int32_t count;
	int32_t newTickPosX;
	int32_t newTickPosY;
	int32_t diffX;
	int32_t diffY;

	if (relative) {
		newPosX = currentPosX + finishX;
		newPosY = currentPosY + finishY;
	} else {
		newPosX = finishX;
		newPosY = finishY;
	}

	newTickPosX = (int32_t) (newPosX * changeRateX);
	newTickPosY = (int32_t) (newPosY * changeRateY);

	diffX = newTickPosX - currentTickPosX;
	diffY = newTickPosY - currentTickPosY;

	if (diffX > 0) {						// Move to positive direction
		directionX = PlotterConfiguration.stepperXDir;
	} else {
		directionX = !PlotterConfiguration.stepperXDir;
		diffX = diffX * -1;
	}

	if (diffY > 0) {						// Move to positive direction
		directionY = PlotterConfiguration.stepperYDir;
	} else {
		directionY = !PlotterConfiguration.stepperYDir;
		diffY = diffY * -1;
	}

	//count = (diffX >= diffY) ? (diffX) : (diffY);

	if (diffX > diffY) {
		RIT_start(diffX, diffY, 500, 500000 / ((diffY * 1000) / diffX));
	} else {
		RIT_start(diffX, diffY, 500000 / ((diffX * 1000) / diffY), 500);
	}

	currentPosX = newPosX;
	currentPosY = newPosY;
	currentTickPosX = newTickPosX;
	currentTickPosY = newTickPosY;
}

/**
 * MotorController_Task .cpp
 * <pre> void MotorControllerTask(void *pvParameters);
 *
 * MotorController_task: control the plotter hardware
 * Initialize plotter
 * Read received instructions and execute them
 *
 * @brief	Reads 1 line from instruction queue and process it.
 *
 * @return    Nothing, function should not exit
 */
void MotorControllerTask(void *pvParameters) {
//	DigitalIoPin stepPinX(0, 27, DigitalIoPin::output, false);	// D10 - P0.27
//	DigitalIoPin dirPinX(0, 28, DigitalIoPin::output, false);	// D11 - P0.28
//	DigitalIoPin stepPinY(0, 24, DigitalIoPin::output, false);	// D8 - P0.24
//	DigitalIoPin dirPinY(1, 0, DigitalIoPin::output, false);	// D9 - P1.0

//	const char * M10answer =
//			"M10 XY 500 500 0.00 0.00 A0 B0 H0 S80 U160 D90\r\rOK\r\n";
//	const char * M11answer = "M11 1 1 1 1\r\nOK\r\n";
	const char * OKanswer = "OK\r\n";
	char msg[UART_QUEUE_ELEMENT_LENGTH];

	Instruction_t command;
	Servo pen(Servo::pen);
	Servo laser(Servo::laser);
	laser.SetLaserPower(0);

	sbRIT = xSemaphoreCreateBinary();

	vTaskDelay(100); /* wait until semaphores are created */
	ITM_write("MotorControl started\r\n");

	InitPlotter();	// Set up plotter initial position and find limit switches

	while (1) {
		xQueueReceive(instructionQueue, (void *) &command, portMAX_DELAY);
		ITM_write("MC Instruction received\r\n");
		switch (command.cmd) {
		case M10:
			snprintf(msg, UART_QUEUE_ELEMENT_LENGTH,
					"M10 %s %u %u 0.00 0.00 %s %s H0 S%u U%u D%u\r\rOK\r\n",
					PlotterConfiguration.plotterType,
					PlotterConfiguration.plotAreaY,
					PlotterConfiguration.plotAreaX,
					PlotterConfiguration.stepperXDir ? "A1" : "A0",
					PlotterConfiguration.stepperYDir ? "B1" : "B0",
					PlotterConfiguration.plottingSpeed,
					PlotterConfiguration.penUp, PlotterConfiguration.penDown);
			ITM_write(msg);
			xQueueSend(outputQueue, (void * ) msg, portMAX_DELAY);
			break;
		case M11:
			snprintf(msg, UART_QUEUE_ELEMENT_LENGTH,
					"M11 %u %u %u %u\r\nOK\r\n",
					(uint8_t) !LimitswitchXNeg->read(),
					(uint8_t) !LimitswitchXPos->read(),
					(uint8_t) !LimitswitchYNeg->read(),
					(uint8_t) !LimitswitchYPos->read());
			ITM_write(msg);
			xQueueSend(outputQueue, (void * ) msg, portMAX_DELAY);
			break;
		case M2:				// Save pen position
			ITM_write("M2\r\n");
			PlotterConfiguration.penUp = (uint8_t) command.arg1;
			PlotterConfiguration.penDown = (uint8_t) command.arg2;
			xQueueSend(outputQueue, (void * ) OKanswer, portMAX_DELAY);
			break;
		case M1:				// Set pen position
			ITM_write("M1\r\n");
			pen.SetPenPosition((uint8_t) command.arg1);
			vTaskDelay(pdMS_TO_TICKS(250));	// Allow some time for the servo before continuing
			xQueueSend(outputQueue, (void * ) OKanswer, portMAX_DELAY);
			break;
		case M5:				// Save plotter settings
			ITM_write("M5\r\n");
			PlotterConfiguration.stepperXDir = (bool) command.arg1;
			PlotterConfiguration.stepperYDir = (bool) command.arg2;
			PlotterConfiguration.plotAreaY = command.arg3;
			PlotterConfiguration.plotAreaX = command.arg4;
			PlotterConfiguration.plottingSpeed = (uint8_t) command.arg5;

			changeRateX = (float) MaxXAxisTick / PlotterConfiguration.plotAreaX;
			changeRateY = (float) MaxYAxisTick / PlotterConfiguration.plotAreaY;
			xQueueSend(outputQueue, (void * ) OKanswer, portMAX_DELAY);
			break;
		case M4:				// Set laser power
			ITM_write("M4\r\n");
			if (command.arg1 == 0) {
				laser.SetLaserPower((uint8_t) 0);
				laser.SetLaser(FALSE);
			} else {
				laser.SetLaserPower((uint8_t) command.arg1);
				laser.SetLaser(TRUE);
			}
			xQueueSend(outputQueue, (void * ) OKanswer, portMAX_DELAY);
			break;
		case G28:				// Go to origin
			ITM_write("G28\r\n");
			MoveToXY(0.0f, 0.0f, FALSE);
			xQueueSend(outputQueue, (void * ) OKanswer, portMAX_DELAY);
			break;
		case G1:				// Go to position
			ITM_write("G1\r\n");
			MoveToXY(command.Xcoord, command.Ycoord, (bool) command.arg1);// arg1 0=absolute 1=relative
			xQueueSend(outputQueue, (void * ) OKanswer, portMAX_DELAY);
			break;
		default:
			ITM_write("Unknown instruction\r\n");
			break;
		}
	}	// End infinite loop
}
