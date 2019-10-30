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

#include <cmath>

// Switch between interrupt drive motor control or simple but slow(er) motor control
#define IRQPLOT

extern QueueHandle_t instructionQueue;
extern QueueHandle_t outputQueue;

volatile bool directionX = false;
volatile bool phaseX = false;
volatile int32_t stepX;
volatile bool directionY = false;
volatile bool phaseY = false;
volatile int32_t stepY;

//extern DigitalIoPin limit1;
//extern DigitalIoPin limit2;
//extern DigitalIoPin limit3;
//extern DigitalIoPin limit4;

extern DigitalIoPin * LimitswitchYPos;
extern DigitalIoPin * LimitswitchYNeg;
extern DigitalIoPin * LimitswitchXPos;
extern DigitalIoPin * LimitswitchXNeg;

#ifdef IRQPLOT
volatile uint32_t RIT_count;
SemaphoreHandle_t sbRIT;

volatile int32_t newTickPosX;
volatile int32_t newTickPosY;
volatile int32_t diffX;
volatile int32_t diffY;
volatile int32_t err, err2;

DigitalIoPin stepPinY(0, 27, DigitalIoPin::output, false);	// D10 - P0.27
DigitalIoPin dirPinY(0, 28, DigitalIoPin::output, false);	// D11 - P0.28
DigitalIoPin stepPinX(0, 24, DigitalIoPin::output, false);	// D8 - P0.24
DigitalIoPin dirPinX(1, 0, DigitalIoPin::output, false);	// D9 - P1.0

/**
 * @brief	RIT interrupt handler
 * Calculate error and move stepper motor when necessary
 * Keeps the record of plotter position
 * @return	Nothing
 */
extern "C" {
void RIT_IRQHandler(void) {
// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
// Tell timer that we have processed the interrupt.
// Timer then removes the IRQ until next match occurs
	Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag

	if ((currentTickPosX == newTickPosX) && (currentTickPosY == newTickPosY)) {
		Chip_RIT_Disable(LPC_RITIMER); // disable timer
// Give semaphore and set context switch flag if a higher priority task was woken up
		xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
	} else {
		err2 = err << 1;
		if (err2 >= diffY && !phaseX) {
			err += diffY;
			currentTickPosX += stepX;
			// Move X-stepper 1 tick
			if (directionX && (currentTickPosX < MaxXAxisTick)) {
				dirPinX.write(directionX);
				stepPinX.write(TRUE);
			}
			if (!directionX && (currentTickPosX > 0)) {
				dirPinX.write(directionX);
				stepPinX.write(TRUE);
			}
		} else {
			stepPinX.write(FALSE);
		}
		if (err2 <= diffX && !phaseY) {
			err += diffX;
			currentTickPosY += stepY;
			// Move Y-stepper 1 tick
			if (directionY && (currentTickPosY < MaxYAxisTick)) {
				dirPinY.write(directionY);
				stepPinY.write(TRUE);
			}
			if (!directionY && (currentTickPosY > 0)) {
				dirPinY.write(directionY);
				stepPinY.write(TRUE);
			}
		} else {
			stepPinY.write(FALSE);
		}
		phaseX = !phaseX;
		phaseY = !phaseY;
	}

// End the ISR and (possibly) do a context switch
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}
}

/**
 * MotorController_Task .cpp
 * <pre> void RIT_start(int32_t us);
 * The following function sets up RIT interrupt at given interval and waits until count RIT interrupts have
 * occurred. Note that the actual counting is performed by the ISR and this function just waits on the semaphore.
 *
 * count: How many times the ISR runs
 * us = pps: Delta time between ISR runs
 * usec = 500000 / pps
 */
void RIT_start(int32_t us) {
	uint64_t cmp_value;
// Determine approximate compare value based on clock rate and passed interval
	cmp_value = (uint64_t) Chip_Clock_GetSystemClockRate() * (uint64_t) us
			/ 1000000;
// disable timer during configuration
	Chip_RIT_Disable(LPC_RITIMER);
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

/**
 * MotorController_Task .cpp
 * <pre> void MoveToXY(float finishX, float finishY, bool relative);
 * @brief Make coordinate conversion and call the interrupt set up function
 *
 * @return	No return value
 */
void MoveToXY(float finishX, float finishY, bool relative) {
	float newPosX;
	float newPosY;

	// Process absolute/relative offsets
	if (relative) {
		newPosX = currentPosX + finishX;
		newPosY = currentPosY + finishY;
	} else {
		newPosX = finishX;
		newPosY = finishY;
	}

	// Convert float difference to Tick/Pulse count
	newTickPosX = (int32_t) round(newPosX * changeRateX);
	newTickPosY = (int32_t) round(newPosY * changeRateX);
//	newTickPosY = (int32_t) (newPosY * changeRateY);

	// Actual difference in Ticks
	diffX = newTickPosX - currentTickPosX;
	diffY = newTickPosY - currentTickPosY;

	if (diffX > 0) {						// X axis moves to positive direction
		directionX = PlotterConfiguration.stepperXDir;
		stepX = 1;
	} else {								// X axis moves to negative direction
		directionX = !PlotterConfiguration.stepperXDir;
		diffX = diffX * -1;
		stepX = -1;
	}

	if (diffY > 0) {						// Y axis moves to positive direction
		directionY = PlotterConfiguration.stepperYDir;
		diffY = diffY * -1;
		stepY = 1;
	} else {								// Y axis moves to negative direction
		directionY = !PlotterConfiguration.stepperYDir;
		stepY = -1;
	}

	err = diffX + diffY;

	// Calculate interrupt frequency from stored plotter speed values
	if (PlotterConfiguration.plottingSpeed == 0) {
		RIT_start(500000 / ((int32_t) MaxPPS / 100));
	} else {
		RIT_start(500000 / ((int32_t) MaxPPS * PlotterConfiguration.plottingSpeed / 100));
	}

	// Save new positions
	currentPosX = newPosX;
	currentPosY = newPosY;
	currentTickPosX = newTickPosX;
	currentTickPosY = newTickPosY;
}

#else
/**
 * MotorController_Task .cpp
 * <pre> void MoveToXY(float finishX, float finishY, bool relative);
 * @brief Make coordinate conversion
 * Direct controll version
 * No interrupt used
 *
 * @return	No return value
 */
void MoveToXY(float finishX, float finishY, bool relative) {
	DigitalIoPin stepPinY(0, 27, DigitalIoPin::output, false);	// D10 - P0.27
	DigitalIoPin dirPinY(0, 28, DigitalIoPin::output, false);// D11 - P0.28
	DigitalIoPin stepPinX(0, 24, DigitalIoPin::output, false);// D8 - P0.24
	DigitalIoPin dirPinX(1, 0, DigitalIoPin::output, false);// D9 - P1.0

	float newPosX;
	float newPosY;
//	int32_t count;
	int32_t newTickPosX;
	int32_t newTickPosY;
	int32_t diffX;
	int32_t diffY;
	int32_t err, err2;

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
		stepX = 1;
	} else {
		directionX = !PlotterConfiguration.stepperXDir;
		diffX = diffX * -1;
		stepX = -1;
	}

	if (diffY > 0) {						// Move to positive direction
		directionY = PlotterConfiguration.stepperYDir;
		diffY = diffY * -1;
		stepY = 1;
	} else {
		directionY = !PlotterConfiguration.stepperYDir;
		stepY = -1;
	}

	err = diffX + diffY;

	while (1) {
		if ((currentTickPosX == newTickPosX) && (currentTickPosY == newTickPosY)) {
			break;
		}
		err2 = err << 1;
		if (err2 >= diffY) {
			err += diffY;
			currentTickPosX += stepX;
			// Move X-stepper 1 tick
			if ( directionX && (currentTickPosX < MaxXAxisTick) ) {
				dirPinX.write(directionX);
				stepPinX.write(TRUE);
			}
			if ( !directionX && (currentTickPosX > 0) ) {
				dirPinX.write(directionX);
				stepPinX.write(TRUE);
			}
		}
		if (err2 <= diffX) {
			err += diffX;
			currentTickPosY += stepY;
			// Move Y-stepper 1 tick
			if ( directionY && (currentTickPosY < MaxYAxisTick) ) {
				dirPinY.write(directionY);
				stepPinY.write(TRUE);
			}
			if ( !directionY && (currentTickPosY > 0) ) {
				dirPinY.write(directionY);
				stepPinY.write(TRUE);
			}
		}
		vTaskDelay(1);
		stepPinX.write(FALSE);
		stepPinY.write(FALSE);
		vTaskDelay((TickType_t) (11 - (PlotterConfiguration.plottingSpeed / 10)));
	}

	currentPosX = newPosX;
	currentPosY = newPosY;
	currentTickPosX = newTickPosX;
	currentTickPosY = newTickPosY;
}

#endif

//plotLine(int x0, int y0, int x1, int y1)
//    dx =  abs(x1-x0);
//    sx = x0<x1 ? 1 : -1;
//    dy = -abs(y1-y0);
//    sy = y0<y1 ? 1 : -1;
//    err = dx+dy;  /* error value e_xy */
//    while (true)   /* loop */
//        if (x0==x1 && y0==y1) break;
//        e2 = 2*err;
//        if (e2 >= dy)
//            err += dy; /* e_xy+e_x > 0 */
//            x0 += sx;
//        end if
//        if (e2 <= dx) /* e_xy+e_y < 0 */
//            err += dx;
//            y0 += sy;
//        end if
//    end while

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
//	const char * M10answer =
//			"M10 XY 500 500 0.00 0.00 A0 B0 H0 S80 U160 D90\r\rOK\r\n";
//	const char * M11answer = "M11 1 1 1 1\r\nOK\r\n";
	const char * OKanswer = "OK\r\n";
	char msg[UART_QUEUE_ELEMENT_LENGTH];

	Instruction_t command;
	Servo pen(Servo::pen);
	Servo laser(Servo::laser);
	laser.SetLaser(FALSE);
	pen.SetPenPosition(PlotterConfiguration.penUp);

#ifdef IRQPLOT
	sbRIT = xSemaphoreCreateBinary();
#endif

	vTaskDelay(pdMS_TO_TICKS(2000)); /* wait until semaphores are created */
	ITM_write("MotorControl started\r\n");

	InitPlotter();	// Set up plotter initial position and find limit switches

	while (1) {
		xQueueReceive(instructionQueue, (void *) &command, portMAX_DELAY);
//		ITM_write("MC Instruction received\r\n");
		switch (command.cmd) {
		case M10:				// mDraw query plotter specifications
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
		case M11:				// mDraw query limit switch statuses
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
			//ITM_write("M1\r\n");
			pen.SetPenPosition((uint8_t) command.arg1);
			vTaskDelay(pdMS_TO_TICKS(500));	// Allow some time for the servo before continuing
			snprintf(msg, UART_QUEUE_ELEMENT_LENGTH, "Pen: %d\r\n",
					command.arg1);
			ITM_write(msg);
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
			//ITM_write("M4\r\n");
			if (command.arg1 == 0) {
				laser.SetLaserPower((uint8_t) 0);
				laser.SetLaser(FALSE);
			} else {
				laser.SetLaserPower((uint8_t) command.arg1);
				laser.SetLaser(TRUE);
			}
			snprintf(msg, UART_QUEUE_ELEMENT_LENGTH, "Laser: %d\r\n",
					command.arg1);
			ITM_write(msg);
			xQueueSend(outputQueue, (void * ) OKanswer, portMAX_DELAY);
			break;
		case G28:				// Go to origin
			//ITM_write("G28\r\n");
			MoveToXY(0.0f, 0.0f, FALSE);
			snprintf(msg, UART_QUEUE_ELEMENT_LENGTH, "X: %ld  Y: %ld\r\n",
					currentTickPosX, currentTickPosY);
			ITM_write(msg);
			xQueueSend(outputQueue, (void * ) OKanswer, portMAX_DELAY);
			break;
		case G1:				// Go to position
			//ITM_write("G1\r\n");
			MoveToXY(command.Xcoord, command.Ycoord, (bool) command.arg1);// arg1 0=absolute 1=relative
			snprintf(msg, UART_QUEUE_ELEMENT_LENGTH, "X: %ld  Y: %ld End: %f %f\r\n",
					currentTickPosX, currentTickPosY, command.Xcoord, command.Ycoord);
			ITM_write(msg);
			xQueueSend(outputQueue, (void * ) OKanswer, portMAX_DELAY);
			break;
		default:
			ITM_write("Unknown instruction\r\n");
			break;
		}
	}	// End infinite loop
}
