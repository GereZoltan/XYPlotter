/*
 * limits.cpp
 *
 *  Created on: Oct 3, 2019
 *      Author: Ville Vainio
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "Limits.h"
#include "Globals.h"
#include "DigitalIoPin.h"
#include "ITM_write.h"

DigitalIoPin limit1(1, 3, DigitalIoPin::pullup, true);
DigitalIoPin limit2(0, 0, DigitalIoPin::pullup, true);
DigitalIoPin limit3(0, 9, DigitalIoPin::pullup, true);
DigitalIoPin limit4(0, 29, DigitalIoPin::pullup, true);

DigitalIoPin * LimitswitchYPos = NULL;
DigitalIoPin * LimitswitchYNeg = NULL;
DigitalIoPin * LimitswitchXPos = NULL;
DigitalIoPin * LimitswitchXNeg = NULL;

DigitalIoPin stepPinX(0, 27, DigitalIoPin::output, false);	// D10 - P0.27
DigitalIoPin dirPinX(0, 28, DigitalIoPin::output, false);	// D11 - P0.28
DigitalIoPin stepPinY(0, 24, DigitalIoPin::output, false);	// D8 - P0.24
DigitalIoPin dirPinY(1, 0, DigitalIoPin::output, false);	// D9 - P1.0

SemaphoreHandle_t limitSwitchSignal;

float changeRateX;					// Float-to-Tick conversion ratio
float changeRateY;					// Float-to-Tick conversion ratio
float currentPosX;
float currentPosY;
int32_t currentTickPosX;
int32_t currentTickPosY;

uint32_t MaxXAxisTick;					// Total number of steps on X-axis
uint32_t MaxYAxisTick;					// Total number of steps on Y-axis

/*
 * Limit switches are grounding switches. They read one when the switch is open (= not at a limit).
 *
 * Automatic printable area and limit switch detection
 * Software must be able to detect which limit switch is connected which axis automatically
 * to prevent swapper connectors from causing problems
 *
 * Enforcement of limits (=always stop at switch, never hit an edge)
 */

void readLimitSwitchesTask(void *pvParameters) {
	while (1) {
		if (limit1.read() || limit2.read() || limit3.read() || limit4.read()) {
			xSemaphoreGive(limitSwitchSignal);
			Board_LED_Set(3, TRUE);
		} else {
			Board_LED_Set(3, FALSE);
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

// Init X-axis
void InitXAxis() {
	int maxPos = 0;

	// Move to Positive direction - No counting
	ITM_write("X+ dir");
	dirPinX.write(PlotterConfiguration.stepperXDir);
	xSemaphoreTake(limitSwitchSignal, 0);
	while ((xSemaphoreTake(limitSwitchSignal, 0)) != pdTRUE) {
		stepPinX.write(TRUE);
		vTaskDelay(1);
		stepPinX.write(FALSE);
		vTaskDelay(1);
	}
	// X+ switch reached
	ITM_write(".");
	if (limit1.read()) {
		LimitswitchXPos = &limit1;
	} else if (limit2.read()) {
		LimitswitchXPos = &limit2;
	} else if (limit3.read()) {
		LimitswitchXPos = &limit3;
	} else if (limit4.read()) {
		LimitswitchXPos = &limit4;
	}
	// Move away from switch
	ITM_write(".\r\n");
	dirPinX.write(!PlotterConfiguration.stepperXDir);
	xSemaphoreTake(limitSwitchSignal, 0);
	while ((xSemaphoreTake(limitSwitchSignal, 0)) == pdTRUE) {
		stepPinX.write(TRUE);
		vTaskDelay(1);
		stepPinX.write(FALSE);
		vTaskDelay(1);
	}
	for (int i = 0; i < 50; i++) {
		stepPinX.write(TRUE);
		vTaskDelay(1);
		stepPinX.write(FALSE);
		vTaskDelay(1);
	}

	// Move to Negative direction - Counting
	ITM_write("X- dir\r\n");
	dirPinX.write(!PlotterConfiguration.stepperXDir);
	xSemaphoreTake(limitSwitchSignal, 0);
	while ((xSemaphoreTake(limitSwitchSignal, 0)) != pdTRUE) {
		stepPinX.write(TRUE);
		vTaskDelay(1);
		stepPinX.write(FALSE);
		vTaskDelay(1);
		maxPos++;
	}
	// X- switch reached
	ITM_write(".");
	if (limit1.read()) {
		LimitswitchXNeg = &limit1;
	} else if (limit2.read()) {
		LimitswitchXNeg = &limit2;
	} else if (limit3.read()) {
		LimitswitchXNeg = &limit3;
	} else if (limit4.read()) {
		LimitswitchXNeg = &limit4;
	}
	// Move away from switch
	ITM_write(".\r\n");
	dirPinX.write(PlotterConfiguration.stepperXDir);
	xSemaphoreTake(limitSwitchSignal, 0);
	while ((xSemaphoreTake(limitSwitchSignal, 0)) == pdTRUE) {
		stepPinX.write(TRUE);
		vTaskDelay(1);
		stepPinX.write(FALSE);
		vTaskDelay(1);
		maxPos--;
	}
	for (int i = 0; i < 50; i++) {
		stepPinX.write(TRUE);
		vTaskDelay(1);
		stepPinX.write(FALSE);
		vTaskDelay(1);
		maxPos--;
	}
	// Head is at 0 position
	currentPosX = 0.0f;
	currentTickPosX = 0;
	MaxXAxisTick = maxPos;
}

// Init Y-axis
void InitYAxis() {
	int maxPos = 0;

	// Move to Positive direction - No counting
	ITM_write("Y+ dir\r\n");
	dirPinY.write(PlotterConfiguration.stepperYDir);
	xSemaphoreTake(limitSwitchSignal, 0);
	while ((xSemaphoreTake(limitSwitchSignal, 0)) != pdTRUE) {
		stepPinY.write(TRUE);
		vTaskDelay(1);
		stepPinY.write(FALSE);
		vTaskDelay(1);
	}
	// Y+ switch reached
	ITM_write(".");
	if (limit1.read()) {
		LimitswitchYPos = &limit1;
	} else if (limit2.read()) {
		LimitswitchYPos = &limit2;
	} else if (limit3.read()) {
		LimitswitchYPos = &limit3;
	} else if (limit4.read()) {
		LimitswitchYPos = &limit4;
	}
	// Move away from switch
	ITM_write(".\r\n");
	dirPinY.write(!PlotterConfiguration.stepperYDir);
	xSemaphoreTake(limitSwitchSignal, 0);
	while ((xSemaphoreTake(limitSwitchSignal, 0)) == pdTRUE) {
		stepPinY.write(TRUE);
		vTaskDelay(1);
		stepPinY.write(FALSE);
		vTaskDelay(1);
	}
	for (int i = 0; i < 50; i++) {
		stepPinY.write(TRUE);
		vTaskDelay(1);
		stepPinY.write(FALSE);
		vTaskDelay(1);
	}

	// Move to Negative direction - Counting
	ITM_write("Y- dir\r\n");
	dirPinY.write(!PlotterConfiguration.stepperYDir);
	xSemaphoreTake(limitSwitchSignal, 0);
	while ((xSemaphoreTake(limitSwitchSignal, 0)) != pdTRUE) {
		stepPinY.write(TRUE);
		vTaskDelay(1);
		stepPinY.write(FALSE);
		vTaskDelay(1);
		maxPos++;
	}
	// Y- switch reached
	ITM_write(".");
	if (limit1.read()) {
		LimitswitchYNeg = &limit1;
	} else if (limit2.read()) {
		LimitswitchYNeg = &limit2;
	} else if (limit3.read()) {
		LimitswitchYNeg = &limit3;
	} else if (limit4.read()) {
		LimitswitchYNeg = &limit4;
	}
	// Move away from switch
	ITM_write(".\r\n");
	dirPinY.write(PlotterConfiguration.stepperYDir);
	xSemaphoreTake(limitSwitchSignal, 0);
	while ((xSemaphoreTake(limitSwitchSignal, 0)) == pdTRUE) {
		stepPinY.write(TRUE);
		vTaskDelay(1);
		stepPinY.write(FALSE);
		vTaskDelay(1);
		maxPos--;
	}
	for (int i = 0; i < 50; i++) {
		stepPinY.write(TRUE);
		vTaskDelay(1);
		stepPinY.write(FALSE);
		vTaskDelay(1);
		maxPos--;
	}
	// Head is at 0 position
	currentPosY = 0.0f;
	currentTickPosY = 0;
	MaxYAxisTick = maxPos;
}

/*
 * void InitPlotter()
 * @brief Set up plotter initial position and find limit switches
 *
 */
void InitPlotter() {

//	limit1 = new DigitalIoPin(1, 3, DigitalIoPin::pullup, true);
//	limit2 = new DigitalIoPin(0, 0, DigitalIoPin::pullup, true);
//	limit3 = new DigitalIoPin(0, 9, DigitalIoPin::pullup, true);
//	limit4 = new DigitalIoPin(0, 29, DigitalIoPin::pullup, true);
//	xMotor = new DigitalIoPin(1, 3, DigitalIoPin::output, true);
//	yMotor = new DigitalIoPin(0, 24, DigitalIoPin::output, true);
//	xDirection = new DigitalIoPin(0, 28, DigitalIoPin::output, true);
//	yDirection = new DigitalIoPin(1, 0, DigitalIoPin::output, true);

//	LimitswitchYPos = &limit1;
//	LimitswitchYNeg = &limit2;
//	LimitswitchXPos = &limit3;
//	LimitswitchXNeg = &limit4;

	ITM_write("Init plotter\r\n");

	xSemaphoreTake(limitSwitchSignal, 0);
	if ((xSemaphoreTake(limitSwitchSignal, 0)) == pdTRUE) {
		ITM_write("Limit switch is pressed. Init can not continue!\r\n");
		while(1) {
		}
	}

#if 0
	InitXAxis();
	InitYAxis();
#endif

#if 1
	currentPosX = 250.0f;
	currentTickPosX = 250;
	MaxXAxisTick = 500;
	currentPosY = 250.0f;
	currentTickPosY = 250;
	MaxYAxisTick = 500;
#endif

	changeRateX = (float) MaxXAxisTick / PlotterConfiguration.plotAreaX;
	changeRateY = (float) MaxYAxisTick / PlotterConfiguration.plotAreaY;

	ITM_write("Init finished\r\n");
}
