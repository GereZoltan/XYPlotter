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

//DigitalIoPin stepPinY(0, 27, DigitalIoPin::output, false);	// D10 - P0.27
//DigitalIoPin dirPinY(0, 28, DigitalIoPin::output, false);	// D11 - P0.28
//DigitalIoPin stepPinX(0, 24, DigitalIoPin::output, false);	// D8 - P0.24
//DigitalIoPin dirPinX(1, 0, DigitalIoPin::output, false);	// D9 - P1.0

SemaphoreHandle_t limitSwitchSignal;

float changeRateX;					// Float-to-Tick conversion ratio
float changeRateY;					// Float-to-Tick conversion ratio
float currentPosX;
float currentPosY;
volatile int32_t currentTickPosX;
volatile int32_t currentTickPosY;

int32_t MaxXAxisTick;					// Total number of steps on X-axis
int32_t MaxYAxisTick;					// Total number of steps on Y-axis

/*
 * Limit switches are grounding switches. They read one when the switch is open (= not at a limit).
 *
 * Automatic printable area and limit switch detection
 * Software must be able to detect which limit switch is connected which axis automatically
 * to prevent swapper connectors from causing problems
 *
 * Enforcement of limits (=always stop at switch, never hit an edge)
 */
//void readLimitSwitchesTask(void *pvParameters) {
//	vTaskDelay(pdMS_TO_TICKS(1000));
//	while (1) {
//		if (limit1.read() || limit2.read() || limit3.read() || limit4.read()) {
//			xSemaphoreGive(limitSwitchSignal);
//		}
//		vTaskDelay(pdMS_TO_TICKS(1));
//	}
//}

// Init X-axis
void InitXAxis() {
	int wait = 1;
	uint32_t t = 1;
	DigitalIoPin stepPinX(0, 24, DigitalIoPin::output, false);	// D8 - P0.24
	DigitalIoPin dirPinX(1, 0, DigitalIoPin::output, false);	// D9 - P1.0

	int32_t maxPos = 0;

	// Move to Positive direction - No counting
	ITM_write("X+ dir");
	dirPinX.write(PlotterConfiguration.stepperXDir);
//	xSemaphoreTake(limitSwitchSignal, 0);
	while (!(limit1.read() || limit2.read() || limit3.read() || limit4.read())) {
//	while ((xSemaphoreTake(limitSwitchSignal, 0)) != pdTRUE) {
		stepPinX.write(TRUE);
		for (int j = 0; j < 72; j++) {t = j << 1;}
//		vTaskDelay(pdMS_TO_TICKS(wait));
		stepPinX.write(FALSE);
		vTaskDelay(pdMS_TO_TICKS(wait));
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
//	xSemaphoreTake(limitSwitchSignal, 0);
	while ((limit1.read() || limit2.read() || limit3.read() || limit4.read())) {

//	while ((xSemaphoreTake(limitSwitchSignal, 1)) == pdTRUE) {
		stepPinX.write(TRUE);
		for (int j = 0; j < 72; j++) {t = j << 1;}
//		vTaskDelay(pdMS_TO_TICKS(wait));
		stepPinX.write(FALSE);
		vTaskDelay(pdMS_TO_TICKS(wait));
	}
	for (int i = 0; i < MARGIN; i++) {
		stepPinX.write(TRUE);
		for (int j = 0; j < 72; j++) {t = j << 1;}
//		vTaskDelay(pdMS_TO_TICKS(wait));
		stepPinX.write(FALSE);
		vTaskDelay(pdMS_TO_TICKS(wait));
	}

	// Move to Negative direction - Counting
	ITM_write("X- dir");
	dirPinX.write(!PlotterConfiguration.stepperXDir);
//	xSemaphoreTake(limitSwitchSignal, 0);
	while (!(limit1.read() || limit2.read() || limit3.read() || limit4.read())) {
		stepPinX.write(TRUE);
		for (int j = 0; j < 72; j++) {t = j << 1;}
//		vTaskDelay(pdMS_TO_TICKS(wait));
		stepPinX.write(FALSE);
		vTaskDelay(pdMS_TO_TICKS(wait));
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
//	xSemaphoreTake(limitSwitchSignal, 0);
	while ((limit1.read() || limit2.read() || limit3.read() || limit4.read())) {
		stepPinX.write(TRUE);
		for (int j = 0; j < 72; j++) {t = j << 1;}
//		vTaskDelay(pdMS_TO_TICKS(wait));
		stepPinX.write(FALSE);
		vTaskDelay(pdMS_TO_TICKS(wait));
		maxPos--;
	}
	for (int i = 0; i < MARGIN; i++) {
		stepPinX.write(TRUE);
		for (int j = 0; j < 72; j++) {t = j << 1;}
//		vTaskDelay(pdMS_TO_TICKS(wait));
		stepPinX.write(FALSE);
		vTaskDelay(pdMS_TO_TICKS(wait));
		maxPos--;
	}
	// Head is at 0 position
	currentPosX = 0.0f;
	currentTickPosX = 0;
	MaxXAxisTick = maxPos;
}

// Init Y-axis
void InitYAxis() {
	int wait = 1;
	uint32_t t = 1;
	DigitalIoPin stepPinY(0, 27, DigitalIoPin::output, false);	// D10 - P0.27
	DigitalIoPin dirPinY(0, 28, DigitalIoPin::output, false);	// D11 - P0.28

	int32_t maxPos = 0;

	// Move to Positive direction - No counting
	ITM_write("Y+ dir");
	dirPinY.write(PlotterConfiguration.stepperYDir);
//	xSemaphoreTake(limitSwitchSignal, 0);
	while (!(limit1.read() || limit2.read() || limit3.read() || limit4.read())) {
		stepPinY.write(TRUE);
		for (int j = 0; j < 72; j++) {t = j << 1;}
		//		vTaskDelay(pdMS_TO_TICKS(wait));
		stepPinY.write(FALSE);
		vTaskDelay(pdMS_TO_TICKS(wait));
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
//	xSemaphoreTake(limitSwitchSignal, 0);
	while ((limit1.read() || limit2.read() || limit3.read() || limit4.read())) {
		stepPinY.write(TRUE);
		for (int j = 0; j < 72; j++) {t = j << 1;}
		//		vTaskDelay(pdMS_TO_TICKS(wait));
		stepPinY.write(FALSE);
		vTaskDelay(pdMS_TO_TICKS(wait));
	}
	for (int i = 0; i < MARGIN; i++) {
		stepPinY.write(TRUE);
		for (int j = 0; j < 72; j++) {t = j << 1;}
		//		vTaskDelay(pdMS_TO_TICKS(wait));
		stepPinY.write(FALSE);
		vTaskDelay(pdMS_TO_TICKS(wait));
	}

	// Move to Negative direction - Counting
	ITM_write("Y- dir");
	dirPinY.write(!PlotterConfiguration.stepperYDir);
//	xSemaphoreTake(limitSwitchSignal, 0);
	while (!(limit1.read() || limit2.read() || limit3.read() || limit4.read())) {
		stepPinY.write(TRUE);
		for (int j = 0; j < 72; j++) {t = j << 1;}
		//		vTaskDelay(pdMS_TO_TICKS(wait));
		stepPinY.write(FALSE);
		vTaskDelay(pdMS_TO_TICKS(wait));
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
//	xSemaphoreTake(limitSwitchSignal, 0);
	while ((limit1.read() || limit2.read() || limit3.read() || limit4.read())) {
		stepPinY.write(TRUE);
		for (int j = 0; j < 72; j++) {t = j << 1;}
		//		vTaskDelay(pdMS_TO_TICKS(wait));
		stepPinY.write(FALSE);
		vTaskDelay(pdMS_TO_TICKS(wait));
		maxPos--;
	}
	for (int i = 0; i < MARGIN; i++) {
		stepPinY.write(TRUE);
		for (int j = 0; j < 72; j++) {t = j << 1;}
		//		vTaskDelay(pdMS_TO_TICKS(wait));
		stepPinY.write(FALSE);
		vTaskDelay(pdMS_TO_TICKS(wait));
		maxPos--;
	}
	// Head is at 0 position
	currentPosY = 0.0f;
	currentTickPosY = 0;
	MaxYAxisTick = maxPos;
}

/**
 * Limits .cpp
 * <pre> void InitPlotter();
 *
 * InitPlotter: Initialize plotter hardware
 * Locate limit switches
 * Measure X/Y axes distance
 * Calculate conversion ratio
 *
 * @brief	Initialize plotter hardware
 *
 * @return    Nothing, result written to global variables
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

	ITM_write("Waiting for open limit switches...\r\n");

	//xSemaphoreTake(limitSwitchSignal, 0);
	while ((limit1.read() || limit2.read() || limit3.read() || limit4.read())) {// Wait for limit switches to open
	}

	ITM_write("Init plotter...\r\n");

#if 1
	InitXAxis();
	InitYAxis();
#else
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
