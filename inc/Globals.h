/*
 * Globals.h
 *
 *  Created on: Oct 12, 2019
 *      Author: Zoltan Gere
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "user_vcom.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "DigitalIoPin.h"

/*
 Queues:

 inputQueue (item type *char, item size 64)
 UART task puts a received command in this queue and GCodeParser picks
 it up for parsing.

 outputQueue (item type *char, item size 64)
 MotorController puts a reply in this queue and UART task picks it up for
 sending.

 instructionQueue (item type Instruction_t)
 GCodeParser puts the formated instruction for MotorController task.

 motorReplyQueue (item type Instruction_t)
 Motor task reports values required for M10 and M11 commands. Motor
 task must send parameters to this queue when it receives M10 or M11
 instruction.
 */

extern QueueHandle_t inputQueue;
extern QueueHandle_t outputQueue;
extern QueueHandle_t instructionQueue;
//QueueHandle_t motorReplyQueue;
extern SemaphoreHandle_t limitSwitchSignal;

#define INPUT_QUEUE_LENGTH (5)
#define ANSWER_QUEUE_LENGTH (5)
#define UART_QUEUE_ELEMENT_LENGTH (RCV_BUFSIZE)
#define INSTRUCTION_QUEUE_LENGTH (5)

typedef enum {
	EMPTY, M10, M11, M2, M1, M5, M4, G28, G1
} Instruction_E;

typedef struct {
	Instruction_E cmd;
	uint16_t arg1, arg2, arg3, arg4, arg5;
	float Xcoord, Ycoord;
} Instruction_t;

struct {
	const char * plotterType = "XY";
	bool stepperXDir = FALSE;
	bool stepperYDir = FALSE;
	// bool switchCheckY, switchCheckX;
	// uint32_t yValue, xValue;
	uint8_t penUp = 160;
	uint8_t penDown = 90;
	uint8_t plottingSpeed = 80;
	uint16_t plotAreaY = 500;
	uint16_t plotAreaX = 500;
} PlotterConfiguration;

extern DigitalIoPin limit1;
extern DigitalIoPin limit2;
extern DigitalIoPin limit3;
extern DigitalIoPin limit4;

extern DigitalIoPin * LimitswitchYPos;
extern DigitalIoPin * LimitswitchYNeg;
extern DigitalIoPin * LimitswitchXPos;
extern DigitalIoPin * LimitswitchXNeg;

//extern DigitalIoPin stepPinX;	// D10 - P0.27
//extern DigitalIoPin dirPinX;	// D11 - P0.28
//extern DigitalIoPin stepPinY;	// D8 - P0.24
//extern DigitalIoPin dirPinY;	// D9 - P1.0

#define MaxPPS (1000)						// Max. pulse-per-second = 100% speed
#define MARGIN (20)

extern float changeRateX;					// Float-to-Tick conversion ratio
extern float changeRateY;					// Float-to-Tick conversion ratio
extern float currentPosX;
extern float currentPosY;
extern volatile int32_t currentTickPosX;
extern volatile int32_t currentTickPosY;

extern int32_t MaxXAxisTick;				// Total number of steps on X-axis
extern int32_t MaxYAxisTick;				// Total number of steps on Y-axis

#endif /* GLOBALS_H_ */
