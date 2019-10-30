/*
 ===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
 ===============================================================================
 */

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

// TODO: insert other include files here

// TODO: insert other definitions and declarations here

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "DigitalIoPin.h"

#include "ITM_write.h"
#include "Globals.h"
#include "UART_Task.h"
#include "MotorController_Task.h"
#include "Limits.h"

void GCodeParserTask(void *pvParameters);

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

QueueHandle_t inputQueue;
QueueHandle_t outputQueue;
QueueHandle_t instructionQueue;

extern SemaphoreHandle_t limitSwitchSignal;
// extern QueueHandle_t motorReplyQueue;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Sets up system hardware */
static void prvSetupHardware(void) {
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
//	Board_LED_Set(0, false);
//	Board_LED_Set(1, false);
//	Board_LED_Set(2, false);

	/* Initialize RITimer */
	Chip_RIT_Init(LPC_RITIMER);
	// set the priority level of the interrupt
	// The level must be equal or lower than the maximum priority specified in FreeRTOS config
	// Note that in a Cortex-M3 a higher number indicates lower interrupt priority
	NVIC_SetPriority(RITIMER_IRQn,
	configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);

	// Init SWM
	Chip_SWM_Init();

	// Init SCT clock
	Chip_SCT_Init(LPC_SCTLARGE0);

	ITM_init();
	ITM_write("Init complete\r\n");
}

void TestingTask(void *pvParameters) {
	char ttext[RCV_BUFSIZE];
	const char * M10answer =
			"M10 XY 500 500 0.00 0.00 A0 B0 H0 S80 U160 D90\r\rOK\r\n";
	const char * M11answer = "M11 1 1 1 1\r\nOK\r\n";
	const char * OKanswer = "OK\r\n";

	vTaskDelay(100); /* wait until semaphores are created */

	while (1) {
		xQueueReceive(inputQueue, (void *) ttext, portMAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(50));
		if (ttext[0] == 'M' && ttext[1] == '1' && ttext[2] == '0') {
			xQueueSend(outputQueue, (void * ) M10answer, portMAX_DELAY);
		} else if (ttext[0] == 'M' && ttext[1] == '1' && ttext[2] == '1') {
			xQueueSend(outputQueue, (void * ) M11answer, portMAX_DELAY);
		} else {
			xQueueSend(outputQueue, (void * ) OKanswer, portMAX_DELAY);
		}
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* the following is required if runtime statistics are to be collected */
extern "C" {

void vConfigureTimerForRunTimeStats(void) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}
/* end runtime statistics collection */

/**
 * @brief    main routine for FreeRTOS blinky example
 * @return    Nothing, function should not exit
 */
int main(void) {
	prvSetupHardware();

	inputQueue = xQueueCreate(INPUT_QUEUE_LENGTH, UART_QUEUE_ELEMENT_LENGTH);
	outputQueue = xQueueCreate(ANSWER_QUEUE_LENGTH, UART_QUEUE_ELEMENT_LENGTH);
	instructionQueue = xQueueCreate(INSTRUCTION_QUEUE_LENGTH,
			sizeof(Instruction_t));
//    motorReplyQueue = xQueueCreate(INSTRUCTION_QUEUE_LENGTH, sizeof(Instruction_t));
	limitSwitchSignal = xSemaphoreCreateBinary();

	if (inputQueue == NULL || outputQueue == NULL || instructionQueue == NULL || limitSwitchSignal == NULL) {
		ITM_write("Error creating queues\r\n");
		while (1)
			;
	}

	xTaskCreate(UARTReaderTask,
			"Read UART",								// Required for UART
			configMINIMAL_STACK_SIZE * 2, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	xTaskCreate(UARTWriterTask,
			"Write UART",								// Required for UART
			configMINIMAL_STACK_SIZE * 2, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	xTaskCreate(cdc_task,
			"CDC",											// Required for UART
			configMINIMAL_STACK_SIZE * 2, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

//	xTaskCreate(TestingTask, "UART Test",									// Test UART communication
//	configMINIMAL_STACK_SIZE * 2, NULL, (tskIDLE_PRIORITY + 1UL),
//			(TaskHandle_t *) NULL);

	xTaskCreate(MotorControllerTask,
			"Motor ctrl",							// Motor control
			configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	xTaskCreate(GCodeParserTask,
			"GCode parser",							// G-code parser
			configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

//	xTaskCreate(readLimitSwitchesTask,
//			"Read lmtSW",							// Monitor limit switches
//			configMINIMAL_STACK_SIZE * 2, NULL, (tskIDLE_PRIORITY + 1UL),
//			(TaskHandle_t *) NULL);

	vQueueAddToRegistry(inputQueue, "UART receive");
	vQueueAddToRegistry(outputQueue, "UART send");
	vQueueAddToRegistry(instructionQueue, "Instructions");
	vQueueAddToRegistry(limitSwitchSignal, "Limit SW");

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
