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
#include "Fmutex.h"

#include "ITM_write.h"
#include "user_vcom.h"
#include "UART_Task.h"
#include "MotorController_Task.h"

#include "gcodeparser.cpp"

Fmutex binary;


/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
#define INPUT_QUEUE_LENGTH (5)
#define ANSWER_QUEUE_LENGTH (5)
#define UART_QUEUE_ELEMENT_LENGTH (RCV_BUFSIZE)
#define INSTRUCTION_QUEUE_LENGTH (5)

extern QueueHandle_t inputQueue;
extern QueueHandle_t answerQueue;
extern QueueHandle_t instructionQueue;

/*****************************************************************************
 * Private functions
 ****************************************************************************/


/* Sets up system hardware */
static void prvSetupHardware(void)
{
    SystemCoreClockUpdate();
    Board_Init();

    /* Initial LED0 state is off */
    Board_LED_Set(0, false);
    Board_LED_Set(1, false);
    Board_LED_Set(2, false);

    ITM_init();
    ITM_write("Init complete\r\n");
}


static void vYLimit(void *pvParameters) {
    //X limits
    DigitalIoPin limit1(1,3,DigitalIoPin::pullup, true);
    DigitalIoPin limit2(0,0,DigitalIoPin::pullup, true);

    //X motors
    DigitalIoPin yMotor(0,24, DigitalIoPin::output, true);
    DigitalIoPin yDirection(1,0, DigitalIoPin::output, true);

    //Rotation direction
    bool switchCheckY = true;
    bool limitHit = false;

    //value between limits
    uint32_t yValue = 0;

    //Lets the motor run to the first limit switch
    while(limitHit == false){
        yMotor.write(true);
        if(!limit1.read() || limit2.read());
        else{
            limitHit = true;
        }
        yDirection.write(switchCheckY);
        vTaskDelay(pdMS_TO_TICKS(2));
        yMotor.write(false);
    }

    //switches the direction
    switchCheckY = (bool) !switchCheckY;

    //runs out of the limit
    while(limit1.read() || limit2.read()){
        yMotor.write(true);
        yDirection.write(switchCheckY);
        vTaskDelay(pdMS_TO_TICKS(2));
        yMotor.write(false);
    }

    //calculates the value every 2 ticks between limit 1 and limit 2
    while(limitHit == true){
        yValue++;
        yMotor.write(true);

        //if limit it hit stops the calculation and exits the limit and then loop
        if(limit1.read() || limit2.read()){
            switchCheckY = (bool) !switchCheckY;
            limitHit = false;
        while(limit1.read() || limit2.read()){
            yMotor.write(true);
            yDirection.write(switchCheckY);
            vTaskDelay(pdMS_TO_TICKS(2));
            yMotor.write(false);
            }
        }
        yDirection.write(switchCheckY);
        vTaskDelay(pdMS_TO_TICKS(2));
        yMotor.write(false);
    }

    //releases the binarySemaphore for the other task
    binary.unlock();
    while (1) {

    }
}

static void vXLimit(void *pvParameters) {
    //same as the above task
    DigitalIoPin xMotor(0,27, DigitalIoPin::output, true);
    DigitalIoPin xDirection(0,28, DigitalIoPin::output, true);
    DigitalIoPin limit3(0,9,DigitalIoPin::pullup, true);
    DigitalIoPin limit4(0,29,DigitalIoPin::pullup, true);
    bool switchCheckX = true;
    bool limitHit = false;
    uint32_t xValue = 0;

    //doesn't run until the other task is finished
    binary.lock();
    while(limitHit == false){
        xMotor.write(true);
        if(!limit3.read() || limit4.read());
        else{
            limitHit = true;
        }
        xDirection.write(switchCheckX);
        vTaskDelay(pdMS_TO_TICKS(2));
        xMotor.write(false);
    }
    switchCheckX = (bool) !switchCheckX;
    while(limit3.read() || limit4.read()){
        xMotor.write(true);
        xDirection.write(switchCheckX);
        vTaskDelay(pdMS_TO_TICKS(2));
        xMotor.write(false);
    }
    while(limitHit == true){
        xValue++;
        xMotor.write(true);
        if(limit3.read() || limit4.read()){
            switchCheckX = (bool) !switchCheckX;
            limitHit = false;
        while(limit3.read() || limit4.read()){
            xMotor.write(true);
            xDirection.write(switchCheckX);
            vTaskDelay(pdMS_TO_TICKS(2));
            xMotor.write(false);
            }

        }
        xDirection.write(switchCheckX);
        vTaskDelay(pdMS_TO_TICKS(2));
        xMotor.write(false);
    }
    while(1){

    }
}

void TestingTask(void *pvParameters) {
	char ttext[RCV_BUFSIZE];
	const char * M10answer = "M10 XY 500 500 0.00 0.00 A0 B0 H0 S80 U160 D90\r\rOK\r\n";
	const char * M11answer = "M11 1 1 1 1\r\nOK\r\n";
	const char * OKanswer = "OK\r\n";

	vTaskDelay(100); /* wait until semaphores are created */

	while(1) {
		xQueueReceive(inputQueue, (void *) ttext, portMAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(50));
		if (ttext[0] == 'M' && ttext[1] == '1' && ttext[2] == '0') {
			xQueueSend(answerQueue, (void *) M10answer, portMAX_DELAY);
		} else if (ttext[0] == 'M' && ttext[1] == '1' && ttext[2] == '1') {
			xQueueSend(answerQueue, (void *) M11answer, portMAX_DELAY);
		} else {
			xQueueSend(answerQueue, (void *) OKanswer, portMAX_DELAY);
		}
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* the following is required if runtime statistics are to be collected */
extern "C" {

void vConfigureTimerForRunTimeStats( void ) {
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
int main(void)
{
    prvSetupHardware();

    inputQueue = xQueueCreate(INPUT_QUEUE_LENGTH, UART_QUEUE_ELEMENT_LENGTH);
    answerQueue = xQueueCreate(ANSWER_QUEUE_LENGTH, UART_QUEUE_ELEMENT_LENGTH);
    instructionQueue = xQueueCreate(INSTRUCTION_QUEUE_LENGTH, sizeof(Instruction_t));

    if (inputQueue == NULL || answerQueue == NULL || instructionQueue == NULL) {
    	ITM_write("Error creating queues\r\n");
    	while(1)
    		;
    }

    xTaskCreate(vYLimit, "vYLimit",
                configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
                (TaskHandle_t *) NULL);

    xTaskCreate(vXLimit, "vXLimit",
            configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
            (TaskHandle_t *) NULL);

	xTaskCreate(UARTReaderTask, "Read UART",								// Required for UART
	configMINIMAL_STACK_SIZE * 2, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	xTaskCreate(UARTWriterTask, "Write UART",								// Required for UART
	configMINIMAL_STACK_SIZE * 2, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	xTaskCreate(cdc_task, "CDC",											// Required for UART
	configMINIMAL_STACK_SIZE * 2, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	xTaskCreate(MotorControllerTask, "Motor ctrl",							// Motor control
	configMINIMAL_STACK_SIZE * 2, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	xTaskCreate(vTaskGCodeParser, "GCode parser",							// G-code parser
	configMINIMAL_STACK_SIZE * 2, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	vQueueAddToRegistry(inputQueue, "UART receive");
	vQueueAddToRegistry(answerQueue, "UART send");
	vQueueAddToRegistry(instructionQueue, "Instructions");

    /* Start the scheduler */
    vTaskStartScheduler();

    /* Should never arrive here */
    return 1;
}
