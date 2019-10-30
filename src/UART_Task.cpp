/*
 * UART_Task.cpp
 *
 *  Created on: Sep 29, 2019
 *      Author: Zoltan Gere
 */

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ITM_write.h"
#include "user_vcom.h"
#include "Globals.h"

extern QueueHandle_t inputQueue;
extern QueueHandle_t outputQueue;

/**
 * UART_Task .h
 * <pre> void UARTReaderTask(void *pvParameters);
 *
 * Uart_task: communicates with mDraw
 *
 * @brief	Reads 1 line.
 * Received string placed in inputQueue.
 *
 * @return    Nothing, function should not exit
 */
void UARTWriterTask(void *pvParameters) {
	char stext[RCV_BUFSIZE];

	vTaskDelay(pdMS_TO_TICKS(1000)); /* wait until semaphores are created */

	while (1) {
		xQueueReceive(outputQueue, (void *) stext, portMAX_DELAY);
		USB_send((uint8_t *) stext, strlen(stext));
//		ITM_write("UART SEND: ");
//		ITM_write(stext);
//		ITM_write("\r\n");
	}	// End infinite loop
}

/**
 * UART_Task .h
 * <pre> void UARTWriterTask(void *pvParameters);
 *
 * Uart_task: communicates with mDraw
 *
 * @brief	sends 1 line.
 * Reads from answerQueue and sends back to mDraw.
 *
 * @return    Nothing, function should not exit
 */
void UARTReaderTask(void *pvParameters) {
	uint32_t readCharCount;
	char rtext[RCV_BUFSIZE + 1];		// +1 therefore trailing \0 always fits

	vTaskDelay(pdMS_TO_TICKS(1000)); /* wait until semaphores are created */

	while (1) {
		readCharCount = USB_receive((uint8_t *) rtext, RCV_BUFSIZE);		// Function blocks until data is available.
		rtext[readCharCount] = '\0';										// mDraw strings end with '\n'
		xQueueSend(inputQueue, (void *) rtext, portMAX_DELAY);				// Add '\0' to library string functions work properly
//		ITM_write("UART RECEIVE: ");
//		ITM_write(rtext);
//		ITM_write("\r\n");
	}	// End infinite loop
}
