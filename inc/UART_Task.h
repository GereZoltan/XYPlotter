/*
 * UART_Task.h
 *
 *  Created on: Sep 29, 2019
 *      Author: Zoltan Gere
 */

#ifndef UART_TASK_H_
#define UART_TASK_H_



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
void UARTReaderTask(void *pvParameters);

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
void UARTWriterTask(void *pvParameters);

#endif /* UART_TASK_H_ */
