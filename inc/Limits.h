/*
 * Limits.h
 *
 *  Created on: Oct 3, 2019
 *      Author: Ville Vainio
 */

#ifndef LIMITS_H_
#define LIMITS_H_
#include "DigitalIoPin.h"

typedef struct{
	bool switchCheckY, switchCheckX;
	uint32_t yValue, xValue;
	uint8_t penUp = 160, penDown = 90, plottingSpeed = 80;
	uint16_t plotAreaY = 380, plotAreaX = 310;
}specifications;

void vLimits(void *pvParameters);
void xMiddle();
void yMiddle();


#endif /* LIMITS_H_ */
