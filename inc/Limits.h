/*
 * Limits.h
 *
 *  Created on: Oct 3, 2019
 *      Author: Ville Vainio
 */

#ifndef LIMITS_H_
#define LIMITS_H_

void readLimitSwitchesTask(void *pvParameters);

/**
 * Limits .h
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
void InitPlotter();

#endif /* LIMITS_H_ */
