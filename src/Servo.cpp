/*
 * Servo.cpp
 *
 *  Created on: Oct 3, 2019
 *      Author: Zoltan Gere
 */

#include "Servo.h"
#include "DigitalIoPin.h"

uint8_t Servo::configuredSCTs = 0;
DigitalIoPin laserPin(0, 12, DigitalIoPin::output, false);	// D12 - P0.12

/*
 * Large SCT0 as two 16-bit timers
 * Low part drive pen servo
 */
void Servo::SCT_Init_L() {
	// Low part - Pen
	LPC_SCTLARGE0->CONFIG |= (1 << 17);				// Autolimit_L
	LPC_SCTLARGE0->CTRL_L |= (1 << 3);				// Clear counter_L
	LPC_SCTLARGE0->CTRL_L |= ((72 - 1) << 5);		// Set Prescaler_L to 1MHz

	LPC_SCTLARGE0->MATCHREL[0].L = 20000 - 1;		// 1MHz / 50Hz = 20000
	LPC_SCTLARGE0->MATCHREL[1].L = 1500;			// 0 - 1000

	LPC_SCTLARGE0->EVENT[0].STATE = 0xFFFF;
	LPC_SCTLARGE0->EVENT[0].CTRL = (0 << 4) | (1 << 12);

	LPC_SCTLARGE0->EVENT[1].STATE = 0xFFFF;
	LPC_SCTLARGE0->EVENT[1].CTRL = (1 << 0) | (0 << 4) | (1 << 12);

	LPC_SCTLARGE0->OUT[0].SET = (1 << 0);			// Output0 set by event 0
	LPC_SCTLARGE0->OUT[0].CLR = (1 << 1);			// Output0 cleared by event 1

	// Start counters
	LPC_SCTLARGE0->CTRL_L &= ~(1 << 2);
}

/*
 * Large SCT0 as two 16-bit timers
 * High part drive laser power
 */
void Servo::SCT_Init_H() {
	// High part - Laser
	LPC_SCTLARGE0->CONFIG |= (1 << 18);				// Autolimit_H
	LPC_SCTLARGE0->CTRL_H |= (1 << 3);				// Clear counter_H
	LPC_SCTLARGE0->CTRL_H |= ((72 - 1) << 5);		// Set Prescaler_H to 1MHz

	LPC_SCTLARGE0->MATCHREL[0].H = 256 - 1;			// 1MHz / 3906Hz = 256
	LPC_SCTLARGE0->MATCHREL[1].H = 255;				// 0 - 255

	LPC_SCTLARGE0->EVENT[2].STATE = 0xFFFF;
	LPC_SCTLARGE0->EVENT[2].CTRL = (1 << 4) | (1 << 12);

	LPC_SCTLARGE0->EVENT[3].STATE = 0xFFFF;
	LPC_SCTLARGE0->EVENT[3].CTRL = (1 << 0) | (1 << 4) | (1 << 12);

	LPC_SCTLARGE0->OUT[1].SET = (1 << 2);			// Output1 set by event 2
	LPC_SCTLARGE0->OUT[1].CLR = (1 << 3);			// Output1 cleared by event 3

	// Start counters
//	LPC_SCTLARGE0->CTRL_H &= ~(1 << 2);
}

/*
 * Constructor
 * @brief	Initialize Pen / Laser
 * Protected to init only once
 */
Servo::Servo(ServoType_E servoType) {
	// Set up pen only once
	if (((Servo::configuredSCTs & PENBIT) == 0)
			&& (servoType == ServoType_E::pen)) {
		// Initialize SWM
		Chip_SWM_MovablePinAssign(SWM_SCT0_OUT0_O, PENPIN);

		// Initialize PWM
		SCT_Init_L();

		Servo::configuredSCTs |= PENBIT;
	}

	// Set up laser only once
	if (((Servo::configuredSCTs & LASERBIT) == 0)
			&& (servoType == ServoType_E::laser)) {
		// Initialize SWM
		Chip_SWM_MovablePinAssign(SWM_SCT0_OUT1_O, LASERPIN);

		// Initialize PWM
		SCT_Init_H();

		Servo::configuredSCTs |= LASERBIT;
	}
}

Servo::~Servo() {
	// TODO Auto-generated destructor stub
}

/*
 * void Servo::SetPenPosition(uint8_t position)
 * @brief	Translate 0-255 position to 1000-2000 microsecond
 *			Write PWM output
 */
void Servo::SetPenPosition(uint8_t position) {
	// Todo: translate 0-255 position to 1000-2000 usecond
	uint32_t newPosition = (uint32_t) position;

	// Pen
	if (Servo::configuredSCTs & PENBIT) {
		newPosition *= 1000;
		if (newPosition != 0) {
			newPosition /= 255;
		}
		newPosition += 1000;

		LPC_SCTLARGE0->MATCHREL[1].L = newPosition;			// Set Pen position
	}

}

/*
 * void Servo::SetLaserPower(uint8_t power)
 * @brief	Write PWM output
 */
void Servo::SetLaserPower(uint8_t power) {
	// Laser
	if (Servo::configuredSCTs & LASERBIT) {
		LPC_SCTLARGE0->MATCHREL[1].H = (uint16_t) power;	// Set Laser Power
	}

}

/*
 * void Servo::SetLaser(bool state)
 * @brief	Turn On/Off Laser by turning On/Off PWM counter
 */
void Servo::SetLaser(bool state) {
//	DigitalIoPin laserPin(0, 12, DigitalIoPin::output, false);	// D12 - P0.12
	// Laser
	if (Servo::configuredSCTs & LASERBIT) {
		if (state) {
			LPC_SCTLARGE0->CTRL_H &= ~(1 << 2);				// Turn Laser On
			laserPin.write(TRUE);
		} else {
			LPC_SCTLARGE0->CTRL_H |= (1 << 2);				// Turn Laser Off
			LPC_SCTLARGE0->CTRL_H |= (1 << 3);				// Clear counter_H
			laserPin.write(FALSE);
		}
	}
}
