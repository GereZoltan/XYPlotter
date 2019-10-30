/*
 * Servo.h
 *
 *  Created on: Oct 3, 2019
 *      Author: Zoltan Gere
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "FreeRTOS.h"

#define PENPIN 10
#define LASERPIN 12
#define PENBIT (1 << 0)
#define LASERBIT (1 << 1)

/*
 * Servo class
 * @brief    PWM capability to control RC servos and gradient power outputs
 */
class Servo {
private:
	void SCT_Init_L();
	void SCT_Init_H();
	static uint8_t configuredSCTs;	// Bit0: Pen, Bit1: Laser
public:
	enum ServoType_E {
		pen, laser
	};

	Servo(ServoType_E servoType);
	virtual ~Servo();
	void SetPenPosition(uint8_t position);
	void SetLaserPower(uint8_t power);
	void SetLaser(bool state);
};

#endif /* SERVO_H_ */
