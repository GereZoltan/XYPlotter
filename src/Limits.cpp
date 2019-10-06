/*
 * limits.cpp
 *
 *  Created on: Oct 3, 2019
 *      Author: Ville Vainio
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "Limits.h"

DigitalIoPin *limit1;
DigitalIoPin *limit2;
DigitalIoPin *limit3;
DigitalIoPin *limit4;
DigitalIoPin *yMotor;
DigitalIoPin *xMotor;
DigitalIoPin *yDirection;
DigitalIoPin *xDirection;


void yMiddle(){
	specifications s;
	//Rotation direction
	s.switchCheckY = true;
	bool limitHit = false;

	//value between limits

	s.yValue = 0;

	//Lets the motor run to the first limit switch
	while(limitHit == false){
		yMotor->write(true);
		if(!limit1->read() || !limit2->read()); // runs to the second limit
		else{
			limitHit = true;
		}
		yDirection->write(s.switchCheckY);
		vTaskDelay(pdMS_TO_TICKS(2));
		yMotor->write(false);
	}

	//switches the direction
	s.switchCheckY = (bool) !s.switchCheckY;

	//runs out of the limit
	while(limit1->read() || limit2->read()){ // while motor hits either of the limits
		yMotor->write(true); // run the motor until it doesnt hit
		yDirection->write(s.switchCheckY);
		vTaskDelay(pdMS_TO_TICKS(2));
		yMotor->write(false);
	}

	//calculates the value every 2 ticks between limit 1 and limit 2
	while(limitHit == true){
		s.yValue++; //store the ticks between limits here
		yMotor->write(true);

		//if limit it hit stops the calculation and exits the limit and then loop
		if(limit1->read() || limit2->read()){ // if limit is hit, switch the direction
			s.switchCheckY = (bool) !s.switchCheckY;
			limitHit = false;
		while(limit1->read() || limit2->read()){ // as long as the limits are open, run to one dircetion
			yMotor->write(true);
			yDirection->write(s.switchCheckY);
			vTaskDelay(pdMS_TO_TICKS(2));
			yMotor->write(false);
			}
		}
		yDirection->write(s.switchCheckY); // run out of the limit after it's hit.
		vTaskDelay(pdMS_TO_TICKS(2));
		yMotor->write(false);
	}
	//ticks from the middle position to limit
	s.yValue = s.yValue/2; // store the amount of ticks here.

	//run the motor to the middle position
	while(s.yValue > 0){
		yMotor->write(true);
		yDirection->write(s.switchCheckY);
		vTaskDelay(pdMS_TO_TICKS(2));
		yMotor->write(false);
		s.yValue--;
	}
}

// same as with the y
void xMiddle(){
		specifications s;
		s.switchCheckX = true;
		bool limitHit = false;
		s.xValue = 0;

		//doesn't run until the other task is finished
		while(limitHit == false){
			xMotor->write(true);
			if(!limit3->read() || limit4->read());
			else{
				limitHit = true;
			}
			xDirection->write(s.switchCheckX);
			vTaskDelay(pdMS_TO_TICKS(2));
			xMotor->write(false);
		}
		s.switchCheckX = (bool) !s.switchCheckX;
		while(limit3->read() || limit4->read()){
			xMotor->write(true);
			xDirection->write(s.switchCheckX);
			vTaskDelay(pdMS_TO_TICKS(2));
			xMotor->write(false);
		}
		while(limitHit == true){
			s.xValue++;
			xMotor->write(true);
			if(limit3->read() || limit4->read()){
				s.switchCheckX = (bool) !s.switchCheckX;
				limitHit = false;
			while(limit3->read() || limit4->read()){
				xMotor->write(true);
				xDirection->write(s.switchCheckX);
				vTaskDelay(pdMS_TO_TICKS(2));
				xMotor->write(false);
				}

			}
			xDirection->write(s.switchCheckX);
			vTaskDelay(pdMS_TO_TICKS(2));
			xMotor->write(false);
		}
		s.xValue = s.xValue/2;

		while(s.xValue > 0){
				xMotor->write(true);
				xDirection->write(s.switchCheckX);
				vTaskDelay(pdMS_TO_TICKS(2));
				xMotor->write(false);
				s.xValue--;
			}
}



void vLimits(void *pvParameters) {

    limit1 = new DigitalIoPin(1,3,DigitalIoPin::pullup, true);
    limit2 = new DigitalIoPin(0,0,DigitalIoPin::pullup, true);
    limit3 = new DigitalIoPin(0,9,DigitalIoPin::pullup, true);
    limit4 = new DigitalIoPin(0,29,DigitalIoPin::pullup, true);
    xMotor = new DigitalIoPin(1,3,DigitalIoPin::output, true);
    yMotor = new DigitalIoPin(0,24, DigitalIoPin::output, true);
	xDirection = new DigitalIoPin(0,28, DigitalIoPin::output, true);
	yDirection = new DigitalIoPin(1,0, DigitalIoPin::output, true);


    //releases the binarySemaphore for the other task

    while (1) {

    }
}


