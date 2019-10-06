/*
 * GCodeParser_Task.cpp
 *
 *  Created on: Sep 5, 2019
 *      Author: Joonas Saarinen
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <vector>
#include <cstring>
#include <string>
#include <cstdio>

#include "MotorController_Task.h"

/*
	Queues:

	inputQueue (item type *char, item size 64)
		UART task puts a received command in this queue and GCodeParser picks
		it up for parsing.

	outputQueue (item type *char, item size 64)
		GCodeParser puts a reply in this queue and UART task picks it up for
		sending.

	instructionQueue (item type Instruction_t)
		GCodeParser reports an updated parameter to Motor task.

	motorReplyQueue (item type Instruction_t)
		Motor task reports values required for M10 and M11 commands. Motor
		task must send parameters to this queue when it receives M10 or M11
		instruction.
*/

QueueHandle_t inputQueue;
QueueHandle_t outputQueue;
QueueHandle_t instructionQueue;
QueueHandle_t motorReplyQueue;

void GCodeParserTask (void *pvParameters) {
	const unsigned int buffersize = 64;
	char buffer[64];
	Instruction_t instruction;

	std::string word;
	std::vector<std::string> words;

	while(1) {
		words.clear();
		word.clear();

//		instruction = { 0 };

		xQueueReceive(inputQueue, buffer, portMAX_DELAY);

		for (unsigned int c = 0; c < (strlen(buffer) + 1); c++) {
			if (buffer[c] == ' ' || buffer[c] == 0) {
				words.push_back(word);
				word.clear();
			} else {
				word.push_back(buffer[c]);
			}
		}

		if (!words.empty()) {
			if (words[0] == "G1") {
				// G1: Move to coordinates

				bool good = true;

				// Must be 4 words (for example: G1 X85.14 Y117.29 A0)
				good &= words.size() == 4;

				// Last word must be "A0"
				good &= words[words.size() - 1] == "A0";

				if (good) {
					// Parse coordinates

					float coordx = 0.0f;
					float coordy = 0.0f;
					
					std::string coordtextx;
					std::string coordtexty;

					coordtextx = words[1];
					coordtextx.erase(0, 1);
					coordx = std::stof(coordtextx);

					coordtexty = words[2];
					coordtexty.erase(0, 1);
					coordy = std::stof(coordtexty);

					// The coordinates are now stored in coordx and coordy

					// Create instruction for Motor task

					instruction.cmd = Instruction_E::G1;
					instruction.Xcoord = coordx;
					instruction.Ycoord = coordy;

					// Send instruction to Motor task

					xQueueSendToBack(instructionQueue, ( void * ) &instruction, portMAX_DELAY);

					// Create reply for UART task
					//   Contents: OK<CR><LF>

					snprintf(buffer, buffersize, "OK\r\n");

					// Send reply to UART task

					xQueueSendToBack(outputQueue, buffer, portMAX_DELAY);
				}
			} else if (words[0] == "G28") {
				// G28: Go to origin

				// Create instruction to Motor task

				instruction.cmd = Instruction_E::G28;

				// Send instruction to Motor task

				xQueueSendToBack(instructionQueue, ( void * ) &instruction, portMAX_DELAY);


				// No reply to UART task

			} else if (words[0] == "M1") {
				// M1: Set pen position

				bool good = true;
				int penposition = 0;

				// Must be 2 words (for example: M1 90)
				good &= words.size() == 2;

				if (good) {
					penposition = std::stoi(words[1]);
					if (penposition >= 0 && penposition <= 255) {
						// Range OK

						// Set new position
						
						// Create instruction for Motor task

						instruction.cmd = Instruction_E::M1;
						instruction.arg1 = (uint16_t)penposition;

						// Send instruction to Motor task

						xQueueSendToBack(instructionQueue, ( void * ) &instruction, portMAX_DELAY);


						// Create reply for UART task
						//   Contents: OK<CR><LF>

						snprintf(buffer, buffersize, "OK\r\n");

						// Send reply to UART task

						xQueueSendToBack(outputQueue, buffer, portMAX_DELAY);
					}
				}
			} else if (words[0] == "M2") {
				// M2: Save pen up/down position

				bool good = true;

				// Must be 3 words (for example: M2 U150 D90)
				good &= words.size() == 3;

				if (good) {
					// Parse pen/up down position

					int penpositionup = 0;
					int penpositiondown = 0;
					
					std::string penpositiontextup;
					std::string penpositiontextdown;

					penpositiontextup = words[1];
					penpositiontextup.erase(0, 1);
					penpositionup = std::stoi(penpositiontextup);

					penpositiontextdown = words[2];
					penpositiontextdown.erase(0, 1);
					penpositiondown = std::stoi(penpositiontextdown);

					// Pen position is now stored in penpositionup and penpositiondown

					// Create instruction for Motor task

					instruction.cmd = Instruction_E::M2;
					instruction.arg1 = (uint16_t)penpositionup;
					instruction.arg2 = (uint16_t)penpositiondown;

					// Send instruction to Motor task

					xQueueSendToBack(instructionQueue, ( void * ) &instruction, portMAX_DELAY);


					// Create reply for UART task
					//   Contents: OK<CR><LF>

					snprintf(buffer, buffersize, "OK\r\n");

					// Send reply to UART task

					xQueueSendToBack(outputQueue, buffer, portMAX_DELAY);
				}

			} else if (words[0] == "M4") {
				// M4: Set laser power
				bool good = true;
				int laserpower = 0;

				// Must be 2 words (for example: M4 122)
				good &= words.size() == 2;

				if (good) {
					laserpower = std::stoi(words[1]);
					if (laserpower >= 0 && laserpower <= 255) {
						// Range OK

						// Apply laser power

						// Create instruction for Motor task

						instruction.cmd = Instruction_E::M4;
						instruction.arg1 = (uint16_t)laserpower;

						// Send instruction to Motor task

						xQueueSendToBack(instructionQueue, ( void * ) &instruction, portMAX_DELAY);


						// No reply to UART task
					}
				}
			} else if (words[0] == "M5") {
				// M5: Save stepper directions

				bool good = true;

				// Must be 6 words (for example: M5 A0 B0 H310 W380 S80)
				good &= words.size() == 6;

				if (good) {
					// false: clockwise
					// true: counterclockwise
					bool directionx = false; // Stepper X axis direction
					bool directiony = false; // Stepper Y axis direction

					int plotareax = 0; // Plot area X (mm)
					int plotareay = 0; // Plot area Y (mm)

					int plotspeed = 0; // Plotting speed

					// Parse X and Y axis direction

					std::string directiontextx;
					std::string directiontexty;

					directiontextx = words[1];
					directiontextx.erase(0, 1);
					directionx = ((bool)std::stoi(directiontextx));

					directiontexty = words[2];
					directiontexty.erase(0, 1);
					directiony = ((bool)std::stoi(directiontexty));

					// Parse height and width

					std::string heighttext;
					std::string widthtext;

					heighttext = words[3];
					heighttext.erase(0, 1);
					plotareax = std::stoi(heighttext);

					widthtext = words[4];
					heighttext.erase(0, 1);
					plotareay = std::stoi(widthtext);

					// Parse speed

					std::string speedtext;

					speedtext = words[5];
					speedtext.erase(0, 1);
					plotspeed = std::stoi(speedtext);

					// Stepper X axis direction is now in directionx
					// Stepper Y axis direction is now in directiony
					// Plot area width is now in plotareax
					// Plot area height is now in plotareay
					// Plotting speed is now in plotspeed

					// Create instruction for Motor task

					instruction.cmd = Instruction_E::M5;
					instruction.arg1 = (uint16_t)directionx;
					instruction.arg2 = (uint16_t)directiony;
					instruction.arg3 = (uint16_t)plotareax;
					instruction.arg4 = (uint16_t)plotareay;
					instruction.arg5 = (uint16_t)plotspeed;

					// Send instruction to Motor task

					xQueueSendToBack(instructionQueue, ( void * ) &instruction, portMAX_DELAY);


					// Create reply for UART task
					//   Contents: OK<CR><LF>

					snprintf(buffer, buffersize, "OK\r\n");

					// Send reply to UART task

					xQueueSendToBack(outputQueue, buffer, portMAX_DELAY);
				}

			} else if (words[0] == "M10") {
				// M10: Begin plotting

				// Report M10 command and request parameters from Motor task

				instruction.cmd = Instruction_E::M10;
				xQueueSendToBack(instructionQueue, ( void * ) &instruction, portMAX_DELAY);

				// Get reply back from Motor task

//				instruction = { 0 };
				xQueueReceive(motorReplyQueue, &instruction, portMAX_DELAY);

				// Create reply for UART task
				//   For example:
				//   M10 XY 380 310 0.00 0.00 A0 B0 H0 S80 U160 D90<CR><LF>OK<CR><LF>

				int plotareax = (int)instruction.arg1; // Plot area X (mm)
				int plotareay = (int)instruction.arg2; // Plot area Y (mm)
				
				// false: clockwise
				// true: counterclockwise
				bool directionx = (bool)instruction.arg3; // Stepper X axis direction
				bool directiony = (bool)instruction.arg4; // Stepper Y axis direction

				int plotspeed = (int)instruction.arg5; // Plotting speed

				int penup = (int)instruction.arg6; // Pen up (0 to 255)
				int pendown = (int)instruction.arg7; // Pen down (0 to 255)

				snprintf(buffer, buffersize,
					"M10 XY %d %d %.2f %.2f A%d B%d H0 S%d U%d D%d\r\nOK\r\n",
					plotareax,
					plotareay,
					(float)0.0f,
					(float)0.0f,
					(int)directionx,
					(int)directiony,
					plotspeed,
					penup,
					pendown);

				// Send reply to UART task

				xQueueSendToBack(outputQueue, buffer, portMAX_DELAY);

			} else if (words[0] == "M11") {
				// M11: Limit switch status query

				// Report M10 command and request parameters from Motor task

				instruction.cmd = Instruction_E::M11;
				xQueueSendToBack(instructionQueue, ( void * ) &instruction, portMAX_DELAY);

				// Get reply back from Motor task

//				instruction = { 0 };
				xQueueReceive(motorReplyQueue, &instruction, portMAX_DELAY);

				// Create reply for UART task
				//   For example:
				//   M11 1 1 1 1<CR><LF>OK<CR><LF>

				// false: switch closed
				// true: switch open
				bool l4 = (bool)instruction.arg4;
				bool l3 = (bool)instruction.arg3;
				bool l2 = (bool)instruction.arg2;
				bool l1 = (bool)instruction.arg1;

				snprintf(buffer, buffersize,
					"M11 %d %d %d %d\r\nOK\r\n",
					(int)l4,
					(int)l3,
					(int)l2,
					(int)l1);

				// Send reply to UART task

				xQueueSendToBack(outputQueue, buffer, portMAX_DELAY);
			}
		}
	}
}
