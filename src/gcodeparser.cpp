#include <vector>
#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*
	Queues:

	inputQueue
	xQueueFromUART (item type *char)
		UART puts a received command in this queue and GCodeParser picks it up
		for parsing.

	instructionQueue
	xQueueToUART (item type *char)
		GCodeParser puts a reply in this queue and UART picks it up for
		sending.

	The strings are allocated using pvPortMalloc() and must be freed with
	pvPortFree() after receiving.

*/
QueueHandle_t inputQueue;
QueueHandle_t instructionQueue;

void vTaskGCodeParser (void *pvParameters) {
	const unsigned int buffersize = 128;
	char *buffer;
	std::string word;
	std::vector<std::string> words;

	while(1) {
		words.clear();
		word.clear();

		xQueueReceive(xQueueFromUART, buffer, portMAX_DELAY);

		for (unsigned int c = 0; c < (strlen(buffer) + 1); c++) {
			if (buffer[c] == ' ' || buffer[c] == 0) {
				words.push_back(word);
				word.clear();
			} else {
				word.push_back(buffer[c]);
			}
		}

		pvPortFree(buffer);

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

					double coordx = 0.0f;
					double coordy = 0.0f;
					
					std::string coordtextx;
					std::string coordtexty;

					coordtextx = words[1];
					coordtextx.erase(0, 1);
					coordx = std::stod(coordtextx);

					coordtexty = words[2];
					coordtexty.erase(0, 1);
					coordy = std::stod(coordtexty);

					// The coordinates are now stored in coordx and coordy



					// Create reply
					//   Contents: OK<CR><LF>

					char *reply = pvPortMalloc(buffersize * sizeof(char));

					snprintf(reply, buffersize,
						"OK\r\n");

					// Send reply

					xQueueSendToBack(xQueueToUART, reply, portMAX_DELAY);
				}
			} else if (words[0] == "G28") {
				// G28: Go to origin

				// Go to origin


				// No reply

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
						

						// Create reply
						//   Contents: OK<CR><LF>

						char *reply = pvPortMalloc(buffersize * sizeof(char));

						snprintf(reply, buffersize,
							"OK\r\n");

						// Send reply

						xQueueSendToBack(xQueueToUART, reply, portMAX_DELAY);
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



					// Create reply
					//   Contents: OK<CR><LF>

					char *reply = pvPortMalloc(buffersize * sizeof(char));

					snprintf(reply, buffersize,
						"OK\r\n");

					// Send reply

					xQueueSendToBack(xQueueToUART, reply, portMAX_DELAY);
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


						// No reply
					}
				}
			} else if (words[0] == "M5") {
				// M5: Save stepper directions

				bool good = true;

				// Must be 6 words (M5 A0 B0 H310 W380 S80)
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


					// Create reply
					//   Contents: OK<CR><LF>

					char *reply = pvPortMalloc(buffersize * sizeof(char));

					snprintf(reply, buffersize,
						"OK\r\n");

					// Send reply

					xQueueSendToBack(xQueueToUART, reply, portMAX_DELAY);
				}

			} else if (words[0] == "M10") {
				// M10: Begin plotting

				// Create reply
				//   For example:
				//   M10 XY 380 310 0.00 0.00 A0 B0 H0 S80 U160 D90<CR><LF>OK<CR><LF>

				char *reply = pvPortMalloc(buffersize * sizeof(char));

				int plotareax = 0; // Plot area X (mm)
				int plotareay = 0; // Plot area Y (mm)
				
				// false: clockwise
				// true: counterclockwise
				bool directionx = false; // Stepper X axis direction
				bool directiony = false; // Stepper Y axis direction

				int plotspeed = 0; // Plotting speed

				int penup = 0; // Pen up (0 to 255)
				int pendown = 0; // Pen down (0 to 255)

				snprintf(reply, buffersize,
					"M10 XY %d %d %.2f %.2f A%d B%d H0 S%d U%d D%d\r\nOK\r\n",
					plotareax,
					plotareay,
					(double)0.0f,
					(double)0.0f,
					(int)directionx,
					(int)directiony,
					plotspeed,
					penup,
					pendown);

				// Send reply

				xQueueSendToBack(xQueueToUART, reply, portMAX_DELAY);

			} else if (words[0] == "M11") {
				// M11: Limit switch status query

				// Create reply
				//   For example:
				//   M11 1 1 1 1<CR><LF>OK<CR><LF>

				// false: switch closed
				// true: switch open
				bool l4 = false;
				bool l3 = false;
				bool l2 = false;
				bool l1 = false;

				char *reply = pvPortMalloc(buffersize * sizeof(char));

				snprintf(reply, buffersize,
					"M11 %d %d %d %d\r\nOK\r\n",
					(int)l4,
					(int)l3,
					(int)l2,
					(int)l1);

				// Send reply

				xQueueSendToBack(xQueueToUART, reply, portMAX_DELAY);
			}
		}
	}
}
