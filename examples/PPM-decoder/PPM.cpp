/*
Example code is placed under the BSD license.
Written by Team Dark Water (team@darkwater.io) - based on original by Emlid
Copyright (c) 2014, Emlid Limited. All rights reserved.
Written by Mikhail Avkhimenia (mikhail.avkhimenia@emlid.com)
twitter.com/emlidtech || www.emlid.com || info@emlid.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Emlid Limited nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <pigpio.h>
#include <stdio.h>
#include <unistd.h>

#include "darkwater/DW640.h"
#include "darkwater/Util.h"
#include <stdlib.h>

//================================ Options =====================================

unsigned int samplingRate      = 1;      // 1 microsecond (can be 1,2,4,5,10)
unsigned int ppmInputGpio      = 4;      // PPM input on Navio's 2.54 header
unsigned int ppmSyncLength     = 4000;   // Length of PPM sync pause
unsigned int ppmChannelsNumber = 6;      // Number of channels packed in PPM
unsigned int motorFrequency    = 100;     // Servo control frequency
bool verboseOutputEnabled      = true;   // Output channels values to console

//============================ Objects & data ==================================

using namespace DarkWater;

DW640 *dw;
DW_Motor *motors[6];

float channels[6];

//============================== PPM decoder ===================================

unsigned int currentChannel = 0;
unsigned int previousTick;
unsigned int deltaTime;

void ppmOnEdge(int gpio, int level, uint32_t tick)
{
	if (level == 0) {	
		deltaTime = tick - previousTick;
		previousTick = tick;
	
		if (deltaTime >= ppmSyncLength) { // Sync
			currentChannel = 0;

			// RC output
			for (int i = 0; i < ppmChannelsNumber; i++) {
				// Because we aren't going to get exact readings as this may not be running with a RT kernel
				// We need to add some sanity checks for values
				if( channels[i] > 2000 ) channels[i] = 2000; // Set top forward speed
				if( channels[i] < 1000 ) channels[i] = 1000; // Set top reverse speed
				if( channels[i] < 1550 && channels[i] > 1450 ) channels[i] = 1500; // Add a dead band around center
				// Set motor speed
				motors[i]->setMotorSpeed( channels[i] );
			}

			// Console output
			if (verboseOutputEnabled) {
				printf("\n");
				for (int i = 0; i < ppmChannelsNumber; i++)
					printf("%4.f ", channels[i]);
			}
		}
		else
			if (currentChannel < ppmChannelsNumber)
				channels[currentChannel++] = deltaTime;
	}
}

//================================== Main ======================================

int main(int argc, char *argv[])
{
    if (check_apm()) {
        return 1;
    }
    
    dw = new DW640();
    dw->initialize();
    dw->setFrequency(motorFrequency);

    motors[0] = dw->getMotor(1);
    motors[1] = dw->getMotor(2);
    motors[2] = dw->getMotor(3);
    motors[3] = dw->getMotor(4);
    motors[4] = dw->getMotor(5);
    motors[5] = dw->getMotor(6);

	// GPIO setup
	gpioCfgClock(samplingRate, PI_DEFAULT_CLK_PERIPHERAL, 0); /* last parameter is deprecated now */
	gpioInitialise();
	previousTick = gpioTick();
	gpioSetAlertFunc(ppmInputGpio, ppmOnEdge);

	// Infinite sleep - all action is now happening in ppmOnEdge() function

	while(1)
		sleep(10);
	return 0;
}
