/*
Dark Water 640 driver code is placed under the BSD license.
Written by Team Dark Water (team@darkwater.io)
Copyright (c) 2014, Dark Water
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

#ifndef DW640_H
#define DW640_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include "PCA9685.h"
#include "gpio.h"

#define DW640_DEFAULT_ADDRESS     0x60 // 640 default

#define DW_FORWARD                1
#define DW_REVERSE                2
#define DW_BRAKEFORWARD           3
#define DW_BRAKEREVERSE           4
#define DW_STOP                   5
#define DW_COAST                  6

#define DW_SINGLE                 1
#define DW_DOUBLE                 2
#define DW_INTERLEAVE             3
#define DW_MICROSTEP              4

#define DW_ININ                   0
#define DW_PHASE                  1

using namespace DarkWater;

class DW640 {
    public:
        DW640(uint8_t address = DW640_DEFAULT_ADDRESS);

        bool initialize();
        bool testConnection();

        float getFrequency();
        void setFrequency(float frequency);

        uint8_t getMode();
        void setMode(uint8_t mode);

        void setPWM(uint8_t channel, uint16_t offset, uint16_t length);
        void setPWM(uint8_t channel, uint16_t length);
        void setPWMmS(uint8_t channel, float length_mS);
        void setPWMuS(uint8_t channel, float length_uS);

        void setAllPWM(uint16_t offset, uint16_t length);
        void setAllPWM(uint16_t length);
        void setAllPWMmS(float length_mS);
        void setAllPWMuS(float length_uS);

        void setPin(uint8_t channel, uint8_t value);
        void setAllPin(uint8_t value);

        void allOff();

        void setMotorSpeed(uint8_t motor, int16_t speed);
        void setMotorOff(uint8_t motor);
        void runMotor( uint8_t control, uint8_t in1, uint8_t in2, uint16_t speed );

        // void setServoOff(uint8_t servo);
        // void setServoPWMmS(uint8_t servo, float length_mS);
        // void setServoPWMuS(uint8_t servo, float length_uS);

        // void setStepperOff(uint8_t stepper);
        // void setStepperSpeed(uint8_t stepper, uint16_t speed);
        // void oneStep(uint8_t stepper, uint8_t direction, uint8_t style);
        // void step(uint8_t stepper, uint16_t steps, uint8_t direction, uint8_t style);

     private:
        uint8_t devAddr;
        float frequency;
        uint8_t mode;
        PCA9685* pwm;
        Pin* modePin;


};

#endif // DW640_H