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

#define DW640_DEFAULT_ADDRESS     0x60 // 640 default

#define DW_FORWARD                0x00
#define DW_REVERSE                0x01
#define DW_BRAKEFORWARD           0x06
#define DW_BRAKEREVERSE           0x07
#define DW_STOP                   0x08
#define DW_COAST                  0x07

#define DW_SINGLE                 0x07
#define DW_DOUBLE                 0x07
#define DW_INTERLEAVE             0x07
#define DW_MICROSTEP              0x07

#define DW_ININ                   0x07
#define DW_PHASE                  0x07

class DW640 {
    public:
        DW640(uint8_t address = DW640_DEFAULT_ADDRESS);

        void initialize();
        bool testConnection();

        float getMode();
        void setMode(float frequency);

        void setPin(uint8_t channel, uint16_t value);
        void setAllPin(uint16_t value);

        void setPWM(uint8_t channel, uint16_t offset, uint16_t length);
        void setPWM(uint8_t channel, uint16_t length);
        void setPWMmS(uint8_t channel, float length_mS);
        void setPWMuS(uint8_t channel, float length_uS);

        void setAllPWM(uint16_t offset, uint16_t length);
        void setAllPWM(uint16_t length);
        void setAllPWMmS(float length_mS);
        void setAllPWMuS(float length_uS);

        void allOff(uint16_t value);

        void setMotorSpeed(uint8_t motor, uint16_t speed);
        void setMotorOff(uint8_t motor);

        void setServoOff(uint8_t servo);
        void setServoPWMmS(uint8_t servo, float length_mS);
        void setServoPWMuS(uint8_t servo, float length_uS);

        void setStepperOff(uint8_t stepper);
        void setStepperSpeed(uint8_t stepper, uint16_t speed);
        void oneStep(uint8_t stepper, uint8_t direction, uint8_t style);
        void step(uint8_t stepper, uint16_t steps, uint8_t direction, uint8_t style);

     private:
        uint8_t devAddr;
        float frequency;
        uint8_t mode;
};

#endif // DW640_H