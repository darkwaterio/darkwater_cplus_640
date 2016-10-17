/*
Dark Water 640 driver code is placed under the BSD license.
Written by Team Dark Water (team@darkwater.io) based off libraries by Adafruit https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
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

#define MICROSTEPS 16         // 8 or 16

using namespace DarkWater;

class DW640;

class DW_Motor {
    public:
        DW_Motor(void);
        friend class DW640;

        void setMotorSpeed(int16_t speed);
        void off(void);
        void run(uint8_t control, uint16_t speed );

    private:
        uint8_t in1, in2;
        DW640 *DWC;
        uint8_t motor;

        uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

};

class DW_Servo {
    public:
        DW_Servo(void);
        friend class DW640;

        void off(void);
        void setPWMmS(float length_mS);
        void setPWMuS(float length_uS);

    private:
        uint8_t pin;
        DW640 *DWC;
        uint8_t servo;

};

class DW_Stepper {
    public:
        DW_Stepper(void);
        friend class DW640;

        void off(void);
        void setMotorSpeed(uint16_t speed);
        uint8_t onestep(uint8_t dir, uint8_t style);
        void step(uint16_t steps, uint8_t dir,  uint8_t style = DW_SINGLE);
        uint32_t usperstep;

    private:
        uint8_t AIN1pin, AIN2pin;
        uint8_t BIN1pin, BIN2pin;
        uint16_t revsteps; // # steps per revolution
        uint8_t currentstep;
        uint8_t stepper;
        DW640 *DWC;

};

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

        DW_Motor *getMotor(uint8_t motor);
        DW_Servo *getServo(uint8_t servo);      
        DW_Stepper *getStepper(uint8_t stepper, uint16_t steps = 48);  

     private:
        uint8_t devAddr;
        float frequency;
        uint8_t mode;
        PCA9685* pwm;
        Pin* modePin;

        DW_Motor motors[6];
        DW_Servo servos[2];
        DW_Stepper steppers[3];

};

#endif // DW640_H