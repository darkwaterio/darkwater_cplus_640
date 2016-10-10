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

#include "DW640.h"

/** PCA9685 constructor.
 * @param address I2C address
 * @see DW640_DEFAULT_ADDRESS
 */
DW640::DW640(uint8_t address) {
    this->devAddr = address;
}

/** Power on and prepare for general usage.
 * This method reads prescale value stored in PCA9685 and calculate frequency based on it.
 * Then it enables auto-increment of register address to allow for faster writes.
 * And finally the restart is performed to enable clocking.
 */
bool DW640::initialize() {
    this->pwm = new PCA9685( this->devAddr );
    this->pwm->initialize();
    if (!testConnection() ) {
    	printf("No 640 board found\n");
    	return 0;		
    }
    // set the default frequency
    setFrequency( 100 );
    // Create the mode pin
    this->modePin = new Pin( RPI_GPIO_27 );
    if (!this->modePin->init()) {
    	fprintf(stderr, "Pin Mode can not be set. Are you root?");
    	return 0;
    }
    this->modePin->setMode(Pin::GpioModeOutput);
    // set the default mode to ININ
    setMode( DW_ININ );
    
}

/** Verify the I2C connection.
 * @return True if connection is valid, false otherwise
 */
bool DW640::testConnection() {
    return this->pwm->testConnection();
}

/** Calculate prescale value based on the specified frequency and write it to the device.
 * @return Frequency in Hz
 */
float DW640::getFrequency() {
    return this->pwm->getFrequency();
}


/** Calculate prescale value based on the specified frequency and write it to the device.
 * @param Frequency in Hz
 */
void DW640::setFrequency(float frequency) {
    this->pwm->setFrequency( frequency );
}

/** Calculate prescale value based on the specified frequency and write it to the device.
 * @return Frequency in Hz
 */
uint8_t DW640::getMode() {
    return this->mode;
}


/** Calculate prescale value based on the specified frequency and write it to the device.
 * @param Frequency in Hz
 */
void DW640::setMode(uint8_t mode) {
    this->modePin->write( mode );
    this->mode = mode;
}

/** Set channel start offset of the pulse and it's length
 * @param Channel number (0-15)
 * @param Offset (0-4095)
 * @param Length (0-4095)
 */
void DW640::setPWM(uint8_t channel, uint16_t offset, uint16_t length) {
	this->pwm->setPWM( channel, offset, length );
}

/** Set channel's pulse length
 * @param Channel number (0-15)
 * @param Length (0-4095)
 */
void DW640::setPWM(uint8_t channel, uint16_t length) {
    this->pwm->setPWM(channel, length);
}

/** Set channel's pulse length in milliseconds
 * @param Channel number (0-15)
 * @param Length in milliseconds
 */
void DW640::setPWMmS(uint8_t channel, float length_mS) {
    this->pwm->setPWMmS(channel, length_mS);
}

/** Set channel's pulse length in microseconds
 * @param Channel number (0-15)
 * @param Length in microseconds
 */
void DW640::setPWMuS(uint8_t channel, float length_uS) {
    this->pwm->setPWMuS(channel, length_uS);
}

/** Set start offset of the pulse and it's length for all channels
 * @param Offset (0-4095)
 * @param Length (0-4095)
 */
void DW640::setAllPWM(uint16_t offset, uint16_t length) {
	this->pwm->setAllPWM(offset, length);
}

/** Set pulse length for all channels
 * @param Length (0-4095)
 */
void DW640::setAllPWM(uint16_t length) {
    this->pwm->setAllPWM(length);
}

/** Set pulse length in milliseconds for all channels
 * @param Length in milliseconds
 */
void DW640::setAllPWMmS(float length_mS) {
	this->pwm->setAllPWMmS(length_mS);
}

/** Set pulse length in microseconds for all channels
 * @param Length in microseconds
 */
void DW640::setAllPWMuS(float length_uS) {
	this->pwm->setAllPWMuS(length_uS);
}

void DW640::setPin(uint8_t channel, uint8_t value) {

	if( channel < 0 || channel > 15 ) {
		fprintf(stderr, "PWM pin must be between 0 and 15 inclusive");
	}

	if( value == 0 ) {
		setPWM( channel, 0, 4096 );
	} else if( value == 1 ) {
		setPWM( channel, 4096, 0 );
	} else {
		fprintf(stderr, "Pin value must be 0 or 1!");
	}
}

void DW640::setAllPin(uint8_t value) {

	if( value == 0 ) {
		setAllPWM( 0, 4096 );
	} else if( value == 1 ) {
		setAllPWM( 4096, 0 );
	} else {
		fprintf(stderr, "Pin value must be 0 or 1!");
	}

}

void DW640::allOff() {
	setAllPin( 0 );
}

/* DC Motor specific code */

DW_Motor *DW640::getMotor(uint8_t motor) {

	uint8_t in1;
	uint8_t in2;

	motor--;

	// Get the motor
	switch(motor) {
		case 0:
				in2 = 2;
				in1 = 3;
				break;
		case 1:
				in2 = 4;
				in1 = 5;
				break;
		case 2:
				in2 = 6;
				in1 = 7;
				break;
		case 3:
				in2 = 8;
				in1 = 9;
				break;
		case 4:
				in2 = 10;
				in1 = 11;
				break;
		case 5:
				in2 = 12;
				in1 = 13;
				break;
		default:
				fprintf(stderr, "Motor number must be between 1 and 6 inclusive");
	}

	if(motors[motor].motor == 0) {
		// We don't have one yet
		motors[motor].motor = motor; // how many motors on one line?!?
		motors[motor].DWC = this;

		motors[motor].in1 = in1;
		motors[motor].in2 = in2;
	}

	return &motors[motor];

}

DW_Servo *DW640::getServo(uint8_t servo) {

	uint8_t pin;

	servo--;

	// Get the servo
	switch(servo) {
		case 0:
				pin = 0; // Servo 1 is on PWM 0
				break;
		case 1:
				pin = 1; // Servo 2 is on PWM 1
				break;
		default:
				fprintf(stderr, "Servo number must be between 1 and 2 inclusive");
	}

	if(servos[servo].servo == 0) {
		// We don't have one yet
		servos[servo].servo = servo; // how many servos on one line?!?
		servos[servo].DWC = this;

		servos[servo].pin = pin;
	}

	return &servos[servo];

}

/* Motors code */

DW_Motor::DW_Motor(void) {
  DWC = NULL;
  motor = 0;
  in1 = in2 = 0;
}

void DW_Motor::setMotorSpeed(int16_t speed) {

	// Speed deciphering for the two control modes
	if( speed >= 1000 && speed < 1500 ) {
		run( DW_REVERSE, map(speed, 1500, 1000, 0, 255 ) );
		} else if( speed > 1500 && speed <= 2000 ) {	
			run( DW_FORWARD, map(speed, 1500, 2000, 0, 255 ) );
			} else if( speed > 0 && speed <= 255 ) {
				run( DW_FORWARD, speed );
				} else if( speed < 0 && speed >= -255 ) {
					run( DW_REVERSE, abs(speed) );
					} else if( speed == 0 || speed == 1500 ) {
						run( DW_STOP, speed );
					}

	

}

void DW_Motor::off(void) {
	setMotorSpeed( 0 );
}

void DW_Motor::run( uint8_t control, uint16_t speed ) {
	// get the mode
	if( DWC->getMode() == DW_PHASE ) {
		if( control == DW_FORWARD ) {
				DWC->setPin( in2, 0 );
				DWC->setPWM( in1, 0, speed * 16 );
			} else if( control == DW_REVERSE ) {
					DWC->setPin( in2, 1 );
					DWC->setPWM( in1, 0, speed * 16 );
				} else if( control == DW_STOP ) {
					DWC->setPin( in2, 0 );
					DWC->setPin( in1, 0 );
				}
	} else {	// DW_ININ
		if( control == DW_FORWARD ) {
			DWC->setPin( in2, 0 );
			DWC->setPWM( in1, 0, speed * 16 );
			} else if( control == DW_REVERSE ) {
				DWC->setPin( in1, 0 );
				DWC->setPWM( in2, 0, speed * 16 );
				} else if( control == DW_STOP ) {
					DWC->setPin( in1, 1 );
					DWC->setPin( in2, 1 );
					} else if( control == DW_COAST ) {
						DWC->setPin( in1, 0 );
						DWC->setPin( in2, 0 );
					}
	}
}

uint16_t DW_Motor::map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Servo code */

DW_Servo::DW_Servo(void) {
  DWC = NULL;
  servo = 0;
  pin = 0;
}

void DW_Servo::off(void) {

	DWC->setPin( pin, 0 );

}
    
void DW_Servo::setPWMmS(float length_mS) {

	DWC->setPWMmS( pin, length_mS ); // Servo 1 is on PWM 0

}

void DW_Servo::setPWMuS(float length_uS) {

	DWC->setPWMuS( pin, length_uS ); // Servo 1 is on PWM 0
	

}

/* Stepper functions */

// void DW640::setStepperOff(uint8_t stepper) {

// }

// void DW640::setStepperSpeed(uint8_t stepper, uint16_t speed) {

// }

// void DW640::oneStep(uint8_t stepper, uint8_t direction, uint8_t style) {

// }

// void DW640::step(uint8_t stepper, uint16_t steps, uint8_t direction, uint8_t style) {

// }

/* Private functions */

