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

void DW640::setMotorSpeed(uint8_t motor, int16_t speed) {

	uint8_t in1;
	uint8_t in2;

	// Get the motor
	switch(motor) {
		case 1:
				in2 = 2;
				in1 = 3;
				break;
		case 2:
				in2 = 4;
				in1 = 5;
				break;
		case 3:
				in2 = 6;
				in1 = 7;
				break;
		case 4:
				in2 = 8;
				in1 = 9;
				break;
		case 5:
				in2 = 10;
				in1 = 11;
				break;
		case 6:
				in2 = 12;
				in1 = 13;
				break;
		default:
				fprintf(stderr, "Motor number must be between 1 and 6 inclusive");
	}
	// Speed deciphering for the two control modes
	if( speed >= 1000 && speed < 1500 ) {
		printf( "%d - %d\n", speed, map(speed, 1500, 1000, 0, 255 ) );
		} else if( speed > 1500 && speed <= 2000 ) {	
			printf( "%d - %d\n", speed, map(speed, 1500, 2000, 0, 255 ) );
			} else if( speed > 0 && speed <= 255 ) {
				runMotor( DW_FORWARD, in1, in2, speed );
				} else if( speed < 0 && speed >= -255 ) {
					runMotor( DW_REVERSE, in1, in2, abs(speed) );
					} else if( speed == 0 || speed == 1500 ) {
						runMotor( DW_STOP, in1, in2, speed );
					}

	

}

void DW640::setMotorOff(uint8_t motor) {

}

void DW640::runMotor( uint8_t control, uint8_t in1, uint8_t in2, uint16_t speed ) {
	// get the mode
	if( getMode() == DW_PHASE ) {
		if( control == DW_FORWARD ) {
				setPin( in2, 0 );
				setPWM( in1, 0, speed * 16 );
			} else if( control == DW_REVERSE ) {
					setPin( in2, 1 );
					setPWM( in1, 0, speed * 16 );
				} else if( control == DW_STOP ) {
					setPin( in2, 0 );
					setPin( in1, 0 );
				}
	} else {	// DW_ININ
		if( control == DW_FORWARD ) {
			setPin( in2, 0 );
			setPWM( in1, 0, speed * 16 );
			} else if( control == DW_REVERSE ) {
				setPin( in1, 0 );
				setPWM( in2, 0, speed * 16 );
				} else if( control == DW_STOP ) {
					setPin( in1, 1 );
					setPin( in2, 1 );
					} else if( control == DW_COAST ) {
						setPin( in1, 0 );
						setPin( in2, 0 );
					}
	}
}

/* Private functions */

uint16_t DW640::map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}