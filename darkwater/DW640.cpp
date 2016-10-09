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
    if (!testConnection() ) {
    	printf("No 640 board found\n");
    	return 0;		
    }
    // set the default frequency
    setFrequency( 100 );
    // Create the mode pin
    this->modePin = new Pin( RPI_GPIO_27 );
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