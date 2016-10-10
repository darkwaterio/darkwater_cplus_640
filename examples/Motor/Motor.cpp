/*
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Read accelerometer, gyroscope and magnetometer values from
MPU9250 inertial measurement unit over SPI on Raspberry Pi + Navio.

Navio's onboard MPU9250 is connected to the SPI bus on Raspberry Pi
and can be read through /dev/spidev0.1

To run this example navigate to the directory containing it and run following commands:
make
./AccelGyroMag
*/

#include "darkwater/DW640.h"
#include "darkwater/Util.h"
#include <stdlib.h>
//=============================================================================

int main()
{
    if (check_apm()) {
        return 1;
    }
	//-------------------------------------------------------------------------
	DW640 dw;
	dw.initialize();

	dw.setMotorSpeed( 1, 1500 );
	printf("forward\n");
	dw.setMotorSpeed( 1, 2000 );
	usleep(2000000);
	dw.setMotorSpeed( 1, 1950 );
	usleep(2000000);
	dw.setMotorSpeed( 1, 1900 );
	usleep(2000000);
	dw.setMotorSpeed( 1, 1800 );
	usleep(2000000);
	dw.setMotorSpeed( 1, 1700 );
	usleep(2000000);
	dw.setMotorSpeed( 1, 1600 );
	usleep(2000000);
	printf("stop\n");
	dw.setMotorSpeed( 1, 1500 );
	usleep(5000000);
	printf("reverse\n");
	dw.setMotorSpeed( 1, 1250 );
	usleep(5000000);
	printf("stop\n");
	dw.setMotorSpeed( 1, 1500 );


}
