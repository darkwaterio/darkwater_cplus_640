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

	dw.setMotorOff(1);
	dw.setMotorOff(2);
	dw.setMotorOff(3);
	dw.setMotorOff(4);
	dw.setMotorOff(5);
	dw.setMotorOff(6);
	usleep(1000000);

	printf("Set forward - \n");
	printf("Motor 1\n");
	dw.setMotorSpeed( 1, 255 );
	usleep(1000000);
	printf("Motor 2\n");
	dw.setMotorSpeed( 2, 255 );
	usleep(1000000);
	printf("Motor 3\n");
	dw.setMotorSpeed( 3, 255 );
	usleep(1000000);
	printf("Motor 4\n");
	dw.setMotorSpeed( 4, 255 );
	usleep(1000000);
	printf("Motor 5\n");
	dw.setMotorSpeed( 5, 255 );
	usleep(1000000);
	printf("Motor 6\n");
	dw.setMotorSpeed( 6, 255 );
	usleep(1000000);
	printf("Stopping - \n");
	printf("Motor 1\n");
	dw.setMotorSpeed( 1, 0 );
	usleep(1000000);
	printf("Motor 2\n");
	dw.setMotorSpeed( 2, 0 );
	usleep(1000000);
	printf("Motor 3\n");
	dw.setMotorSpeed( 3, 0 );
	usleep(1000000);
	printf("Motor 4\n");
	dw.setMotorSpeed( 4, 0 );
	usleep(1000000);
	printf("Motor 5\n");
	dw.setMotorSpeed( 5, 0 );
	usleep(1000000);
	printf("Motor 6\n");
	dw.setMotorSpeed( 6, 0 );
	usleep(1000000);
	printf("Set reverse - \n");
	printf("Motor 1\n");
	dw.setMotorSpeed( 1, -255 );
	usleep(1000000);
	printf("Motor 2\n");
	dw.setMotorSpeed( 2, -255 );
	usleep(1000000);
	printf("Motor 3\n");
	dw.setMotorSpeed( 3, -255 );
	usleep(1000000);
	printf("Motor 4\n");
	dw.setMotorSpeed( 4, -255 );
	usleep(1000000);
	printf("Motor 5\n");
	dw.setMotorSpeed( 5, -255 );
	usleep(1000000);
	printf("Motor 6\n");
	dw.setMotorSpeed( 6, -255 );
	usleep(1000000);
	printf("All off \n");
	dw.setMotorOff(1);
	dw.setMotorOff(2);
	dw.setMotorOff(3);
	dw.setMotorOff(4);
	dw.setMotorOff(5);
	dw.setMotorOff(6);

}
