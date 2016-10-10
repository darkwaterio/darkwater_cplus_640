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

	DW_Motor *dw1 = dw.getMotor(1);
	DW_Motor *dw2 = dw.getMotor(2);
	DW_Motor *dw3 = dw.getMotor(3);
	DW_Motor *dw4 = dw.getMotor(4);
	DW_Motor *dw5 = dw.getMotor(5);
	DW_Motor *dw6 = dw.getMotor(6);

	dw1->off();
	dw2->off();
	dw3->off();
	dw4->off();
	dw5->off();
	dw6->off();
	usleep(1000000);

	printf("Set forward - \n");
	printf("Motor 1\n");
	dw1->setMotorSpeed( 255 );
	usleep(1000000);
	printf("Motor 2\n");
	dw2->setMotorSpeed( 255 );
	usleep(1000000);
	printf("Motor 3\n");
	dw3->setMotorSpeed( 255 );
	usleep(1000000);
	printf("Motor 4\n");
	dw4->setMotorSpeed( 255 );
	usleep(1000000);
	printf("Motor 5\n");
	dw5->setMotorSpeed( 255 );
	usleep(1000000);
	printf("Motor 6\n");
	dw6->setMotorSpeed( 255 );
	usleep(1000000);
	printf("Stopping - \n");
	printf("Motor 1\n");
	dw1->setMotorSpeed( 0 );
	usleep(1000000);
	printf("Motor 2\n");
	dw2->setMotorSpeed( 0 );
	usleep(1000000);
	printf("Motor 3\n");
	dw3->setMotorSpeed( 0 );
	usleep(1000000);
	printf("Motor 4\n");
	dw4->setMotorSpeed( 0 );
	usleep(1000000);
	printf("Motor 5\n");
	dw5->setMotorSpeed( 0 );
	usleep(1000000);
	printf("Motor 6\n");
	dw6->setMotorSpeed( 0 );
	usleep(1000000);
	printf("Set reverse - \n");
	printf("Motor 1\n");
	dw1->setMotorSpeed( -255 );
	usleep(1000000);
	printf("Motor 2\n");
	dw2->setMotorSpeed( -255 );
	usleep(1000000);
	printf("Motor 3\n");
	dw3->setMotorSpeed( -255 );
	usleep(1000000);
	printf("Motor 4\n");
	dw4->setMotorSpeed( -255 );
	usleep(1000000);
	printf("Motor 5\n");
	dw5->setMotorSpeed( -255 );
	usleep(1000000);
	printf("Motor 6\n");
	dw6->setMotorSpeed( -255 );
	usleep(1000000);
	printf("All off \n");
	dw1->off();
	dw2->off();
	dw3->off();
	dw4->off();
	dw5->off();
	dw6->off();

}
