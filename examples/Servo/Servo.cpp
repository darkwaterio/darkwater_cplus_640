#define SERVO_MIN 1.250 /*mS*/
#define SERVO_MAX 1.750 /*mS*/

#include "darkwater/DW640.h"
#include "darkwater/Util.h"
#include <stdlib.h>

using namespace DarkWater;

int main()
{
    
    if (check_apm()) {
        return 1;
    }

    DW640 dw;
    dw.initialize();
    dw.setFrequency(50);

    dw.setServoOff( 1 );
    dw.setServoOff( 2 );

    for( int a = 10; a >= 0; a-- ) {
        dw.setServoPWMmS(1, SERVO_MIN);
        dw.setServoPWMmS(2, SERVO_MIN);
        usleep(1000000);
        dw.setServoPWMmS(1, SERVO_MAX);
        dw.setServoPWMmS(2, SERVO_MAX);
        usleep(1000000);
    }

    dw.setServoOff( 1 );
    dw.setServoOff( 2 );

    return 0;
}
