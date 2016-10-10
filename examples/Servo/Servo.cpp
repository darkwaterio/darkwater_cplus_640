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

    DW_Servo *s1 = dw.getServo(1);
    DW_Servo *s2 = dw.getServo(2);

    s1->off();
    s2->off();

    printf("Start servo moves\n");
    for( int a = 10; a >= 0; a-- ) {
        printf("Step %d\n", a);
        s1->setPWMmS(SERVO_MIN);
        s2->setPWMmS(SERVO_MIN);
        usleep(1000000);
        s1->setPWMmS(SERVO_MAX);
        s2->setPWMmS(SERVO_MAX);
        usleep(1000000);
    }

    s1->off();
    s2->off();

    return 0;
}
