/*
Example code is placed under the BSD license.
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
