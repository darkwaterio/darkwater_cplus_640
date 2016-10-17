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

	DW_Stepper *st1 = dw.getStepper(1);
	DW_Stepper *st2 = dw.getStepper(2);
	DW_Stepper *st3 = dw.getStepper(3);

	st1->off();
	st2->off();
	st3->off();
	usleep(1000000);

	st1->setMotorSpeed(200);
	st2->setMotorSpeed(200);
	st3->setMotorSpeed(200);

	printf("1 Forward\n");
	st1->step( 400, DW_FORWARD, DW_SINGLE );
	printf("2 Forward\n");
	st2->step( 400, DW_FORWARD, DW_SINGLE );
	printf("3 Forward\n");
	st3->step( 400, DW_FORWARD, DW_SINGLE );

	printf("1 Reverse\n");
	st1->step( 400, DW_REVERSE, DW_SINGLE );
	printf("2 Reverse\n");
	st2->step( 400, DW_REVERSE, DW_SINGLE );
	printf("3 Reverse\n");
	st3->step( 400, DW_REVERSE, DW_SINGLE );
	
	st1->off();
	st2->off();
	st3->off();
}
