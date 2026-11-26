//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#include "misc.h"
#include <cstdio>
#include <cstdlib>

uint32_t micro_timestamp = 0;

void setTimerMs(uint16_t *timer)
{
	*timer = micro_timestamp>>10;/// (LPC_TIMER2->TC)>>10;
}

uint16_t getTimerMs(uint16_t timer)
{
	uint16_t result, t;
	t = micro_timestamp>>10;///(LPC_TIMER2->TC)>>10;
	result = t-timer;	

	return result;
}

void showError(uint8_t num, uint32_t color, const char *message)
{

	printf("%s\n", message);
	exit(1);
}
