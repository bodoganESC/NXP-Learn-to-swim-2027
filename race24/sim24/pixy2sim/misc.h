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

#ifndef _MISC_H
#define _MISC_H

#include <inttypes.h>

// for stringification of preprocessor values
#define STR(s)           #s
#define STRINGIFY(s)     STR(s)

#ifdef __cplusplus
extern "C"
{
#endif

extern uint32_t micro_timestamp; // needs to be updated somehow on each frame (helps with line/intersection tracking time based validation)
void setTimerMs(uint16_t *timer);
uint16_t getTimerMs(uint16_t timer);
void showError(uint8_t num, uint32_t color, const char *message);


#ifdef __cplusplus
}
#endif

#endif
