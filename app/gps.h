#ifndef _gps_h_
#define _gps_h_

#include <stdint.h>
#include "stdinout.h"

uint32_t gpsInit(stdinout_t *uart);
void gpsProcess(void);
#endif