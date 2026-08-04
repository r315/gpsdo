#ifndef _gpsdo_h_
#define _gpsdo_h_

#ifndef VERSION
#define VERSION "vx.x.x"
#endif

#include "stdinout.h"

void gps_process(void);
uint32_t gps_init(stdinout_t *uart);
void gpsdo_log_gps_output(uint8_t on);
void gpsdo_start(void);
void gpsdo_stop(void);
#endif