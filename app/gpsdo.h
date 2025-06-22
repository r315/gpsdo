#ifndef _gpsdo_h_
#define _gpsdo_h_

#ifndef VERSION
#define VERSION "vx.x.x"
#endif

void gpsdo(void);
void gpsdo_init(void);
void gpsdo_log_gps_output(uint8_t on);
void gpsdo_start(void);
void gpsdo_stop(void);
#endif