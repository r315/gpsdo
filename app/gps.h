#ifndef _gps_h_
#define _gps_h_

#include <stdint.h>
#include "stdinout.h"

#define NMEA_SENTENCE_MAX_SIZE      (80 + 2)

/**
GGA Global Positioning System Fix Data. Time, Position and fix related data for a GPS receiver
GLL Geographic Position – Latitude/Longitude
GSA GPS DOP and active satellites
VTG Track Made Good and Ground Speed
RMC Recommended Minimum Navigation Information
 */
enum nmea_ident{
    NMEA_UNKNOWN,
    NMEA_INVALID,
    NMEA_GGA,
    NMEA_GLL,
    NMEA_GSA,
    NMEA_VTG,
    NMEA_RMC
};

typedef struct {
    char time[11];        // UTC Time (hhmmss.ss)
    char status;          // 'A' = Active/Valid, 'V' = Void/Invalid
    double latitude;      // Signed Decimal Degrees (+ for N, - for S)
    double longitude;     // Signed Decimal Degrees (+ for E, - for W)
    float speed_knots;    // Speed over ground in knots
    float course_degrees; // Course over ground in degrees
    char date[7];         // Date (ddmmyy)
} nmea_gprmc_t;

typedef struct {
    char time[11];        // UTC Time (hhmmss.ss)
    double latitude;      // Signed Decimal Degrees (+ for N, - for S)
    double longitude;     // Signed Decimal Degrees (+ for E, - for W)
    char fix;             // '0' = fix not available, '1' = GPS fix, '2' = dif GSP fix
    char nsat;            // Number of visible satellites, 00 - 12
} nmea_gpgga_t;

uint32_t gpsInit(stdinout_t *uart);
void gpsProcess(void);

void gpsFilter(const char *filter);

const char *nmeaSentenceGet(void);
enum nmea_ident nmeaSentenceParse(const char* sentence, void* parsed);
#endif