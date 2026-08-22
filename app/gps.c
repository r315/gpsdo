#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "strfunc.h"
#include "gps.h"

// protocols UBX, NMEA 2.3

static uint8_t nmea_sentence[NMEA_SENTENCE_MAX_SIZE];
static uint16_t serial_wr_idx;
static stdinout_t *serial;


static uint8_t gps_line_get(void)
{
    uint8_t len;
    char c;

    if(serial->available()){
        serial->read(&c, 1);
        nmea_sentence[serial_wr_idx++] = c;
        if(c == '\n'){
            // no need for double buffer
            // since buffer is copied to DMA buffer
            len = serial_wr_idx;
            serial_wr_idx = 0;
            return len;
        }
    }

    return 0;
}

uint32_t gpsInit(stdinout_t *uart)
{
    if(!uart){
        return 0;
    }

    serial = uart;
    serial_wr_idx = 0;
    return 1;
}

void gpsProcess(void)
{
    uint8_t len;
    if((len = gps_line_get()) > 0){
        serial->write((const char*)nmea_sentence, len);
    }
}

/**
 * @brief
 * @param
 * @return
 */
const char *nmeaSentenceGet(void)
{
    uint8_t len;

    if((len = gps_line_get()) > 0){
        nmea_sentence[len] = '\0';
        return (const char*)nmea_sentence;
    }

    return NULL;
}


/*
static uint8_t nmeaChecksum(const char *sentence)
{
    uint8_t checksum = 0;
    // Skip the '$' sign
    int i = 1;

    // XOR characters until hitting the '*' or end of string
    while (sentence[i] != '\0' && sentence[i] != '*') {
        checksum ^= sentence[i];
        i++;
    }

    return checksum;
}
*/

static double nmea_to_decimal(double raw_coord, char direction) {
    int degrees = (int)(raw_coord / 100);
    double minutes = raw_coord - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

/**
 * @brief
 * @param sentence
 * @param data
 * @return
 */
static enum nmea_ident nmeaGpggaParse(const char *sentence, nmea_gpgga_t *data)
{
    UNUSED(sentence);
    UNUSED(data);
    return NMEA_UNKNOWN;
}

static enum nmea_ident nmeaGpgllParse(const char *sentence, nmea_gpgga_t *data)
{
    UNUSED(sentence);
    UNUSED(data);
    return NMEA_UNKNOWN;
}

static enum nmea_ident nmeaGpgsaParse(const char *sentence, nmea_gpgga_t *data)
{
    UNUSED(sentence);
    UNUSED(data);
    return NMEA_UNKNOWN;
}

static enum nmea_ident nmeaGpvtgParse(const char *sentence, nmea_gpgga_t *data)
{
    UNUSED(sentence);
    UNUSED(data);
    return NMEA_UNKNOWN;
}


/**
 * @brief Parse RMC sentence
 *
 * @param sentence  sentence without identifier
 * @param data
 *
 * @return
 */
static enum nmea_ident nmeaGprmcParse(const char *sentence, nmea_gprmc_t *data)
{
    const char *p = sentence;
    int field = 0;
    char buffer[32];

    // Variables for internal coordinate parsing
    double raw_dir;

    while (*p) {
        // Find the end of the current field
        const char *next_comma = strchr(p, ',');
        if (!next_comma) {
            next_comma = strchr(p, '*'); // Checksum separator at the end
        }

        // Calculate field length
        size_t len = next_comma ? (size_t)(next_comma - p) : strlen(p);
        if (len >= sizeof(buffer)) len = sizeof(buffer) - 1;

        // Copy field data into an isolated null-terminated buffer
        strncpy(buffer, p, len);
        buffer[len] = '\0';


        // Parse individual fields by sequential index
        switch (field) {
            case 1: // UTC Time
                strncpy(data->time, buffer, sizeof(data->time) - 1);
                break;
            case 2: // Status
                data->status = buffer[0];
                break;
            case 3: // Raw Latitude
                if (len > 0) da2d(buffer, &raw_dir); else raw_dir = 0.0f;
                break;
            case 4: // Latitude Direction
                if (raw_dir != 0.0) data->latitude = nmea_to_decimal(raw_dir, buffer[0]);
                break;
            case 5: // Raw Longitude
                if (len > 0) da2d(buffer, &raw_dir); else raw_dir = 0.0f;
                break;
            case 6: // Longitude Direction
                if (raw_dir != 0.0) data->longitude = nmea_to_decimal(raw_dir, buffer[0]);
                break;
            case 7: // Speed
                data->speed_knots = (len > 0) ? da2d(buffer, &raw_dir) : 0.0f;
                break;
            case 8: // Course
                data->course_degrees = (len > 0) ? da2d(buffer, &raw_dir) : 0.0f;
                break;
            case 9: // Date
                strncpy(data->date, buffer, sizeof(data->date) - 1);
                break;
            default:
                break;
        }

        if (!next_comma || *next_comma == '*') break;
        p = next_comma + 1;
        field++;
    }

    return (data->status == 'A') ? NMEA_RMC : NMEA_INVALID;
}

/**
 * @brief Tries to parse NMEA sentence to a structure
 *
 * $GPRMC,,V,,,,,,,,,,N*53
 * $GPVTG,,,,,,,,,N*30
 * $GPGGA,,,,,,0,00,99.99,,,,,,*48
 * $GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30
 * $GPGLL,,,,,,V,N*64
 *
 * @param sentence  well formed nmea sentence starting with '$'
 * @param parsed    pointer to unknown structure, it has to have a minimum size
 *                  to parse expected sentences
 *
 * @return  identifier of parsed sentence
 */
enum nmea_ident nmeaSentenceParse(const char* sentence, void* parsed)
{
    const char *identifier;
    const char *data_field;

    if(!sentence || sentence[0] != '$'){
        return NMEA_INVALID;
    }

    identifier = sentence + 1;

    if((data_field = strstr(identifier, "GPRMC")) != NULL){
        return nmeaGprmcParse(data_field, parsed);
    }else if((data_field = strstr(identifier, "GPVTG")) != NULL){
        return nmeaGpvtgParse(data_field, parsed);
    }else if((data_field = strstr(identifier, "GPGGA")) != NULL){
        return nmeaGpggaParse(data_field, parsed);
    }else if((data_field = strstr(identifier, "GPGSA")) != NULL){
        return nmeaGpgsaParse(data_field, parsed);
    }else if((data_field = strstr(identifier, "GPGLL")) != NULL){
        return nmeaGpgllParse(data_field, parsed);
    }

    return NMEA_UNKNOWN;
}

/*

*/