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
static double raw;
typedef void (*fparser_t)(uint8_t, const char*, void*);


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
 * @brief $GPGGA,,,,,,0,00,99.99,,,,,,*48
 * @param field_index
 * @param raw
 * @param field
 * @param func
 */
static void nmeaGgaFields(uint8_t field_index, const char *field, void *id)
{
    nmea_gpgga_t *identifier = (nmea_gpgga_t*)id;

    // Parse individual fields by sequential index
    switch (field_index) {
        case 0:
        identifier->id = NMEA_GGA;
        break;
        case 1: // UTC Time
        strncpy(identifier->time, field, sizeof(identifier->time) - 1);
        break;
        case 2: // Raw Latitude
        da2d(field, &raw);
        break;
        case 3: // Latitude Direction
        identifier->latitude = nmea_to_decimal(raw, field[0]);
        break;
        case 4: // Raw Longitude
        da2d(field, &raw);
        break;
        case 5: // Longitude Direction
        identifier->longitude = nmea_to_decimal(raw, field[0]);
        break;
        case 6: // GPS Quality indicator
        identifier->fix = field[0] - '0';
        break;
        case 7: // Satellites in view
        {
            int32_t nsat;
            if(ia2i(field, (int32_t*)&nsat))
            identifier->nsat = (int32_t)nsat;
            break;
        }
        default:
        break;
    }
}

/**
 * @brief
 * $GPRMC,,V,,,,,,,,,,N*53
 * @param field_index
 * @param raw
 * @param field
 * @param func
 */
static void nmeaRmcFields(uint8_t field_index, const char *field, void *id)
{
    nmea_gprmc_t *identifier = (nmea_gprmc_t*)id;

    // Parse individual fields by sequential index
    switch (field_index) {
        case 0:
            identifier->id = NMEA_RMC;
            break;
        case 1: // UTC Time
            strncpy(identifier->time, field, sizeof(identifier->time) - 1);
            break;
        case 2: // Status
            identifier->status = field[0];
            break;
        case 3: // Raw Latitude
            da2d(field, &raw);
            break;
        case 4: // Latitude Direction
            identifier->latitude = nmea_to_decimal(raw, field[0]);
            break;
        case 5: // Raw Longitude
            da2d(field, &raw);
            break;
        case 6: // Longitude Direction
            identifier->longitude = nmea_to_decimal(raw, field[0]);
            break;
        case 7: // Speed
            identifier->speed_knots = da2d(field, &raw);
            break;
        case 8: // Course
            identifier->course_degrees = da2d(field, &raw);
            break;
        case 9: // Date
            strncpy(identifier->date, field, sizeof(identifier->date) - 1);
            break;
        default:
            break;
    }
}

/**
 * @brief
 *
 * @param sentence
 * @param data
 * @return
 */
static enum nmea_ident nmeaParse(const char *sentence, void *identifier, fparser_t parser)
{
    const char *p = sentence;
    uint8_t field = 0;
    char buffer[32];
    uint8_t checksum = 0;

    if(!p || !identifier || !parser){
        return NMEA_INVALID;
    }

    while (*p) {
        // Find the end of the current field, compute crc and len
        const char *next_comma = p;
        uint8_t len = 0;
        while (*next_comma != ',') {
            char tmp = *next_comma;
            if(tmp == '\0'){
                // unexpected end
                return NMEA_INVALID;
            }
            if(tmp == '*'){
                // End of fields, check checksum
                return *(enum nmea_ident*)identifier;
            }
            // Copy field data into an isolated null-terminated buffer
            if(len < sizeof(buffer)){
                buffer[len++] = tmp;
            }
            checksum ^= tmp;
            next_comma++;
        }

        buffer[len] = '\0';

        // Call identifier parser only if field is not empty
        if(len > 0){
            parser(field, buffer, identifier);
        }

        p = next_comma + 1;
        field++;
    }

    return NMEA_INVALID;
}

/**
 * @brief Tries to parse NMEA sentence to a structure
 *
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

    // skip sentence start
    identifier = sentence + 1;

    if((data_field = strstr(identifier, "GPRMC")) != NULL){
        return nmeaParse(data_field, parsed, nmeaRmcFields);
    }else if((data_field = strstr(identifier, "GPVTG")) != NULL){
        return nmeaParse(data_field, parsed, NULL);
    }else if((data_field = strstr(identifier, "GPGGA")) != NULL){
        return nmeaParse(data_field, parsed, nmeaGgaFields);
    }else if((data_field = strstr(identifier, "GPGSA")) != NULL){
        return nmeaParse(data_field, parsed, NULL);
    }else if((data_field = strstr(identifier, "GPGLL")) != NULL){
        return nmeaParse(data_field, parsed, NULL);
    }

    return NMEA_UNKNOWN;
}

/*

*/