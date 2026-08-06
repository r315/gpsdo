#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "gps.h"

static uint8_t log_nmea;
static uint8_t gps_uart_buf[64];
static uint16_t gps_uart_wr_idx;
static stdinout_t *gps_uart;


static uint8_t gps_line_get(void)
{
    uint8_t len;
    char c;

    if(gps_uart->available()){
        gps_uart->read(&c, 1);
        gps_uart_buf[gps_uart_wr_idx++] = c;
        if(c == '\n'){
            // no need for double buffer
            // since buffer is copied to DMA buffer
            len = gps_uart_wr_idx;
            gps_uart_wr_idx = 0;
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

    gps_uart = uart;
    gps_uart_wr_idx = 0;
    log_nmea = 0;

    return 1;
}

void gpsProcess(void)
{
    if(log_nmea){
        uint8_t len;
        if((len = gps_line_get()) > 0){
            gps_uart->write((const char*)gps_uart_buf, len);
        }
    }
}
