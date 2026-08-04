#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "gpsdo.h"
#include "board.h"
#include "logger.h"

static uint32_t last_count;
static uint32_t cur_count;
static uint8_t log_nmea;
static uint8_t gps_uart_buf[64];
static uint16_t gps_uart_wr_idx;
static uint32_t gpsdo_ticks;
static stdinout_t *gps_uart;

static void counter_cb(uint32_t count)
{
    static uint8_t color = 1;
    cur_count = count - last_count;
    last_count = count;

    color = (color == 7) ? 1 : color + 1;

    ledSet(LED_PPS, color);

    LOG_PRINT("\2C%ld\3", cur_count);
}

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

void gpsdo_log_gps_output(uint8_t on)
{
    log_nmea = on;
}

uint32_t gps_init(stdinout_t *uart)
{
    if(!uart){
        return 0;
    }

    gps_uart = uart;

    gpsdo_ticks = millis();
    gps_uart_wr_idx = 0;
    log_nmea = 0;

    return 1;
}

void gps_process(void)
{
    if(log_nmea){
        uint8_t len;
        if((len = gps_line_get()) > 0){
            gps_uart->write((const char*)gps_uart_buf, len);
        }
    }
}

void gpsdo_start(void)
{
    countStart(counter_cb);
}

void gpsdo_stop(void)
{
    countStop();
}