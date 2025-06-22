#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "gpsdo.h"
#include "board.h"
#include "logger.h"


static uint32_t last_counter;
static uint32_t counts;
static uint8_t nmea_output, print_adc;
static uint8_t gps_uart_buf[64];
static uint16_t gps_uart_wr_idx;
static uint32_t gpsdo_ticks;
static uint8_t i2c_buf[64];


static void counter_cb(uint32_t counter)
{
    static uint8_t color = 1;
    counts = counter - last_counter;
    last_counter = counter;

    color = (color == 7) ? 1 : color + 1;

    led_set(LED_PPS, color);

    LOG_PRINT("\2C%ld\3", counts);
}

static uint8_t gps_line_get(void)
{
    uint8_t c, len;

    if(serial_b_ops.available()){
        serial_b_ops.read(&c, 1);
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

void gpsdo_print_gps_output(uint8_t on)
{
    nmea_output = on;
}

void gpsdo_init(void)
{
    gpsdo_ticks = get_ms();
    gps_uart_wr_idx = 0;
    nmea_output = 0;
}

void gpsdo(void)
{
    if(elapsed_ms(gpsdo_ticks) > 500 && print_adc){
        float temp = temp_get();
        printf("\2T%d.%d\3", (int)temp, (int)(temp*10)%10);
        gpsdo_ticks = get_ms();
    }

    if(nmea_output){
        uint8_t len;
        if((len = gps_line_get()) > 0){
            serial_a_ops.write(gps_uart_buf, len);
        }
    }
}

void gpsdo_start(void)
{
    counter_start(counter_cb);
}

void gpsdo_stop(void)
{
    counter_stop();
}