#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "cli_simple.h"
#include "board.h"
#include "i2c.h"
#include "ds1086.h"
#include "wdt.h"


/* static void dump_buf(uint8_t *buf, uint32_t count)
{
    for(int i = 0; i < count; i ++){
        if( (i & 15) == 0)
            printf("\n%02X: ", i & 0xF0);
        printf("%02X ", buf[i]);
    }
    putchar('\n');
} */

static uint32_t last_time_stamp;
static uint32_t time_diff;
static uint8_t nmea_output;
static uint8_t gps_uart_buf[64];
static uint16_t gps_uart_wr_idx;

static int clockCmd(int argc, char **argv)
{
    return CLI_OK;
}

static int i2cCmd(int argc, char **argv)
{
    int32_t val;
    uint8_t i2c_buf[256], count;
    i2cbus_t *i2c = board_i2c_get();


    if(!strcmp("help", argv[1]) || argc == 1){
        printf("Usage: i2c <operation>\n");
        printf("\tinit <bus>,       Initialize i2c bus (0-3)\n");
        printf("\tscan,             scan i2c bus\n");
        printf("\tslave <addr>,     set slave address (0-7F) for read/write registers\n");
        printf("\trd [count], Read current location, count (0-255)\n");
        printf("\trr <reg> [count], Read register, reg (0-ff), count (0-255)\n");
        printf("\twr <reg> <data>,  Write register, reg (0-ff), data (multiple)\n");
        return CLI_OK;
    }

    if( !strcmp("init", argv[1])){
        if(CLI_Ia2i(argv[2], &val) == 0){
            return CLI_BAD_PARAM;
        }

        if(val > 3){
            printf("Invalid bus %ld\n", val);
		    return CLI_BAD_PARAM;
	    }

        i2c->bus_num = val;
        I2C_Init(i2c);

        return CLI_OK;
    }

    if(!strcmp("slave", argv[1])){
        if(CLI_Ha2i(argv[2], (uint32_t*)&val)){
            i2c->addr = val;
            return CLI_OK;
        }
    }

    uint8_t read = 0;

    if(!strcmp("rd", argv[1])){
        read = 1;
    }

    if( !strcmp("rr", argv[1])){
        read = 2;
    }

    if(read){
        if (read == 2){
            if(CLI_Ha2i(argv[2], (uint32_t*)&val)){
                I2C_Write(i2c, i2c->addr, (uint8_t*)&val, 1);
                val = 3; // next parameter
            }else{
                return CLI_BAD_PARAM;
            }
        }else{
            val = 2; // next parameter
        }

        count = CLI_Ia2i(argv[val], &val) ? val : 1;

        if(I2C_Read(i2c, i2c->addr, i2c_buf, count) > 0){
            for(uint8_t i = 0; i < count; i ++){
                if( (i & 15) == 0) {
                    if(i == (count - 1)){
                        putchar('\n');
                    }else{
                        printf("\n%02X: ", i & 0xF0);
                    }
                }
                printf("%02X ", i2c_buf[i]);
            }
            return CLI_OK_LF;
        }else{
            printf("I2C Error\n");
        }
        return CLI_OK;
    }

    if(!strcmp("wr", argv[1])){
        if(CLI_Ha2i(argv[2], (uint32_t*)&val)){
            i2c_buf[0] = val;
            if(CLI_Ha2i(argv[3], (uint32_t*)&val)){
                i2c_buf[1] = val;
                I2C_Write(i2c, i2c->addr, i2c_buf, 2);
            }
        }
        return CLI_OK;
    }

    if( !strcmp("scan", argv[1])){
        printf("\n   ");

        for(int i = 0; i < 16; i++){
            printf("%02X ", i);
        }

        for(int i = 0; i < 128; i++){
            if( (i & 15) == 0)
                printf("\n%02X ", i & 0xF0);

            if(I2C_Read(i2c, i, &count, 1) == 0){
                printf("-- ");
            }else{
                printf("%02X ", i);
            }

            delay_ms(1);
        }
        putchar('\n');

        return CLI_OK;
    }

    return CLI_BAD_PARAM;
}
/*
static int trimCmd(int argc, char **argv)
{
    int32_t val;

    if(!CLI_Ia2i(argv[1], &val)){
        val = 0;
    }

    printf("Trim: %lx\n", board_trim_irc(val));

    return CLI_OK;
}
*/

static int resetCmd(int argc, char **argv)
{
    SW_Reset();
    return CLI_OK;
}

static int clearCmd(int argc, char **argv)
{
    CLI_Clear();
    return CLI_OK;
}

static int ds1086Cmd(int argc, char **argv)
{
    int32_t val;

/*     if(!xstrcmp("help", argv[1]) || argc < 2){
        return CLI_OK;
    } */

    if( !strcmp("init", argv[1])){
        val = DS1086_Init(&i2c);
        if(val < 0){
            printf("Failed to initialize DS1086\n");
        }
        printf("OFFSET Val: 0x%x (%d)\n", (uint16_t)val, (int)val);
        return CLI_OK;
    }

    if( !strcmp("freq", argv[1])){
        if(CLI_Ia2i(argv[2], &val)){
            if(DS1086_FrequencySet(val)){
                return CLI_OK;
            }
        }
    }

    if( !strcmp("regs", argv[1])){
        if(DS1086_PrescallerRead((uint16_t*)&val))
            printf("[02] PRES:\t0x%x\n", (uint16_t)val);
        if(DS1086_DacRead((uint16_t*)&val))
            printf("[08] DAC:\t0x%x (%d)\n", (uint16_t)val, (uint16_t)val);
        if(DS1086_OffsetRead((uint8_t*)&val))
            printf("[0E] OFFSET:\t0x%x\n", (uint8_t)val);
        if(DS1086_AddrRead((uint8_t*)&val))
            printf("[0D] ADDR:\t0x%x\n", (uint8_t)val);
        if(DS1086_RangeRead((uint8_t*)&val))
            printf("[37] RANGE:\t0x%x\n", (uint8_t)val);
        return CLI_OK;
    }

    if( !strcmp("rr", argv[1])){
        if(CLI_Ha2i(argv[2], (uint32_t*)&val)){
            if(DS1086_ReadReg((uint8_t)val, (uint16_t*)&val) == 0){
                return CLI_ERROR;
            }
            printf(" %x\n", (uint16_t)val);
            return CLI_OK;
        }
    }

    if( !strcmp("wr", argv[1])){
        uint8_t addr;
        if(CLI_Ha2i(argv[2], (uint32_t*)&val)){
            addr = (uint8_t)val;
            if(CLI_Ha2i(argv[3], (uint32_t*)&val)){
                DS1086_WriteReg(addr, val);
                return CLI_OK;
            }
        }
    }

    return CLI_BAD_PARAM;
}



static void time_stamp_cb(uint32_t val)
{
    time_diff = val - last_time_stamp;
    printf("TS: 0x%08lx diff: %ld (0x%08lx)\n", val, time_diff, time_diff);
    last_time_stamp = val;
}

static int tsCmd(int argc, char **argv)
{
    if(!strcmp("start", argv[1])){
        board_frequency_measurement_cb(time_stamp_cb);
        board_frequency_measurement_start();
        return CLI_OK;
    }

    if(!strcmp("stop", argv[1])){
        board_frequency_measurement_stop();
        return CLI_OK;
    }

static int dacCmd(int argc, char **argv)
{
    uint32_t val;

    if(!strcmp("init", argv[1])){
        DAC_Init();
    }

    if(!strcmp("duty", argv[1])){
        if(CLI_Ha2i(argv[2], &val)){
            DAC_DutySet(val);
        }
    }
    return CLI_OK;
}

static int nmeaCmd(int argc, char **argv)
{
    nmea_output = argv[1][0] == '1' ? 1 : 0;
    return CLI_OK;
}

cli_command_t cli_cmds [] = {
    {"help", ((int (*)(int, char**))CLI_Commands)},
    {"reset", resetCmd},
    {"clear", clearCmd},
    {"clock", clockCmd},
    //{"trim", trimCmd},
    {"i2c", i2cCmd},
    {"ds1086", ds1086Cmd},
    //{"gps", i2cCmd}, passtrough
    {"ts", tsCmd},
    {"nmea", nmeaCmd},
    {"dac", dacCmd},
};

static uint8_t gps_line_get(void)
{
    uint8_t c, len;

    if(serial_b_available()){
        serial_b_read(&c, 1);
        gps_uart_buf[gps_uart_wr_idx++] = c;
        if(c == '\n'){
            // no need for double buffer
            // since buffer is copy to DMA buffer
            len = gps_uart_wr_idx;
            gps_uart_wr_idx = 0;
            return len;
        }
    }

    return 0;
}

static void process_gps_output(void)
{
    uint8_t len;
    if((len = gps_line_get()) > 0){
        serial_a_write(gps_uart_buf, len);
    }
}

void gpsdo(void)
{
    CLI_Init("gpsdo >");
    CLI_RegisterCommand(cli_cmds, sizeof(cli_cmds) / sizeof(cli_command_t));
    CLI_Clear();

    gps_uart_wr_idx = 0;
    nmea_output = 0;

    WDT_Init(3000);

    while(1){
        if(CLI_ReadLine()){
            CLI_HandleLine();
        }

        if(nmea_output){
            process_gps_output();
        }

        WDT_Reset();
    }
}