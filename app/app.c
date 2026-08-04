
#include <stdint.h>
#include <stdio.h>
#include "board.h"
#include "gpsdo.h"
#include "logger.h"
#include "cli_simple.h"
#include "clock_gen.h"
#include "wdt.h"

#define SYS_FLAG_LOG_TEMP     (1 << 0)
#define SYS_FLAG_LOG_VREFINT  (1 << 1)

static uint32_t sys_flags;
static uint32_t next_tick;
static uint32_t sys_tick_ms;
#ifdef ENABLE_I2C
static uint8_t i2c_device_addr;
#endif

#ifdef ENABLE_CLI

#ifdef ENABLE_TDC
static void tdc_cb(int32_t val)
{
    printf("%ld\n", val);
}
#endif

#ifdef ENABLE_I2C
static int i2cCmd(int argc, char **argv)
{
    int32_t val;
    uint8_t count;
    uint8_t read = 0;
    uint8_t i2c_buf[64];

    if(!strcmp("help", argv[1]) || argc == 1){
        printf("Usage: i2c <operation>\n");
        printf("\tinit,             Initialize i2c bus\n");
        printf("\tscan,             scan i2c bus\n");
        printf("\tslave <addr>,     set slave address (0-7F) for read/write registers\n");
        printf("\trd [count], Read from current location, count (1-64)\n");
        printf("\trr <reg> [count], Read register, reg (0-ff), count (1-64)\n");
        printf("\twr <reg> <data>,  Write register, reg (0-ff), data (multiple)\n");
        return CLI_OK;
    }

    if(!strcmp("slave", argv[1])){
        if(CLI_Ha2i(argv[2], (uint32_t*)&val)){
            i2c_device_addr = val;
            return CLI_OK;
        }
    }

    if(!strcmp("rd", argv[1])){
        read = 1;
    }

    if( !strcmp("rr", argv[1])){
        read = 2;
    }

    if(read){
        if (read == 2){
            if(CLI_Ha2i(argv[2], (uint32_t*)&val)){
                board_i2c_write(i2c_device_addr, (uint8_t*)&val, 1);
                val = 3; // next parameter
            }else{
                return CLI_BAD_PARAM;
            }
        }else{
            val = 2; // next parameter
        }

        count = CLI_Ia2i(argv[val], &val) ? val : 1;

        if(board_i2c_read(i2c_device_addr, i2c_buf, count) > 0){
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
                board_i2c_write(i2c_device_addr, i2c_buf, 2);
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

            if(board_i2c_read(i, &count, 1) == 0){
                printf("-- ");
            }else{
                printf("%02X ", i);
            }

            delayMs(1);
        }
        putchar('\n');

        return CLI_OK;
    }

    return CLI_BAD_PARAM;
}
#endif

static int resetCmd(int argc, char **argv)
{
    (void)argc;(void)argv;
    board_reset();
    return CLI_OK;
}

static int clearCmd(int argc, char **argv)
{
    (void)argc;(void)argv;
    CLI_Clear();
    return CLI_OK;
}

static int gpsdoCmd(int argc, char **argv)
{
    (void)argc;

    if(CLI_IS_PARM(1, "help")){
        printf("Usage: gpsdo [args]\n");
        printf("\tstart, \n");
        return CLI_OK;
    }

    if(CLI_IS_PARM(1, "start")){
        gpsdo_start();
        return CLI_OK;
    }

    if(CLI_IS_PARM(1, "stop")){
        gpsdo_stop();
        return CLI_OK;
    }
/*
if(CLI_IS_PARM(1, "div")){
    int32_t div;
    if(CLI_GET_INT_PARM(2, div)){
        div_start(div);
        return CLI_OK;
    }
}
*/

    //printf("counter: 0x%08lx\n", last_counter);
    //printf("count:   0x%08lx\n", counts);

    return CLI_OK;
}

#ifdef ENABLE_TDC
static int tdcCmd(int argc, char **argv)
{
    if(!strcmp("start", argv[1])){
        tdc_start(tdc_cb);
        return CLI_OK;
    }

    if(!strcmp("stop", argv[1])){
        tdc_stop();
        return CLI_OK;
    }

    return CLI_OK;
}
#endif

#if ENABLE_CLOCK_GEN
static int clockCmd(int argc, char **argv)
{
    uint64_t freq;
    int32_t value;
    i2cbus_t *i2c = board_i2c_get();

    if(CLI_IS_PARM(1, "init")){
        if(clock_gen_init() != CLOCK_OK){
            printf("Failed to initialize Si5351\n");
        }
    }

    if (CLI_IS_PARM(1, "start")){
        if(clock_gen_init() != CLOCK_OK){
            printf("Failed to initialize clock generator\n");
            return CLI_OK;
        }

        if(CLI_GET_INT_PARM(2, value)){
            if(parseFreq(&freq, argv[3])){
                if(clock_gen_set_frequency(value, freq) == CLOCK_OK){
                    return CLI_OK;
                }
            }
        }

        printf("Usage: start <clk> <freq>\n");
    }

    if(!strcmp("trim", argv[1])){
        if(CLI_Ia2i(argv[2], &value)){
            //freq = (10000000 - 1000) * SI5351_FREQ_MULT;
            //parseFreq(&freq, argv[3]);
            int32_t val2 = 0;
            freq = 0;
            //CLI_Ia2i(argv[3], (int32_t*)&val2);
            CLI_Ia2i(argv[3], (int32_t*)&freq);
            freq *= SI5351_FREQ_MULT;
            char c;
            do{
                printf("%lu    \r", (uint32_t)(freq + val2));
                c = getchar();
                switch(c){
                    case '+':  val2++; break;
                    case '-':  val2--; break;
                    case '1':  val2 -= 100; break;
                    case '2':  val2 += 100; break;
                    case '3':  val2 -= 1000; break;
                    case '4':  val2 += 1000; break;
                    //case 'r': freq = DAC_MAX_VAL >> 1; break;
                    default: break;
                }
                Si5351_FreqSet((enum si5351_clock)value, freq + val2);
            }while(c != 'q');
        }
    }

    if(!strcmp("reset", argv[1])){
        Si5351_Reset();
    }

     if(!strcmp("enable", argv[1])){
        if(CLI_Ha2i(argv[2], (uint32_t*)&value)){
            Si5351_ClockPwrSet((enum si5351_clock)value, 1);
            Si5351_OutputEnableSet((enum si5351_clock)value, 1);
        }
    }

    if(!strcmp("disable", argv[1])){
        if(CLI_Ha2i(argv[2], (uint32_t*)&value)){
            Si5351_ClockPwrSet((enum si5351_clock)value, 0);
            Si5351_OutputEnableSet((enum si5351_clock)value, 0);
        }
    }
/*
    if (!strcmp("drive", argv[1])){
        uint32_t val1, val2;
        if(CLI_Ha2i(argv[2], &val1)){
            if(CLI_Ha2i(argv[3], &val2)){
                val2 &= 3;
                Si5351_OutputDriveStrengthSet((enum si5351_clock)val1, (enum si5351_drive)val2);
            }
        }
    }

    if (!strcmp("clk-sel", argv[1])){
        int32_t val1, val2;
        if(CLI_Ia2i(argv[2], &val1)){
            if(CLI_Ia2i(argv[3], &val2)){
                Si5351_ClockSourceSet((enum si5351_clock)val1, (enum si5351_clock_source)val2);
            }
        }
    }

    if (!strcmp("clk-r-div", argv[1])){
        int32_t val1, val2;
        if(CLI_Ia2i(argv[2], &val1)){
            if(CLI_Ia2i(argv[3], &val2)){
                Si5351_ClockRdivSet((enum si5351_clock)val1, (enum si5351_clock_r_div)val2);
            }
        }
    }
*/
    return CLI_OK;
}
#endif

#ifdef ENABLE_DAC
static int dacCmd(int argc, char **argv)
{
    int32_t val;

    if(argc == 1){
        printf("%d\n", dac_value_get());
    }

    if(!strcmp("help", argv[1])){
        printf("Usage: dac [args]\n");
        printf("\tNo args for current duty value\n");
        printf("\tduty,   set duty cycle 0-4095\n");
        printf("\ttrim,   Enter trim mode, +,-,r,q keys\n");
    }

    if(!strcmp("duty", argv[1])){
        if(CLI_Ia2i(argv[2], &val)){
            dac_value_set(val);
        }
    }

    if(!strcmp("trim", argv[1])){
        val = dac_value_get();
        char c;
        do{
            printf("%lu    \r", val);
            c = getchar();
            switch(c){
                case '+': if(val < DAC_MAX_VAL) val++; break;
                case '-': if(val > 0) val--; break;
                case 'r': val = DAC_MAX_VAL >> 1; break;
                default: break;
            }
            dac_value_set(val);
        }while(c != 'q');
    }
    return CLI_OK;
}
#endif

static int nmeaCmd(int argc, char **argv)
{
    (void)argc;(void)argv;
    gpsdo_log_gps_output(argv[1][0] == '1' ? 1 : 0);
    return CLI_OK;
}
#ifdef ENABLE_ADC
static int adcCmd(int argc, char **argv)
{
    adc_acquire_start(TRUE);
    LOG_INF("TEMP EXT %d", adc_voltage_get(THM_ADC_CH));
    LOG_INF("TEMP INT %d", adc_voltage_get(TSENSE_ADC_CH));
    LOG_INF("VREF %d", adc_voltage_get(VREFINT_ADC_CH));
    return CLI_OK;
}
#endif
#if 0
static int oscCmd(int argc, char **argv)
{
    uint32_t val;

    if(argc == 1){
        val = 0;
        board_i2c_write(0x13, (const uint8_t*)&val, 1);
        board_i2c_read(0x13, (uint8_t*)&val, sizeof(val));
        printf("0x%lx (%lu)\n", val, val);
    }

    if(!strcmp("trim", argv[1])){
        val = 0;
        board_i2c_write(0x13, (const uint8_t*)&val, 1);
        board_i2c_read(0x13, (uint8_t*)&val, sizeof(val));
        char c;
        do{
            printf("%lx    \r", val);
            c = getchar();
            switch(c){
                case '+': val++; break;
                case '-': val--; break;
                case 'r': val =  0xD5555555; break;
                default: break;
            }
            uint8_t data[5];
            data[0] = 0;
            data[1] = (uint8_t)val;
            data[2] = (uint8_t)(val>>8);
            data[3] = (uint8_t)(val>>16);
            data[4] = (uint8_t)(val>>24);
            board_i2c_write(0x13, data, sizeof(data));
        }while(c != 'q');
    }
    return CLI_OK;
}
#endif
static int sysCmd(int argc, char **argv)
{
    (void)argc;
    if(CLI_IS_PARM(1, "help")){
        printf("Usage: sys [args]\n");
        printf("\tmco <src> main clock output (0-7)\n");
        printf("\tlog <temp|vrefint> <0|1>, Logging\n");
        return CLI_OK;
    }

    if(CLI_IS_PARM(1, "mco")){
        uint32_t src;
        if(CLI_GET_HINT_PARM(2, src)){
            mco(src);
            return CLI_OK;
        }
    }

    if(CLI_IS_PARM(1, "log")){
        if(CLI_IS_PARM(2, "temp")){
            if(CLI_IS_PARM(3, "1")){
                sys_flags |= SYS_FLAG_LOG_TEMP;
            }else{
                sys_flags &= ~SYS_FLAG_LOG_TEMP;
            }
            return CLI_OK;
        }

        if(CLI_IS_PARM(2, "vrefint")){
            if(CLI_IS_PARM(3, "1")){
                sys_flags |= SYS_FLAG_LOG_VREFINT;
            }else{
                sys_flags &= ~SYS_FLAG_LOG_VREFINT;
            }
            return CLI_OK;
        }
    }

    return CLI_BAD_PARAM;
}

cli_command_t cli_cmds [] = {
    //{"help", ((int (*)(int, char**))CLI_Commands)},
    {"reset", resetCmd},
    {"clear", clearCmd},
#ifdef ENABLE_I2C
    {"i2c", i2cCmd},
#endif
    {"gpsdo", gpsdoCmd},
    {"nmea", nmeaCmd},
#ifdef ENABLE_DAC
    {"dac", dacCmd},
#endif
#ifdef ENABLE_DAC
    {"adc", adcCmd},
#endif
#if ENABLE_CLOCK_GEN
    {"pll", pllCmd},
#endif
#ifdef ENABLE_TDC
    {"tdc", tdcCmd},
#endif
    //{"osc", oscCmd},
    {"sys", sysCmd},
};
#endif
/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
void app(void)
{
    #ifdef ENABLE_CLI
    CLI_Init("gpsdo >", &cli_serial);
    CLI_RegisterCommand(cli_cmds, sizeof(cli_cmds) / sizeof(cli_command_t));
    CLI_Clear();
    #endif

    LOG_INF("gpsdo %s", VERSION);
    LOG_INF("core speed %dMHz", (uint16_t)(SystemCoreClock/1000000UL));
    LOG_PRINT("Type 'help' or '<cmd> help', for more information");

    #ifdef ENABLE_PPS
    if(!pps_init()){
        DBG_WRN("Fail to start pps");
    }
    #endif

    WDT_Init(3000);

    #ifdef ENABLE_CLI
    CLI_HandleLine();
    #endif

    sys_flags = 0;
    sys_tick_ms = 500;

    //div_start(10000000);
    //tdc_start(tdc_cb);
    gps_init(&gps_serial);

    while(1) {
        gps_process();
        #ifdef ENABLE_CLI
        if(CLI_ReadLine()){
            CLI_HandleLine();
        }
        #endif

        if(millis() > next_tick){
            #ifdef ENABLE_TERMISTOR
            if(sys_flags & SYS_FLAG_LOG_TEMP){
                float temp = temp_get();
                LOG_INF("Temp %d.%d", (int)temp, (int)(temp*10)%10);
            }
            #endif

            #ifdef ENABLE_ADC
            if(sys_flags & SYS_FLAG_LOG_VREFINT){
                LOG_INF("Vrefint %umV", adc_voltage_get(VREFINT_ADC_CH));
            }
            adc_acquire_start(FALSE);
            #endif
            next_tick = millis() + sys_tick_ms;
        }

        WDT_Reset();
    }
}

