
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "board.h"
#include "gps.h"
#include "logger.h"
#include "cli_simple.h"
#include "clock_gen.h"
#include "wdt.h"
#include "clock.h"

#ifndef VERSION
#define VERSION "vx.x.x"
#endif

#define SYS_FLAG_LOG_TEMP     (1 << 0)
#define SYS_FLAG_LOG_VREFINT  (1 << 1)
#define SYS_FLAG_LOG_COUNT    (1 << 2)
#define SYS_FLAG_LOG_GPS      (1 << 3)
#define SYS_FLAG_PPS          (1 << 4)

typedef struct {
    uint32_t log_temp  : 1;
    uint32_t log_vref  : 1;
    uint32_t log_count : 1;
    uint32_t log_pps   : 1;
    uint32_t log_vocxo : 1;
    uint32_t tunnel_gps  : 1;
    uint32_t count_ready : 1;
    uint32_t pps_ready   : 1;
}app_flags_t;

static app_flags_t sys;
static uint32_t next_tick;
static uint32_t sys_tick_ms;

#ifdef ENABLE_I2C
static uint8_t i2c_device_addr;
#endif

#ifdef ENABLE_COUNT
static countres_t count;

static void countHandler(countres_t cnt)
{
    count = cnt;
    sys.count_ready = TRUE;
}
#endif

#ifdef ENABLE_PPS
static void ppsHandler(void)
{
    sys.pps_ready = TRUE;
}

static void ppsLedBlink(void)
{
    #ifdef ENABLE_I2C
    static uint8_t pps_led_color;
    pps_led_color = (pps_led_color < 6) ? pps_led_color + 1 : 1;
    ledSet(LED_RGB, pps_led_color);
    #endif
}
#endif

#ifdef ENABLE_CLI

#ifdef ENABLE_I2C
static int i2cCmd(int argc, char **argv)
{
    int32_t val;
    uint8_t count;
    uint8_t read = 0;
    uint8_t i2c_buf[64];

    if(!strcmp("help", argv[1]) || argc == 1){
        printf("Usage: i2c <operation>\n");
        printf("\tscan              scan i2c bus\n");
        printf("\tslave <addr>      set slave address (0-7F) for read/write registers\n");
        printf("\trd [count]        Read from current location, count (1-64)\n");
        printf("\trr <reg> [count]  Read register, reg (0-ff), count (1-64)\n");
        printf("\twr <reg> <data>   Write register, reg (0-ff), data (multiple)\n");
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
                i2cWrite(i2c_device_addr, (uint8_t*)&val, 1);
                val = 3; // next parameter
            }else{
                return CLI_BAD_PARAM;
            }
        }else{
            val = 2; // next parameter
        }

        count = CLI_Ia2i(argv[val], &val) ? val : 1;

        if(i2cRead(i2c_device_addr, i2c_buf, count) > 0){
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
                i2cWrite(i2c_device_addr, i2c_buf, 2);
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

            if(i2cRead(i, &count, 1) == 0){
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
        printf("%d\n", dacValueGet());
    }

    if(!strcmp("help", argv[1])){
        printf("Usage: dac [args]\n");
        printf("\tNo args for current duty value\n");
        printf("\tset    set duty cycle 0-4095\n");
        printf("\ttrim   Enter trim mode, +,-,r,q keys\n\n");
    }

    if(!strcmp("set", argv[1])){
        if(CLI_Ia2i(argv[2], &val)){
            dacValueSet(val);
        }
    }

    if(!strcmp("trim", argv[1])){
        val = dacValueGet();
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
            dacValueSet(val);
        }while(c != 'q');
    }
    return CLI_OK;
}
#endif

#ifdef ENABLE_GPS
static int gpsCmd(int argc, char **argv)
{
    if(CLI_IS_PARM_1("help") || argc == 1){
        printf("Usage: gps [args]\n");
        printf("  en <0|1>      Enable gps module\n");
        return CLI_OK;
    }

    if(CLI_IS_PARM_1("en")){
        gpsEnableSet(argv[2][0] == '1');
        return CLI_OK;
    }

    return CLI_BAD_PARAM;
}
#endif

#ifdef ENABLE_ADC
static int adcCmd(int argc, char **argv)
{
    (void)argc;(void)argv;
    adcConversionStart(TRUE);
    printf("TEMP EXT: %ldmV\n", adcVoltageGet(ADC_CHANNEL_TS_EXT));
    printf("DAC FB  : %ldmV\n", adcVoltageGet(ADC_CHANNEL_DAC_FB));
    printf("VREF    : %ldmV\n", adcVoltageGet(ADC_CHANNEL_INTRV));
    printf("OXCO    : %ldmV\n", adcVoltageGet(ADC_CHANNEL_OXCO_VCC) * 2);
    return CLI_OK;
}
#endif

#if 0
static int oscCmd(int argc, char **argv)
{
    uint32_t val;

    if(argc == 1){
        val = 0;
        i2cWrite(0x13, (const uint8_t*)&val, 1);
        i2cRead(0x13, (uint8_t*)&val, sizeof(val));
        printf("0x%lx (%lu)\n", val, val);
    }

    if(!strcmp("trim", argv[1])){
        val = 0;
        i2cWrite(0x13, (const uint8_t*)&val, 1);
        i2cRead(0x13, (uint8_t*)&val, sizeof(val));
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
            i2cWrite(0x13, data, sizeof(data));
        }while(c != 'q');
    }
    return CLI_OK;
}
#endif

#ifdef ENABLE_PPS
static int ppsCmd(int argc, char **argv)
{
    if(CLI_IS_PARM_1("help") || argc == 1){
        printf("Usage: pps [args]\n");
        printf("  sync          Sync pps with external pps\n");
        printf("  pulse <val>   pulse width in units of 100us\n");
        printf("  int <0|1>     attach interrupt to pps\n");
        printf("  src <0|1>\n");
        return CLI_OK;
    }

    if(CLI_IS_PARM_1("sync")){
        ppsSync();
        return CLI_OK;
    }

    if(CLI_IS_PARM_1("pulse")){
        int32_t unit;
        if(CLI_GET_INT_PARM(2, unit)){
            ppsPulseSet(unit);
        }
        return CLI_OK;
    }

    if(CLI_IS_PARM_1("int")){
        ppsInt((argv[2][0] == '1') ? ppsHandler : NULL);
        return CLI_OK;
    }

    if(CLI_IS_PARM_1("src")){
        ppsSelect(argv[2][0] == '1');
        return CLI_OK;
    }

    return CLI_BAD_PARAM;
}
#endif

#ifdef ENABLE_COUNT
static int countCmd(int argc, char **argv)
{
    if(CLI_IS_PARM_1("help") || argc == 1){
        printf("Usage: count [args]\n");
        printf("  start\n");
        printf("  stop\n");
        printf("  mode <0|1>\n");
        printf("  src <0|1>\n");
        return CLI_OK;
    }

    if(CLI_IS_PARM_1("start")){
        countEocSet(countHandler);
        countStart();
        return CLI_OK;
    }

    if(CLI_IS_PARM_1("stop")){
        countStop();
        return CLI_OK;
    }

    if(CLI_IS_PARM_1("mode")){
        countConfig(argv[2][0] == '1');
        return CLI_OK;
    }

    if(CLI_IS_PARM_1("src")){
        countSelect(argv[2][0] == '1');
        return CLI_OK;
    }

    return CLI_BAD_PARAM;
}
#endif

static int sysCmd(int argc, char **argv)
{
    if(CLI_IS_PARM_1("help") || argc == 1){
        printf("Usage: sys [args]\n");
        printf("  log <temp|vrefint|count> <0|1>     Log enable\n");
        printf("  mco <src> <div>              Main clock output 0-15 0-15\n");

        return CLI_OK;
    }

    if(CLI_IS_PARM_1("mco")){
        int32_t src, div;
    #ifdef BOARD_DK415
        if(CLI_GET_INT_PARM(2, src) && CLI_GET_INT_PARM(3, div)){
            mco(src, div);
        }
    #else
        if(CLI_GET_INT_PARM(2, src)){
            mco(src);
        }
    #endif
        return CLI_OK;
    }

    if(CLI_IS_PARM_1("log")){
        if(CLI_IS_PARM_2("temp")){
            sys.log_temp = !!(argv[3][0] == '1');
            return CLI_OK;
        }

        if(CLI_IS_PARM_2("vrefint")){
            sys.log_vref = !!(argv[3][0] == '1');
            return CLI_OK;
        }
    #ifdef ENABLE_GPS
        if(CLI_IS_PARM_2("gps")){
            sys.tunnel_gps = !!(argv[3][0] == '1');
            return CLI_OK;
        }
    #endif
    }

    return CLI_BAD_PARAM;
}

static int helpCmd(int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("\nBase commands:\n");
    printf("  reset   system reset\n");
    printf("  clear   clear window\n");
    printf("  sys     system control\n");

    printf("\nFeature commands:\n");
#ifdef ENABLE_I2C
    printf("  i2c     i2c subsystem\n");
#endif
#ifdef ENABLE_DAC
    printf("  dac     dac subsystem\n");
#endif
#ifdef ENABLE_ADC
    printf("  adc     adc subsystem\n");
#endif
#ifdef ENABLE_CLK
    printf("  clk     clock output\n");
#endif
#ifdef ENABLE_GPS
    printf("  gps     gps subsystem\n");
#endif
#ifdef ENABLE_PPS
    printf("  pps     pps subsystem\n");
#endif
#ifdef ENABLE_PPS
    printf("  count   Frequency counter\n");
#endif

    putchar('\n');

    return CLI_OK;
}

cli_command_t cli_cmds [] = {
    {"help", helpCmd},
    {"reset", resetCmd},
    {"clear", clearCmd},
    {"sys", sysCmd},
#ifdef ENABLE_I2C
    {"i2c", i2cCmd},
#endif
#ifdef ENABLE_GPS
    {"gps", gpsCmd},
#endif
#ifdef ENABLE_DAC
    {"dac", dacCmd},
#endif
#ifdef ENABLE_ADC
    {"adc", adcCmd},
#endif
#if ENABLE_CLOCK_GEN
    {"pll", pllCmd},
#endif
#ifdef ENABLE_COUNT
    {"count", countCmd},
#endif
#ifdef ENABLE_PPS
    {"pps", ppsCmd},
#endif
};
#endif /* ENABLE_CLI */

/**
 * @brief Obtain external temperature using adc and k type thermistor
 * @param
 * @return tmeprature in degre C
 */
static float tempRead(void)
{
    int voltage = adcConversionGet(ADC_CHANNEL_TS_EXT);
    int oxco_vcc = adcConversionGet(ADC_CHANNEL_OXCO_VCC) * 2;

    if (voltage == 0)
        return -273.15f;

    // Calculate thermistor resistance
    float r_ntc = (THM_R_FIXED * voltage) / (oxco_vcc - voltage);

    // Apply Beta formula to get temperature in Kelvin
    float temp_k = 1.0f / ( (1.0f / THM_T0) + (1.0f / THM_BETA) * log(r_ntc / THM_R0) );

    // Convert Kelvin to Celsius
    return temp_k - 273.15f;
}

/**
 * @brief
 * @param start
 * @param mid
 * @param end
 * @param len
 */
static void titleLine(char start, char mid, char end, uint8_t len)
{
    putchar(start);
    while(len--)
        putchar(mid);
    putchar(end);
}

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
    #endif

    #ifdef BOARD_DK415
    sysclock_t clocks;
    CLOCK_GetAll(&clocks);

    putchar('\n');
    // using encoding CP437
    titleLine('\xc9', '\xcd', '\xbb', 40);
    putchar('\n');
    printf("\xba gpsdo %s                           \xba\n", VERSION);
    printf("\xba core speed %dMHz                      \xba\n", (uint16_t)(clocks.cpu/1000000UL));
    printf("\xba pclk1 %dMHz                            \xba\n", (uint16_t)(clocks.clk1/1000000UL));
    printf("\xba pclk2 %dMHz                            \xba\n", (uint16_t)(clocks.clk2/1000000UL));
    printf("\xba adcclk %dMHz                           \xba\n", (uint16_t)(clocks.clk3/1000000UL));
    titleLine('\xc8', '\xcd', '\xbc', 40);
    putchar('\n');
    #endif

    #ifdef ENABLE_GPS
    gpsInit(&gps_serial);
    #endif

    #ifdef ENABLE_CLI
    CLI_HandleLine();
    #endif

    memset(&sys, 0, sizeof(sys));

    sys_tick_ms = 500;

    WDT_Init(3000);

    #ifdef ENABLE_PPS
    ppsInt(ppsHandler);
    #endif


    while(1) {
#ifdef ENABLE_PPS
        if(sys.pps_ready){
            ppsLedBlink();
            sys.pps_ready = FALSE;
        }
#endif

#ifdef ENABLE_GPS
        if(sys.tunnel_gps){
            const char *sentence = nmeaSentenceGet();
            if(sentence){
                printf("%s", sentence);
            }
        }
#endif

#ifdef ENABLE_COUNT
        if(sys.count_ready){
            int int_part = (int)count;
            int frac_part = (int)((count - int_part) * 1000);
            printf("\r%d.%03d ", int_part, frac_part);
            sys.count_ready = FALSE;
        }
#endif

        /* Print log data */
        if(millis() > next_tick){
#ifdef ENABLE_ADC
            if(sys.log_temp){
                float temp = tempRead();
                printf("\rTemp %d.%d ", (int)temp, (int)(temp*10)%10);
            }

            if(sys.log_vref){
                printf("Vrefint %ldmV", adcVoltageGet(ADC_CHANNEL_INTRV));
            }
            adcConversionStart(FALSE);
#endif
            next_tick = millis() + sys_tick_ms;
        }

#ifdef ENABLE_CLI
        if(CLI_ReadLine()){
            CLI_HandleLine();
        }
#endif
        WDT_Reset();
    }
}

