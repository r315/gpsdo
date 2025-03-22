
#include <stdint.h>
#include <stdio.h>
#include "board.h"
#include "gpsdo.h"
#include "logger.h"

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
    board_init();

    dac_init();

    if(!pps_init()){
        DBG_WRN("Fail to start pps");
    }

    gpsdo();

    while(1)
    {
    }
}

