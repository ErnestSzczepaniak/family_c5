#ifdef build_platform_host

#define CATCH_CONFIG_MAIN
#include "build.h"
#include "test.h"

#else

#include "build.h"
#include "hal.h"
#include "alt_qspi.h"

// alt qspi stig cmd 


// MISO D29 zielony CH1
// SS0 D27 zolty CH4
// CLK D25 pomaranczowy CH3
// MOSI D23 czerwony CH2
// SS1 D21 brazowy


int main()
{
    auto value = h_sd_init();

    auto size = 512;

    unsigned char tx[size];
    unsigned char rx[size] = {0};

    for (int i = 0; i < 512; i++)
    {
        tx[i] = i;
    }

    auto status = h_sd_read(0, rx, size);
    status = h_sd_write(0, tx, size);
    status = h_sd_read(0, rx, size);


    while(1);
}

#endif
