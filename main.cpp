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


void write_enable(int slave)
{
    unsigned char tx = 0x06;

    h_spi_write(&tx, 1, 0);
}

void write(int address, unsigned char * buffer, int size, int slave)
{
    unsigned char temp[1024];

    temp[0] = 0x02;
    temp[1] = (address >> 16) & 0xff;
    temp[2] = (address >> 8) & 0xff;
    temp[3] = (address >> 0) & 0xff;
    memcpy(&temp[4], buffer, size);

    h_spi_write(temp, size + 4, slave);
}

void read(int address, unsigned char * buffer, int size, int slave)
{
    unsigned char temp[1024];

    temp[0] = 0x03;
    temp[1] = (address >> 16) & 0xff;
    temp[2] = (address >> 8) & 0xff;
    temp[3] = (address >> 0) & 0xff;

    h_spi_write_read(temp, 4, buffer, size, slave);
}

void delay()
{
    for (int i = 0;i < 10000; i++);
}

int main()
{
    h_spi_init(1e3);

    write_enable(1);
    unsigned char tx[16];
    unsigned char rx[16];
    

    for (int i = 0; i < 16; i++)
    {
        tx[i] = i;
    }
    
    while(1)
    {

        write(0x10, tx, 5, 1);

        read(0x12, rx, 5, 1);

        delay();
    }
}

#endif
