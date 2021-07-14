#ifdef build_platform_host

#define CATCH_CONFIG_MAIN
#include "build.h"
#include "test.h"

#else

#include "build.h"
#include "hal.h"
#include "alt_qspi.h"

void write(unsigned int address, int size, unsigned char * buffer)
{
    unsigned char temp[1024];

    unsigned char write_enable = 0x06;

    h_gpio_set(23, false);

    auto status = h_spi_write(&write_enable, 1, 0);

    h_gpio_set(23, true);

    temp[0] = 0x02;
    temp[1] = (address >> 16) & 0xff;
    temp[2] = (address >> 8) & 0xff;
    temp[3] = (address >> 0) & 0xff;
    memcpy(&temp[4], buffer, size);

    h_gpio_set(23, false);

    status = h_spi_write(temp, size + 4, 0);

    h_gpio_set(23, true);
}

void read(unsigned int address, int size, unsigned char * buffer)
{
    unsigned char temp[1024];

    temp[0] = 0x03;
    temp[1] = (address >> 16) & 0xff;
    temp[2] = (address >> 8) & 0xff;
    temp[3] = (address >> 0) & 0xff;

    h_gpio_set(23, false);
    auto status = h_spi_write_read(temp, 4, buffer, size, 0);
    h_gpio_set(23, true);
}

int main()
{
    unsigned char tx[128];
    unsigned char rx[128];

    auto status = h_qspi_init();

    status = h_qspi_read(0x0, 128, rx);



    // h_gpio_init();
    // h_gpio_configure(23);
    // h_gpio_set(23, true);
    // h_spi_init(1e6);

    // memset(tx, 0xff, 128);

    // read(0, 128, rx);
    
    // write(0, 128, tx);

    // read(0, 128, rx);

    while(1);
}

#endif
