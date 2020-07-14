#ifdef build_platform_host

#define CATCH_CONFIG_MAIN
#include "build.h"
#include "test.h"

#else

#include "build.h"
#include "hal.h"
#include "alt_qspi.h"

// alt qspi stig cmd 


int main()
{

    unsigned char buffer[1024];
    unsigned int t = 0x11223344;

    // h_qspi_deinit();
    auto status = h_qspi_init();

    status = h_qspi_read(0x00940000, 1024, buffer);
    status = h_qspi_erase(0x00940000);
    status = h_qspi_read(0x00940000, 1024, buffer);
    status = h_qspi_write(0x00940000, 4, (unsigned char *) &t);
    status = h_qspi_read(0x00940000, 1024, buffer);
    status = h_qspi_read(0x00140000, 1024, buffer);
    //status = h_qspi_read(0x00940000, 1024, buffer);

    ALT_QSPI_DEV_INST_CONFIG_t config_read;
    ALT_QSPI_TIMING_CONFIG_t config_timing;
    ALT_QSPI_DEV_SIZE_CONFIG_t config_size;

    auto s = alt_qspi_device_read_config_get(&config_read);
    s = alt_qspi_timing_config_get(&config_timing);
    s = alt_qspi_device_size_config_get(&config_size);
    auto size = get_smallest_sector_size();

    while(1)
    { 
    }
}

#endif
