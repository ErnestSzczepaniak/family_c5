#ifdef build_platform_host

#define CATCH_CONFIG_MAIN
#include "build.h"
#include "test.h"

#else

#include "build.h"
#include "hal.h"
#include "alt_16550_uart.h"

ALT_16550_HANDLE_t handle;
    static constexpr auto uart_mux_address =        0xffd08718;
    static constexpr auto uart_baudrate =           115200;

int main()
{
    h_uart_init(0, 921600);
    
    auto * address = (unsigned int *) uart_mux_address;
	*address = 1;

    // _output.address(pio_address_output);
    // _output.init(pio_pin_output, Port_direction::OUTPUT, false);
    // _output.set(pio_pin_output, true);

    h_uart_init(1, uart_baudrate); 

    char tx[] = "pizdeczka\r\n";
    unsigned char rx[16];
    

    while(1)
    {
        h_uart_transmitt(1, (unsigned char *)tx, 12);

        int k = 2;

        auto s = h_uart_receive(1, rx);

        for (int i = 0; i < 100000; i++)
        {
            
        }
        
        
    }
}

#endif
