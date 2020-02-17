#include "hal.h"
#include "alt_16550_uart.h"
#include "alt_dma.h"
#include "alt_bridge_manager.h"
#include "alt_address_space.h"
#include "alt_sdmmc.h"
#include "alt_fpga_manager.h"

//---------------------------------------------| uart |---------------------------------------------//

ALT_16550_HANDLE_t _h_handle_uart0, _h_handle_uart1;

unsigned int _h_uart_status_get(int number)
{
    long unsigned int status;

    if (number == 0)
    {
        alt_16550_line_status_get(&_h_handle_uart0, &status);
    }
    else
    {
        alt_16550_line_status_get(&_h_handle_uart1, &status);   
    }

    if (status & 0x01) h_uart_receive(number);

    return status;
}

bool h_uart_init(int number, int baudrate)
{
    if (number == 0)
    {
        alt_16550_init(ALT_16550_DEVICE_SOCFPGA_UART0, 0, 0, &_h_handle_uart0);
        alt_16550_line_config_set(&_h_handle_uart0, ALT_16550_DATABITS_8, ALT_16550_PARITY_DISABLE, ALT_16550_STOPBITS_1);
        alt_16550_baudrate_set(&_h_handle_uart0, baudrate);
        alt_16550_int_enable_rx(&_h_handle_uart0);
        alt_16550_enable(&_h_handle_uart0);
    }
    else
    {
        alt_16550_init(ALT_16550_DEVICE_SOCFPGA_UART1, 0, 0, &_h_handle_uart1);
        alt_16550_line_config_set(&_h_handle_uart1, ALT_16550_DATABITS_8, ALT_16550_PARITY_DISABLE, ALT_16550_STOPBITS_1);
        alt_16550_baudrate_set(&_h_handle_uart1, baudrate);
        alt_16550_int_enable_rx(&_h_handle_uart1);
        alt_16550_enable(&_h_handle_uart1);
    }

    return true;
}

bool h_uart_transmitt(int number, unsigned char * buffer, int size)
{
    for (int i = 0; i < size; i++)
    {
        while(_h_uart_status_get(number) != 0x60);

        number == 0 ? alt_16550_write(&_h_handle_uart0, *buffer++) : alt_16550_write(&_h_handle_uart1, *buffer++);
    }

    return true;
}

unsigned char h_uart_receive(int number)
{
    char result;

    number == 0 ? alt_16550_read(&_h_handle_uart0, &result) : alt_16550_read(&_h_handle_uart1, &result);

    return result;
}

//---------------------------------------------| dma |---------------------------------------------//

ALT_DMA_CFG_t _h_config_dma;
ALT_DMA_CHANNEL_t _h_channel_dma;

void h_dma_init()
{
	_h_config_dma.manager_sec = ALT_DMA_SECURITY_DEFAULT;
	
	for (int i = 0; i < 8; i++)
	{
		_h_config_dma.irq_sec[i] = ALT_DMA_SECURITY_DEFAULT;
	}

	for (int i = 0; i < 32; i++)
	{
		_h_config_dma.periph_sec[i] = ALT_DMA_SECURITY_DEFAULT;
	}

    alt_dma_init(&_h_config_dma);
    alt_dma_channel_alloc_any(&_h_channel_dma);
}

void h_dma_m2m(void * destination, void * source, int size)
{
    ALT_DMA_CHANNEL_STATE_t state;

    alt_dma_memory_to_memory(_h_channel_dma, nullptr, destination, source, size, false, ALT_DMA_EVENT_0);

    while(state != ALT_DMA_CHANNEL_STATE_STOPPED) alt_dma_channel_state_get(_h_channel_dma, &state);
}

void h_dma_z2m(void * destination, int size)
{
    ALT_DMA_CHANNEL_STATE_t state;

    alt_dma_zero_to_memory(_h_channel_dma, nullptr, destination, size, false, ALT_DMA_EVENT_0);

    while(state != ALT_DMA_CHANNEL_STATE_STOPPED) alt_dma_channel_state_get(_h_channel_dma, &state);
}

//---------------------------------------------| fpga |---------------------------------------------//

bool h_fpga_bridge_init(int bridge)
{
    alt_bridge_init((ALT_BRIDGE_t) bridge, nullptr, nullptr);
    alt_addr_space_remap(ALT_ADDR_SPACE_MPU_ZERO_AT_BOOTROM, ALT_ADDR_SPACE_NONMPU_ZERO_AT_SDRAM, ALT_ADDR_SPACE_H2F_ACCESSIBLE, ALT_ADDR_SPACE_LWH2F_ACCESSIBLE);

    return true;
}

void h_fpga_bridge_write(int bridge, unsigned int offset, unsigned int value)
{
    if (bridge == 1)
    {
        alt_write_word(0xc0000000 + offset, value);
    }
    else if (bridge == 2)
    {
        alt_write_word(0xff200000 + offset, value);
    }
}

unsigned int h_fpga_bridge_read(int bridge, unsigned int offset)
{
    if (bridge == 1)
    {
        return alt_read_word(0xc0000000 + offset);
    }
    else if (bridge == 2)
    {
        return alt_read_word(0xff200000 + offset);
    }

    return 0;
}

bool h_fpga_configure(unsigned char * bitstream, int size)
{
    alt_fpga_control_enable();
    alt_fpga_configure(bitstream, size);
    alt_fpga_control_disable();

    return true;
}

//---------------------------------------------| sd |---------------------------------------------//

ALT_SDMMC_CARD_INFO_t _h_info_sd;
ALT_SDMMC_CARD_MISC_t _h_misc_sd;

void h_sd_init()
{
    alt_sdmmc_init();
	alt_sdmmc_card_pwr_on();
	alt_sdmmc_card_identify(&_h_info_sd);
	alt_sdmmc_card_misc_get(&_h_misc_sd);
	alt_sdmmc_card_bus_width_set(&_h_info_sd, ALT_SDMMC_BUS_WIDTH_4);
	alt_sdmmc_dma_enable();
}

void h_sd_write(unsigned int address, void * source, int size)
{
    alt_sdmmc_write(&_h_info_sd,  (void*) address, source, size);
}

void h_sd_read(unsigned int address, void * destination, int size)
{
    alt_sdmmc_read(&_h_info_sd, destination, (void*) address, size);
}

//---------------------------------------------| info |---------------------------------------------//

ALT_GPIO_PORT_t _h_gpio_pin2port(int pin)
{
    if (pin < 29) return ALT_GPIO_PORTA;
    else if (pin < 58) return ALT_GPIO_PORTB;
    else if (pin < 70) return ALT_GPIO_PORTC;
    else return ALT_GPIO_PORT_UNKNOWN;
}

unsigned int _h_gpio_pin2mask(int pin)
{
    auto port = _h_gpio_pin2port(pin);

    if (port == ALT_GPIO_PORTB) pin -= 29;
    if (port == ALT_GPIO_PORTC) pin -= 58;
    
    return (1 << pin);
}

void h_gpio_init()
{
    alt_gpio_init();
}

void h_gpio_configure(int pin, bool input, bool interrupt)
{
    auto port = _h_gpio_pin2port(pin);
    auto mask = _h_gpio_pin2mask(pin);

    if (input == false)
    {
        alt_gpio_port_datadir_set(port, mask, mask); // 1 - output
    }
    else
    {
        alt_gpio_port_datadir_set(port, mask, 0); // 0 - input

        if (interrupt)
        {
            alt_gpio_port_int_type_set(port, mask, mask); // edge sensitive
            alt_gpio_port_int_pol_set(port, mask, 0); // active low - falling edge
            alt_gpio_port_int_enable(port, mask); // enable
            alt_gpio_port_debounce_set(port, mask, mask); // debounce
        }
    }   
}

void h_gpio_set(int pin, bool value)
{
    auto port = _h_gpio_pin2port(pin);
    auto mask = _h_gpio_pin2mask(pin);
    
    value ? alt_gpio_port_data_write(port, mask, mask): alt_gpio_port_data_write(port, mask, 0);
}

bool h_gpio_get(int pin)
{
    auto port = _h_gpio_pin2port(pin);
    auto mask = _h_gpio_pin2mask(pin);

    auto value = alt_gpio_port_data_read(port, mask);

    return ((value >> pin) & 0x01);
}

void h_gpio_toogle(int pin)
{
    auto value = h_gpio_get(pin);

    h_gpio_set(pin, !value);
}

bool h_gpio_is_irq_pending(int pin)
{
    auto port = _h_gpio_pin2port(pin);
    auto mask = _h_gpio_pin2mask(pin);

    auto value = alt_gpio_port_int_status_get(port);

    return value & mask;
}

void h_gpio_clear_irq_pending(int pin)
{
    auto port = _h_gpio_pin2port(pin);
    auto mask = _h_gpio_pin2mask(pin);

    alt_gpio_port_int_status_clear(port, mask);
}
