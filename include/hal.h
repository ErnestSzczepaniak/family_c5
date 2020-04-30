#ifndef _hal_cv_h
#define _hal_cv_h

#include "stdio.h"
#include "string.h"
#include "port.h"
#include "alt_generalpurpose_io.h"

bool h_uart_init(int number, int baudrate);
bool h_uart_transmitt(int number, unsigned char * buffer, int size);
unsigned char h_uart_receive(int number);

template<typename ...T>
void h_uart_printf(int number, const char * format, T ... ts)
{
    char buffer[256];
    snprintf(buffer, sizeof(buffer), format, ts ...);
    auto size = strlen(buffer);
    snprintf(&buffer[size], sizeof(buffer) - size, "%s", "\r\n");

    h_uart_transmitt(number, (unsigned char *)buffer, strlen(buffer));
}

void h_dma_init();
void h_dma_m2m(void * destination, void * source, int size);
void h_dma_z2m(void * destination, int size);

bool h_fpga_bridge_init(int bridge);
void h_fpga_bridge_write(int bridge, unsigned int offset, unsigned int value);
unsigned int h_fpga_bridge_read(int bridge, unsigned int offset);
bool h_fpga_configure(unsigned char * bitstream, int size);

void h_sd_init();
void h_sd_write(unsigned int address, void * source, int size);
void h_sd_read(unsigned int address, void * destination, int size);

// 0 A, 1 B, 2 C

ALT_GPIO_PORT_t _h_gpio_pin2port(int pin);

void h_gpio_init();
void h_gpio_configure(int pin, bool input = false, bool interrupt = false);

void h_gpio_set(int pin, bool value);
bool h_gpio_get(int pin);
void h_gpio_toogle(int pin);

bool h_gpio_is_irq_pending(int pin);
void h_gpio_clear_irq_pending(int pin);

bool h_qspi_init();
bool h_qspi_deinit();
bool h_qspi_erase(unsigned int address);
bool h_qspi_read(int address, int size, unsigned char * buffer);
bool h_qspi_write(int address, int size, unsigned char * buffer);

#endif