#include "port.h"

Port & Port::address(unsigned int value)
{
    _map = (Map_port *) value;
    _map->edge = 0xffffffff;
}

Port & Port::init(int index, Port_direction direction, bool interrupt)
{
    _set(_map->direction, index, (bool) direction);
    _set(_map->interrupt, index, interrupt);

    return *this;
}

bool Port::get(int index)
{
    return _get(_map->data, index);
}

Port & Port::set(int index, bool value)
{
    value ? _set(_map->out_set, index, true) : _set(_map->out_clear, index, true);

    return *this;
}

Port & Port::toogle(int index)
{
    auto value = _get(_map->data, index);

    value ? _set(_map->out_clear, index, true) : _set(_map->out_set, index, true);

    return *this;
}

bool Port::is_interrupt_pending(int index)
{
    return _get(_map->edge, index);
}

Port & Port::clear_interrupt_pending(int index)
{
    _set(_map->edge, index, true);

    return *this;
}

Port & Port::data(unsigned int value)
{
    _map->data = value;

    return *this;
}

unsigned int Port::data()
{
    return _map->data;
}

Port & Port::direction(unsigned int value)
{
    _map->direction = value;

    return *this;
}

unsigned int Port::direction()
{
    return _map->direction;
}

Port & Port::edge(unsigned int value)
{
    _map->edge = value;

    return *this;
}

unsigned int Port::edge()
{
    return _map->edge;
}

Port & Port::interrupt(unsigned int value)
{
    _map->interrupt = value;

    return *this;
}

unsigned int Port::interrupt()
{
    return _map->interrupt;
}


//---------------------------------------------| info |---------------------------------------------//

void Port::_set(unsigned int & reg, int index, bool value)
{    
    reg &= ~(1 << index);
    reg |= (value << index);
}

bool Port::_get(unsigned int & reg, int index)
{
    return ((reg >> index ) & 0x1);
} 

