#ifndef _port_h
#define _port_h

struct Map_port
{
    unsigned int data;
    unsigned int direction;
    unsigned int interrupt;
    unsigned int edge;
    unsigned int out_set;
    unsigned int out_clear;
};

enum class Port_direction
{
    INPUT = 0,
    OUTPUT
};

class Port
{
public:
    Port() {}
    ~Port() {}

    //---------------------------------------------| public API |---------------------------------------------//

    Port & address(unsigned int address);

    Port & init(int index, Port_direction direction, bool interrupt);

    bool get(int index);
    Port & set(int index, bool value);
    Port & toogle(int index);

    bool is_interrupt_pending(int index);
    Port & clear_interrupt_pending(int index);

    //---------------------------------------------| info |---------------------------------------------//

    Port & data(unsigned int value);
    unsigned int data();

    Port & direction(unsigned int value);
    unsigned int direction();

    Port & edge(unsigned int value);
    unsigned int edge();

    Port & interrupt(unsigned int value);
    unsigned int interrupt();

protected:
    void _set(unsigned int & reg, int index, bool value);
    bool _get(unsigned int & reg, int index);

private:
    Map_port * _map;

};

#endif