#include <iostream>
#include <boost/format.hpp>

#include "stm32canbusif.h"

void on_can_message(const stm32canbus_serialif::can_message_event &ev)
{
    std::cout << boost::format("Device %d CANID: %d Len: %d Data: ") % ev.device_id % ev.canid % ev.len;
    for (size_t i = 0; i < 7; i++)
    {
        std::cout << boost::format(" %d") % ev.data[i];
        // std::cout << boost::format(" Longitud%d") % ev.len;
    }
    std::cout << std::endl;
}

int main(int argc, const char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "No port specified" << std::endl;
        return -1;
    }
    std::cout << "Port: " << argv[1] << std::endl;
    stm32canbus_serialif can(argv[1], 115200, on_can_message);
  
    std::cout << "Ingrese menor a 3 para ver mensajes can y mayor a 3 para encender led rojo" << std::endl;
    char entrada{};
    can.start();
    while (1)
    {

        std::cin >> entrada;
        std::cin.ignore();

        if (entrada < '3')
        {
            can.write_some();
            can.opcodi='3';
            entrada = 0;
        }
        if(entrada > '3')
        {   
            can.write_some();
            std::cout << "Ingrese menor 5 para ver mensajes can" << std::endl;
            can.opcodi='0';
        }

        // Wait for key
    }
    can.stop();

    return 0;
}