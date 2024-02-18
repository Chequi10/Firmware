#include <iostream>
#include <boost/format.hpp>

#include "stm32canbusif.h"

void on_can_message(const stm32canbus_serialif::can_message_event &ev)
{
    std::cout << boost::format("Device %d CANID: 0x%08x Length: %d Data: ") % ev.device_id % ev.canid % ev.len;
    for (size_t i = 0; i < ev.len; i++)
    {

        std::cout << boost::format(" %d ") % uint32_t{ev.data[i]};
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

    std::cout << "Ingrese menor a 3 para ver mensajes CAN:\nIngrese mayor a 3 para encender led rojo:" << std::endl;
    char entrada{};
    can.start();
    while (1)
    {
        std::cin >> entrada;
        std::cin.ignore();
        std::cout << "Ingrese menor a 0 para ver mensajes CAN:\nIngrese mayor a 3 para encender led rojo:" << std::endl;

        switch (entrada)
        {
        case '0': // Event 0: envio mensaje can y enciende led azul.
        { 
            can.write_some();
            can.opcodi = '0';
        }
        break;

        case '3': // Event 1: enciende led rojo.
        {
            can.write_some();
            can.opcodi = '3';
        }
        break;

        default:
        { can.opcodi = '0';
        }
        }

        // Wait for key
    }
    can.stop();

    return 0;
}