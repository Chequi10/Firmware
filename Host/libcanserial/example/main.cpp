#include <iostream>
#include <boost/format.hpp>

#include "stm32canbusif.h"


void on_can_message( const stm32canbus_serialif::can_message_event& ev )
{
    std::cout << boost::format("Device %d CANID: %d Len: %d Data: ") % ev.device_id % ev.canid % ev.len;
        for(size_t i=0;i<7;i++)
    {
        std::cout << boost::format(" %c") % ev.data[i];
       //std::cout << boost::format(" Longitud%d") % ev.len;
    }
    std::cout << std::endl;
}



int main(int argc, const char* argv[])
{
    if(argc < 2)
    {
        std::cerr << "No port specified" << std::endl;
        return -1;
    }
    std::cout << "Port: " << argv[1] << std::endl;
    stm32canbus_serialif can(argv[1],115200,on_can_message);
    
    char entrada{};
    can.start();
 while(1)
 {

 std::cin>> entrada;
 std::cin.ignore();
 
 if(entrada < '5' )
 {
    can.write_some(); 
    entrada=0;     
 }
 
    // Wait for key
   
}
    can.stop();
      
    return 0;
}