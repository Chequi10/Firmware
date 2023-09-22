#include <iostream>
#include <boost/format.hpp>

#include "stm32canbusif.h"


void on_can_message( const stm32canbus_serialif::can_message_event& ev )
{
    std::cout << boost::format("Device %d CANID: %d Len: %d Data: ") % ev.device_id % ev.canid % ev.len;
        for(size_t i=0;i<=8;i++)
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
    
   
    can.start();
   

    // Wait for key
    std::cin.ignore(); 
  
    can.stop();
      
    return 0;
}