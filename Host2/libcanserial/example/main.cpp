#include <iostream>
#include <boost/format.hpp>

#include "stm32canbusif.h"



int main(int argc, const char *argv[])
{
   // if (argc < 2)
   // {
     //   std::cerr << "No port specified" << std::endl;
       // return -1;
    //}
  //  std::cout << "Port: " << argv[1] << std::endl;
    stm32canbus_serialif can("/dev/ttyACM0", 115200);

    std::cout << "Probandoasdas:" << std::endl;
    char entrada{};
    can.start();
   
       for (size_t i = 0; i < 10; i++)
    {
      
               can.itera();
                sleep(1);
     //   std::cout << boost::format("%d ") % tx_buffer[i];
   
    } 
        can.stop(); 
        

        // Wait for key
    
    

    return 0;
}