#include <boost/format.hpp>
#include "stm32canbusif.h"

stm32canbus_serialif::stm32canbus_serialif(const char *dev_name, int baudrate)
    : io(),
      port(io, "/dev/ttyACM0")
     
{
    port.set_option(boost::asio::serial_port_base::parity()); // default none
    port.set_option(boost::asio::serial_port_base::character_size(8));
    port.set_option(boost::asio::serial_port_base::stop_bits()); // default one
    port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
}


void stm32canbus_serialif::start()
{
    comm_thread = std::thread([this]()
                              { this->run(); });
}

void stm32canbus_serialif::stop()
{
    port.cancel();
    comm_thread.join();
}

void stm32canbus_serialif::run()

{
    read_some();
    io.run();
}


// para recibir datos por el puerto serie
void stm32canbus_serialif::read_handler(const boost::system::error_code &error, size_t bytes_transferred)
{
    if (error)
    {
        std::cout << error.message() << std::endl;
    }
    else
    {
        // can_message_event ev;
        for (size_t i = 0; i < bytes_transferred; i++)
        {
            

             std::cout<<"caracteres recibidos " << rx_buffer[i]<< std::endl;
        }
        read_some();
    }
}

void stm32canbus_serialif::read_some()
{
    port.async_read_some(boost::asio::buffer(rx_buffer, BUFSIZE), boost::bind(&stm32canbus_serialif::read_handler, this,
                                                                              boost::asio::placeholders::error,
                                                                              boost::asio::placeholders::bytes_transferred));
}




void stm32canbus_serialif::write_some()
{
    port.async_write_some(boost::asio::buffer(tx_buffer, 2), boost::bind(&stm32canbus_serialif::write_handler, this,
                                                                         boost::asio::placeholders::error,
                                                                  boost::asio::placeholders::bytes_transferred));
}

void stm32canbus_serialif::write_handler(const boost::system::error_code &error, size_t bytes_transferred)
{
    if (error)
    {
        std::cout << error.message() << std::endl;
    }
    else
    {

        tx_buffer[0] = 'P';
        tx_buffer[1] = opcodi;
         

   for (size_t i = 0; i < 2; i++)
    {
      

     //   std::cout << boost::format("%d ") % tx_buffer[i];
    
    }    
    
  
    }
}
void stm32canbus_serialif::itera()
{
    
     
        opcodi='P';
        write_some();
}


