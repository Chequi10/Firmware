#include <boost/format.hpp>
#include "stm32canbusif.h"

stm32canbus_serialif::stm32canbus_serialif(const char *dev_name, int baudrate, on_can_message_callback on_event_callback)
    : io(),
      port(io, dev_name),
      on_event_callback(on_event_callback)
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
            ::protocol::packet_decoder::feed(rx_buffer[i]);
            ::protocol::packet_decoder::check_timeouts();

            // std::cout << rx_buffer[i];
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

void stm32canbus_serialif::handle_packet(const uint8_t *payload, size_t n)
{

    switch (payload[0])
    {
    // Event 0: received message.
    case '0':
    {

        can_message_event ev;

        ev.device_id = payload[1];
        ev.canid = payload[2];
        ev.len = payload[3];
        ev.data[8];

        for (size_t i = 0; i < 5; i++)
        {
            ev.data[i] = payload[i + 1];
            // std::cout << boost::format(" %d") % ev.data[i];
        }
        on_event_callback(ev);
    }
    break;

    default:
    {

        // error, unknown packet
        this->set_error(error_code::unknown_opcode);
        this->reset();
    }
    }
}

// para enviar datos por el puerto serie
void stm32canbus_serialif::send_impl(const uint8_t *buf, uint8_t n)
{

    for (size_t i = 0; i < n; i++)
    {
        tx_buffer[i] = *buf++;

        std::cout << boost::format("%d ") % tx_buffer[i];
    }
   
}


void stm32canbus_serialif::write_some()
{
    port.async_write_some(boost::asio::buffer(tx_buffer, 12), boost::bind(&stm32canbus_serialif::write_handler, this,
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
        ::protocol::packet_encoder::send(0x4);
        ::protocol::packet_decoder::check_timeouts();
    }
  
    
}

// para ver errores y estado de la conexion
void stm32canbus_serialif::set_error(error_code ec)
{
    std::cout << boost::format("Error debido a: %c ") % ec << std::endl;
}

void stm32canbus_serialif::handle_connection_lost()
{
    // std::cout << boost::format("Se perdio la conexion") << std::endl;
}