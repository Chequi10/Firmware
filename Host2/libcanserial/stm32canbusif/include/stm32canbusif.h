#pragma once

#include <iostream>
#include <atomic>
#include <thread>
#include <fstream>
#include <array>

#include <unistd.h>

#include <boost/ref.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>



/**
 * @brief Clase para controlar la interfaz CAN implementada en STM32 a través de puerto serie.
 */
class stm32canbus_serialif 
{
public:
    /**
     * @brief Evento de recepción.
     */
    struct can_message_event
    {
        uint32_t device_id;  /**< Identificador del dispositivo: 0,1,... */
        uint32_t event_type; /**< Por ahora un ùnico tipo de evento: recepción, pero se pueden agregar otros: status, error, etc.  */
        uint32_t canid;      /**< CANID (32bits para soportar standard y extended). */
        uint32_t len;        /**< Cantidad de bytes del mensaje (máximo 8 bytes). */
        uint8_t data[8];     /**< Datos del mensaje. */
    };

    /**
     * @brief Callback para cuando llega un mensaje.
     */
    using on_can_message_callback = std::function<void(const can_message_event &)>;

    static constexpr size_t BUFSIZE = 64;

    /**
     * @brief Constructor por defecto.
     *
     * @param dev_name Nombre del puerto serie (ej: /dev/ttyACM0).
     * @param baudrate Baudrate (recomendado 115200).
     * @param on_event_callback Callback definido por el usuario para responder cuando llega un mensaje.
     */
    stm32canbus_serialif(const char *dev_name, int baudrate);

    /**
     * @brief Iniciar monitor/controlador. Cuando el controlador está arrancado, se habilita el monitoreo y capacidad de envío de mensajes.
     */
    void start();

    /**
     * @brief Detener monitor/controlador.
     */
    void stop();
    void read_some();
    void write_some();
    uint opcodi{0};
    void itera();

private:
    boost::asio::io_service io;
    boost::asio::serial_port port;
    // boost::asio::streambuf buffer;
    std::array<char, BUFSIZE> rx_buffer;
    std::array<char, BUFSIZE> tx_buffer;
    std::thread comm_thread;
    std::atomic<bool> keep_running;
    on_can_message_callback on_event_callback;

    void read_handler(const boost::system::error_code &error, size_t bytes_transferred);
    void write_handler(const boost::system::error_code &error, size_t bytes_transferred);
    
    // void write_some();
    std::string response_get(std::size_t length);
    void run();

};