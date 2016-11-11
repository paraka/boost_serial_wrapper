#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <string>
#include <vector>
#include <memory>
#include <boost/signals2.hpp>

class Serial
{
public:
    // Constructor
    explicit Serial(const std::string &port_name, bool blocking_reader = false);

    // Destructor: close connection and clean everything
    ~Serial();

    // open connection with serial port
    void open();

    // close connection with serial port
    void close();

    // checks if serial port is open 
    bool is_open() const;

    // serial port parity configuration
    enum class parity_type { NONE, ODD, EVEN };
    
    void set_parity(parity_type parity = DEFAULT_PARITY);

    // serial port stop bits consfiguration
    enum class stop_bits_type { ONE, ONEPOINTFIVE, TWO };
    
    void set_stop_bits(stop_bits_type stop_bits = DEFAULT_STOP_BITS);

    // serial port flow control configuration
    enum class flow_control_type { NONE, HARDWARE, SOFTWARE };

    void set_flow_control(flow_control_type flow_control = DEFAULT_FLOW_CONTROL);
    
    // serial port character size configuration
    void set_character_size(unsigned int size = DEFAULT_CHARACTER_SIZE);
    
    // serial port baud rate configuration
    void set_baud_rate(unsigned int baud_rate = DEFAULT_BAUD_RATE);

    // set delimiter to read until in async connection
    void set_delimiter(const std::string &delim);

    // starts async reception
    void start_async_reception();

    // Checks if has to quit from async reception
    bool quit() const;

    // Forces async reception to quit
    void force_quit();

    // This read data until it ends of timeout expired (blocking call)
    bool read_data(std::vector<char> &buf, std::size_t ms_timeout = DEFAULT_MILLISECONDS_TIMEOUT);

    // Write in serial port in different forms. All of them are blocking calls
    void send_data(const std::string& data);
    
    void send_data(const char *data, std::size_t data_len);
    
    void send_data(const std::vector<char>& data);

private:
    void set_default_configuration();

public:
    // will be emited when async data is being received
    boost::signals2::signal<void(std::string)> on_data_recv;

    // will be emited when async data reception get an error
    boost::signals2::signal<void(std::string)> on_error_receiving_data;

private:
    class SerialImpl;
    using SerialImplPtr = std::unique_ptr<SerialImpl>;
    SerialImplPtr impl_;

    // static const stuff
    static const unsigned int DEFAULT_BAUD_RATE = 115200;
    static const parity_type DEFAULT_PARITY = parity_type::NONE;
    static const unsigned int DEFAULT_CHARACTER_SIZE = 8;
    static const stop_bits_type DEFAULT_STOP_BITS = stop_bits_type::ONE;
    static const flow_control_type DEFAULT_FLOW_CONTROL = flow_control_type::NONE;
    static const std::size_t DEFAULT_MILLISECONDS_TIMEOUT = 500;
};

#endif // _SERIAL_H_
