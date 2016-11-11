#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/thread.hpp>
#include "Serial.h"

/* implementation details class */


class Serial::SerialImpl
{
public:
    SerialImpl(Serial &serial,
                const std::string &port_name,
                bool blocking_reader)
                    : serial_(serial)
                    , blocking_reader_(blocking_reader)
                    , port_(io_service_)
                    , read_error_(false)
                    , timer_(port_.get_io_service())
                    , port_name_(port_name)
                    , delim_(DEFAULT_DELIMITER)
                    , quit_(false)
    {
        std::cout << "Serial::Serial() : " << port_name.c_str() << std::endl;
    }

    ~SerialImpl()
    {
        port_.close();

        if (!blocking_reader_)
        {
            io_service_.stop();
            runner_.join();
        }
    }

    void open()
    {
        port_.open(port_name_);
    }

    bool is_open() const
    {
        return port_.is_open();
    }

    void close()
    {
        port_.close();
    }

    void set_parity(boost::asio::serial_port_base::parity::type parity)
    {
        boost::asio::serial_port_base::parity p(parity);
        port_.set_option(p);
    }

    void set_stop_bits(boost::asio::serial_port_base::stop_bits::type stop_bits)
    {
        boost::asio::serial_port_base::stop_bits sb(stop_bits);
        port_.set_option(sb);
    }

    void set_flow_control(boost::asio::serial_port_base::flow_control::type flow_control)
    {
        boost::asio::serial_port_base::flow_control fc(flow_control);
        port_.set_option(fc);
    }

    void set_character_size(unsigned int size)
    {
        port_.set_option(boost::asio::serial_port::character_size(size));
    }

    void set_baud_rate(unsigned int baud_rate)
    {
        port_.set_option(boost::asio::serial_port::baud_rate(baud_rate));
    }

    void set_delimiter(const std::string& delim)
    {
        delim_ = delim;
    }

    void start_async_reception()
    {
        if (port_.is_open() && !blocking_reader_)
        {
            runner_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
            std::cout << "Runner thread created with id: " << runner_.get_id() << std::endl;
            start_receive();
        }
    }

    // Called when an async read completes or has been cancelled
    void on_read_complete(const boost::system::error_code& error, size_t bytes_transferred)
    {
        std::cout << "read_complete callback!" << std::endl;

        read_error_ = (error || bytes_transferred == 0);

        // Read has finished, so cancel the timer.
        timer_.cancel();
    }

    // Called when the timer's deadline expires.
    void on_time_out(const boost::system::error_code& error) 
    {
        // timeout has been cancelled
        if (error)
            return;

        std::cout << "time_out callback!" << std::endl;

        // We have a timeout -> kill read operation
        // The read_complete callback will be called with an error
        port_.cancel();
    }

    bool read_data(std::vector<char> &buf, size_t timeout_ms)
    {
        // After a timeout & cancel it seems we need
        // to do a reset for subsequent reads to work.
        port_.get_io_service().reset();

        boost::asio::async_read(port_, boost::asio::buffer(buf), 
                                boost::bind(&Serial::SerialImpl::on_read_complete, this, _1, _2));


        // Setup a deadline time to implement our timeout.
        timer_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
        timer_.async_wait(boost::bind(&Serial::SerialImpl::on_time_out, this, _1));

        // This will block until desired bytes are read
        // or until the it is cancelled.
        port_.get_io_service().run();

        return !read_error_;
    }

    void send_data(const std::string &data)
    {
        if (!is_open()) return;
        boost::asio::write(port_, boost::asio::buffer(data));
    }

    void send_data(const char *data, std::size_t data_len)
    {
        if (!is_open()) return;
        boost::asio::write(port_, boost::asio::buffer(data, data_len));
    }

    void send_data(const std::vector<char>& data)
    {
        if (!is_open()) return;

        std::size_t len = data.size();
        char buf[len];

        std::copy(data.begin(), data.end(), buf);
        buf[len] = '\0';
        boost::asio::write(port_, boost::asio::buffer(buf, len));
    }

    void on_data_received(const boost::system::error_code &ec, std::size_t bytes_transferred)
    {
        std::cout << "on_data_received callback!" << std::endl;
        if (!ec)
        {
            boost::asio::streambuf::const_buffers_type bufs = buffer_.data();
            std::string astr(boost::asio::buffers_begin(bufs), boost::asio::buffers_begin(bufs) + buffer_.size());
            serial_.on_data_recv(astr);
        }
        else
        {
            std::cout << "Error receiving data..." << std::endl;
            serial_.on_error_receiving_data(ec.message());
        }

        start_receive();
    }

    void start_receive()
    {
        boost::asio::async_read_until(port_, buffer_, delim_, boost::bind(&Serial::SerialImpl::on_data_received, this, _1, _2));
    }

    // check if must quit
    bool quit() const { return quit_; }

    // force quit
    void force_quit() { quit_ = true; } 

private:
    Serial &serial_;
    boost::asio::io_service io_service_;
    boost::asio::serial_port port_;
    boost::thread runner_;
    boost::asio::streambuf buffer_;
    boost::asio::deadline_timer timer_;
    std::string port_name_;
    std::string delim_;
    bool blocking_reader_;
    bool read_error_;
    bool quit_;
    static const std::string DEFAULT_DELIMITER;
};

const std::string Serial::SerialImpl::DEFAULT_DELIMITER("\r\n");

/* Serial interface implementation */

Serial::Serial(const std::string &port_name, bool blocking_reader) 
{
    impl_ = std::make_unique<SerialImpl>(*this, port_name, blocking_reader);
}

Serial::~Serial() = default;

void Serial::open()
{
    impl_->open();
    set_default_configuration();
}

void Serial::close()
{
    impl_->close();
}

bool Serial::is_open() const
{
    return impl_->is_open();
}

void Serial::start_async_reception()
{
    impl_->start_async_reception();
}

void Serial::set_default_configuration()
{
    set_parity();
    set_stop_bits();
    set_flow_control();
    set_character_size();
    set_baud_rate();
}

void Serial::set_parity(parity_type parity)
{
    switch (parity)
    {
        case parity_type::NONE:
        {
            impl_->set_parity(boost::asio::serial_port_base::parity::none);
            break;
        }
        case parity_type::ODD:
        {
            impl_->set_parity(boost::asio::serial_port_base::parity::odd);
            break;
        }
        case parity_type::EVEN:
        {
            impl_->set_parity(boost::asio::serial_port_base::parity::even);
            break;
        }
    }
}

void Serial::set_stop_bits(stop_bits_type stop_bits)
{
    switch (stop_bits)
    {
        case stop_bits_type::ONE:
        {
            impl_->set_stop_bits(boost::asio::serial_port_base::stop_bits::one);
            break;
        }
        case stop_bits_type::ONEPOINTFIVE:
        {
            impl_->set_stop_bits(boost::asio::serial_port_base::stop_bits::onepointfive);
            break;
        }
        case stop_bits_type::TWO:
        {
            impl_->set_stop_bits(boost::asio::serial_port_base::stop_bits::two);
            break;
        }
    }
}

void Serial::set_flow_control(flow_control_type flow_control)
{
    switch (flow_control)
    {
        case flow_control_type::NONE:
        {
            impl_->set_flow_control(boost::asio::serial_port_base::flow_control::none);
            break;
        }
        case flow_control_type::HARDWARE:
        {
            impl_->set_flow_control(boost::asio::serial_port_base::flow_control::hardware);
            break;
        }
        case flow_control_type::SOFTWARE:
        {
            impl_->set_flow_control(boost::asio::serial_port_base::flow_control::software);
            break;
        }
    }
}

void Serial::set_character_size(unsigned int size)
{
    impl_->set_character_size(size);
}

void Serial::set_baud_rate(unsigned int baud_rate)
{
    impl_->set_baud_rate(baud_rate);
}

void Serial::set_delimiter(const std::string &delim)
{
    impl_->set_delimiter(delim);
}

bool Serial::read_data(std::vector<char> &buf, size_t timeout_ms)
{
    return impl_->read_data(buf, timeout_ms);
}

void Serial::send_data(const std::string &data)
{
    impl_->send_data(data);
}

void Serial::send_data(const char *data, std::size_t data_len)
{
    impl_->send_data(data, data_len);
}

void Serial::send_data(const std::vector<char>& data)
{
    impl_->send_data(data);
}

bool Serial::quit() const
{ 
    return impl_->quit(); 
}

void Serial::force_quit() 
{
    impl_->force_quit();
}
