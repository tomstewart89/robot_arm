#include "robot_arm_control/serial_port.hpp"

#include <boost/bind/bind.hpp>
#include <iostream>

using namespace boost::asio;

namespace robot_arm_control
{

SerialPort::SerialPort(const std::string& port_name, const int baudrate) : serial_(io_, port_name), timeout_(io_)
{
    serial_.set_option(serial_port_base::baud_rate(baudrate));
}

void SerialPort::write(const std::vector<uint8_t>& data)
{
    boost::asio::write(serial_, buffer(data.data(), data.size()));
}

void SerialPort::flush() { read(buffer_.size(), 0); }

std::vector<uint8_t> SerialPort::read(std::size_t len, const unsigned int timeout_ms)
{
    timeout_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
    timeout_.async_wait(boost::bind(&SerialPort::on_timeout, this, placeholders::error));

    serial_.async_read_some(buffer(buffer_.data(), len), boost::bind(&SerialPort::on_read, this, placeholders::error,
                                                                     placeholders::bytes_transferred));

    io_.run();
    io_.reset();

    std::vector<uint8_t> out;
    read_queue_.swap(out);

    return out;
}

void SerialPort::on_read(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error)
    {
        read_queue_.insert(read_queue_.begin(), buffer_.begin(), buffer_.begin() + bytes_transferred);
        timeout_.cancel();
    }
}

void SerialPort::on_timeout(const boost::system::error_code& error)
{
    if (!error)
    {
        serial_.cancel();
    }
}

}  // namespace robot_arm_control
