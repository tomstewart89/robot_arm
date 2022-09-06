#include "robot_arm_control/serial_port.hpp"

#include <boost/bind/bind.hpp>
#include <iostream>

using namespace boost::asio;

namespace robot_arm_control
{

SerialPort::SerialPort(const std::string& port_name, const int baudrate) : serial_(io_, port_name)
{
    serial_.set_option(serial_port_base::baud_rate(baudrate));

    serial_.async_read_some(
        buffer(buffer_.data(), buffer_.size()),
        boost::bind(&SerialPort::on_read, this, placeholders::error, placeholders::bytes_transferred));

    std::thread t([this]() { io_.run(); });
    thread_.swap(t);
}

SerialPort::~SerialPort()
{
    serial_.cancel();
    thread_.join();
}

void SerialPort::write(const std::vector<uint8_t>& packet)
{
    boost::asio::write(serial_, buffer(packet.data(), packet.size()));
}

std::vector<uint8_t> SerialPort::read()
{
    std::vector<uint8_t> out;

    {
        std::lock_guard<std::mutex> l(read_queue_mutex_);
        read_queue_.swap(out);
    }

    return out;
}

void SerialPort::on_read(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if (!error)
    {
        {
            std::lock_guard<std::mutex> l(read_queue_mutex_);
            read_queue_.insert(read_queue_.begin(), buffer_.begin(), buffer_.begin() + bytes_transferred);
        }

        serial_.async_read_some(
            buffer(buffer_.data(), buffer_.size()),
            boost::bind(&SerialPort::on_read, this, placeholders::error, placeholders::bytes_transferred));
    }
}

}  // namespace robot_arm_control
