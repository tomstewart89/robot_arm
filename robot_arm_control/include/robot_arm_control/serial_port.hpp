#include <boost/asio.hpp>

namespace robot_arm_control
{

class SerialPort
{
   public:
    SerialPort(const std::string& port_name, const int baudrate);

    void write(const std::vector<uint8_t>& data);
    std::vector<uint8_t> read(const std::size_t len, const unsigned int timeout_ms = 50);
    void flush();

   private:
    void on_read(const boost::system::error_code& error, std::size_t bytes_transferred);
    void on_timeout(const boost::system::error_code& error);
    std::array<uint8_t, 256> buffer_;
    std::vector<uint8_t> read_queue_;

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    boost::asio::deadline_timer timeout_;
};

}  // namespace robot_arm_control
