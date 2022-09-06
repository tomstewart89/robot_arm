#include <boost/asio.hpp>
#include <mutex>
#include <thread>

namespace robot_arm_control
{

class SerialPort
{
   public:
    SerialPort(const std::string& port_name, const int baudrate);

    ~SerialPort();

    void write(const std::vector<uint8_t>& packet);
    std::vector<uint8_t> read();

   private:
    void on_read(const boost::system::error_code& error, std::size_t bytes_transferred);

    std::array<uint8_t, 256> buffer_;
    std::vector<uint8_t> read_queue_;

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    std::thread thread_;
    std::mutex read_queue_mutex_;
};

}  // namespace robot_arm_control
