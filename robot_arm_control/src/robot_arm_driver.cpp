#include <boost/asio.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "robot_arm_control/cds5500.hpp"

using boost::asio::io_service;
using boost::asio::serial_port;
using namespace std::chrono_literals;

namespace robot_arm_control
{
class RobotArmDriver : public rclcpp::Node
{
    io_service io_;
    std::shared_ptr<SerialPort> serial_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

    std::vector<Cds5500> servos_;

    void hello(uint8_t *data, std::size_t)
    {
        std::cout << "hello!\n";
        std::cout << data[2] << "\n";
    }

   public:
    RobotArmDriver() : Node("robot_arm_driver")
    {
        declare_parameter<std::string>("port", "/dev/serial/by-id/usb-FTDI_Toms_FTDI_Cable_A8WW0LV8-if00-port0");
        declare_parameter<int>("baudrate", 1000000);

        serial_ = std::make_shared<SerialPort>(get_parameter("port").as_string(), get_parameter("baudrate").as_int());

        servos_.emplace_back(serial_, "shoulder_psi", 0x10, 0.0);
        servos_.emplace_back(serial_, "shoulder_theta", 0x11, 0.04601942363656919);
        servos_.emplace_back(serial_, "elbow", 0x12, -0.0869255779801863);
        servos_.emplace_back(serial_, "wrist_theta", 0x13, 0.04090615434361711);
        servos_.emplace_back(serial_, "wrist_psi", 0x14, 0.20964404101103762);

        for (auto &servo : servos_)
        {
            servo.set_position(0.0);
        }

        joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        timer_ = create_wall_timer(100ms, std::bind(&RobotArmDriver::update, this));
    }

    void update()
    {
        sensor_msgs::msg::JointState msg;

        msg.header.stamp = get_clock()->now();

        for (auto &servo : servos_)
        {
            servo.request_update();
            get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.001));
        }

        auto data = serial_->read();

        for (auto &servo : servos_)
        {
            servo.on_read(data);

            msg.name.push_back(servo.name_);
            msg.position.push_back(servo.get_position());
            msg.velocity.push_back(servo.get_velocity());
            msg.effort.push_back(servo.get_effort());
        }

        joint_states_pub_->publish(msg);
    }
};
}  // namespace robot_arm_control

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_arm_control::RobotArmDriver>());
    rclcpp::shutdown();
    return 0;
}