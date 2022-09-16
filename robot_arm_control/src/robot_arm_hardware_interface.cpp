
#include "robot_arm_control/robot_arm_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_arm_control
{
hardware_interface::CallbackReturn RobotArmPositionOnlyHardware::on_init(const hardware_interface::HardwareInfo &info)
{
    serial_ = std::make_shared<SerialPort>(info.hardware_parameters.at("port"),
                                           std::stoi(info.hardware_parameters.at("baudrate")));

    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    states_.resize(info_.joints.size(), 0.0);
    commands_.resize(info_.joints.size(), 0.0);

    broadcast_ = std::make_unique<robot_arm_control::Cds5500>(serial_, "broadcast", 0xfe, 0.0);

    for (const auto &joint : info_.joints)
    {
        // RobotArmPositionOnlyHardware has exactly one state and command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(rclcpp::get_logger("RobotArmPositionOnlyHardware"),
                         "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                         joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("RobotArmPositionOnlyHardware"),
                         "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                         joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1)
        {
            RCLCPP_FATAL(rclcpp::get_logger("RobotArmPositionOnlyHardware"),
                         "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
                         joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("RobotArmPositionOnlyHardware"),
                         "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        joints_.emplace_back(serial_, joint.name, std::stoi(joint.parameters.at("id")),
                             std::stod(joint.parameters.at("offset")), std::stod(joint.parameters.at("multiplier")));
    }

    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        joints_[i].queue_move(commands_[i]);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotArmPositionOnlyHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        try
        {
            states_[i] = joints_[i].get_position();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotArmPositionOnlyHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        joints_[i].queue_move(commands_[i]);
    }

    broadcast_->execute();

    // not clear why this is necessary, but without it get_position returns a weird status packet, so go figure.
    joints_[0].ping();

    return hardware_interface::return_type::OK;
}

}  // namespace robot_arm_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robot_arm_control::RobotArmPositionOnlyHardware, hardware_interface::SystemInterface)