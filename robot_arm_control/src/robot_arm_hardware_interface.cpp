// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
    serial_ = std::make_shared<SerialPort>("/dev/serial/by-id/usb-FTDI_Toms_FTDI_Cable_A8WW0LV8-if00-port0", 1000000);

    servos_.emplace_back(serial_, "shoulder_psi", 0x10, 0.0);
    servos_.emplace_back(serial_, "shoulder_theta", 0x11, 0.04601942363656919);
    servos_.emplace_back(serial_, "elbow", 0x12, -0.0869255779801863);
    servos_.emplace_back(serial_, "wrist_theta", 0x13, 0.04090615434361711);
    servos_.emplace_back(serial_, "wrist_psi", 0x14, 0.20964404101103762);

    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
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
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmPositionOnlyHardware::on_configure(const rclcpp_lifecycle::State &)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotArmPositionOnlyHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotArmPositionOnlyHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn RobotArmPositionOnlyHardware::on_activate(const rclcpp_lifecycle::State &)
{
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("RobotArmPositionOnlyHardware"), "Activating ...please wait...");

    // command and state should be equal when starting
    for (uint i = 0; i < hw_states_.size(); i++)
    {
        hw_commands_[i] = hw_states_[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("RobotArmPositionOnlyHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmPositionOnlyHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotArmPositionOnlyHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (auto &servo : servos_)
    {
        servo.request_update();
        rclcpp::sleep_for(std::chrono::nanoseconds(1000));
    }

    auto data = serial_->read();

    for (std::size_t i = 0; i < servos_.size(); ++i)
    {
        servos_[i].on_read(data);
        hw_states_[i] = servos_[i].get_position();
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotArmPositionOnlyHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (std::size_t i = 0; i < servos_.size(); ++i)
    {
        servos_[i].set_position(hw_commands_[i]);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace robot_arm_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robot_arm_control::RobotArmPositionOnlyHardware, hardware_interface::SystemInterface)