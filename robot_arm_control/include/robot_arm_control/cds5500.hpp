#pragma once

#include <optional>

#include "robot_arm_control/serial_port.hpp"

namespace robot_arm_control
{
class Cds5500
{
    enum class ControlTableAddress
    {
        MODEL_NUMBER_L,
        MODEL_NUMBER_H,
        VERSION,
        ID,
        BAUD_RATE,
        RETURN_DELAY_TIME,
        CW_ANGLE_LIMIT_L,
        CW_ANGLE_LIMIT_H,
        CCW_ANGLE_LIMIT_L,
        CCW_ANGLE_LIMIT_H,
        SYSTEM_DATA2,
        LIMIT_TEMPERATURE,
        DOWN_LIMIT_VOLTAGE,
        UP_LIMIT_VOLTAGE,
        MAX_TORQUE_L,
        MAX_TORQUE_H,
        RETURN_LEVEL,
        ALARM_LED,
        ALARM_SHUTDOWN,
        OPERATING_MODE,
        DOWN_CALIBRATION_L,
        DOWN_CALIBRATION_H,
        UP_CALIBRATION_L,
        UP_CALIBRATION_H,
        TORQUE_ENABLE,
        LED,
        CW_COMPLIANCE_MARGIN,
        CCW_COMPLIANCE_MARGIN,
        CW_COMPLIANCE_SLOPE,
        CCW_COMPLIANCE_SLOPE,
        GOAL_POSITION_L,
        GOAL_POSITION_H,
        GOAL_SPEED_L,
        GOAL_SPEED_H,
        TORQUE_LIMIT_L,
        TORQUE_LIMIT_H,
        PRESENT_POSITION_L,
        PRESENT_POSITION_H,
        PRESENT_SPEED_L,
        PRESENT_SPEED_H,
        PRESENT_LOAD_L,
        PRESENT_LOAD_H,
        PRESENT_VOLTAGE,
        PRESENT_TEMPERATURE,
        REGISTERED_INSTRUCTION,
        PAUSE_TIME,
        MOVING,
        LOCK,
        PUNCH_L,
        PUNCH_H
    };

    enum class Instructions
    {
        PING = 1,
        READ = 2,
        WRITE = 3,
        REG_WRITE = 4,
        ACTION = 5,
        RESET = 6,
        DIGITAL_RESET = 7,
        SYSTEM_READ = 12,
        SYSTEM_WRITE = 13,
        SYNC_WRITE = 131,
        SYNC_REG_WRITE = 132
    };

    std::shared_ptr<SerialPort> serial_;

   public:
    Cds5500(std::shared_ptr<SerialPort> serial, const std::string &name, const uint8_t id, const double offset = 0.0);

    bool ping();
    void set_led(const bool state);
    void queue_move(const double position, const double velocity = 0.0);
    void execute();

    const std::string name_;

    double get_position();

   private:
    const uint8_t id_;
    const double offset_;

    static constexpr double max_position = 300.0 / 180.0 * M_PI;  // 300 degrees
    static constexpr double max_velocity = M_PI / (0.18 * 3.0);   // 0.18 sec / 60 degrees

    void send(const Instructions instruction, const std::vector<uint8_t> &parameters);
    std::optional<std::vector<uint8_t>> receive();
};

}  // namespace robot_arm_control
