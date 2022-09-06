#include "robot_arm_control/cds5500.hpp"

#include <iostream>
#include <numeric>
#include <sstream>

using namespace boost::asio;

namespace robot_arm_control
{
Cds5500::Cds5500(std::shared_ptr<SerialPort> serial, const std::string &name, const uint8_t id, const double offset)
    : serial_(serial), name_(name), id_(id), offset_(offset)
{
}

void Cds5500::request_update() { send(Instructions::READ, ControlTableAddress::PRESENT_POSITION_L, {6}); }

double Cds5500::get_position() { return position_; }

double Cds5500::get_velocity() { return velocity_; }

double Cds5500::get_effort() { return effort_; }

void Cds5500::set_led(const bool state) { send(Instructions::WRITE, ControlTableAddress::LED, {state}); }

void Cds5500::set_position(const double radians)
{
    uint16_t position_val = static_cast<uint16_t>((radians + offset_) / 300.0 / M_PI * 180.0 * 1024) + 512;

    send(Instructions::WRITE, ControlTableAddress::GOAL_POSITION_L,
         {static_cast<uint8_t>(position_val & 0xff), static_cast<uint8_t>(position_val >> 8)});
}

void Cds5500::send(const Instructions instruction, const ControlTableAddress start_address,
                   const std::vector<uint8_t> &parameters)
{
    std::vector<uint8_t> packet_out;
    packet_out.reserve(parameters.size() + 7);

    packet_out.push_back(0xff);
    packet_out.push_back(0xff);
    packet_out.push_back(id_);
    packet_out.push_back(parameters.size() + 3);
    packet_out.push_back(static_cast<uint8_t>(instruction));
    packet_out.push_back(static_cast<uint8_t>(start_address));
    packet_out.insert(packet_out.end(), parameters.begin(), parameters.end());
    packet_out.push_back(~std::accumulate(packet_out.begin() + 2, packet_out.end(), 0) & 0xff);

    serial_->write(packet_out);
}

void Cds5500::on_read(const std::vector<uint8_t> &data)
{
    for (uint8_t c : data)
    {
        if (packet_in_.size() < 2 && c != 0xff)
        {
            continue;
        }

        if (packet_in_.size() == 2 && c != id_)
        {
            packet_in_.clear();
            continue;
        }

        packet_in_.push_back(c);

        if (packet_in_.size() >= 4 && packet_in_.size() == static_cast<std::size_t>(packet_in_[3] + 4))
        {
            uint8_t checksum = ~std::accumulate(packet_in_.begin() + 2, packet_in_.end() - 1, 0) & 0xff;

            if (packet_in_.back() == checksum)
            {
                if (packet_in_.size() == 12)
                {
                    position_ = ((packet_in_[5] | packet_in_[6] << 8) - 512) * 300.0 * M_PI / 180.0 / 1024 - offset_;
                    velocity_ = (packet_in_[7] | packet_in_[8] << 8) * 1024 / 62.0 * M_PI / 60.0;
                    effort_ = packet_in_[9] | packet_in_[10] << 8;
                }
            }

            packet_in_.clear();
        }
    }
}

}  // namespace robot_arm_control
