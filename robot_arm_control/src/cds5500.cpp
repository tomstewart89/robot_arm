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

double Cds5500::get_position()
{
    send(Instructions::READ, {static_cast<uint8_t>(ControlTableAddress::PRESENT_POSITION_L), 2});

    auto packet = *receive();

    return ((packet[5] | packet[6] << 8) - 512) * 300.0 * M_PI / 180.0 / 1024 - offset_;
}

bool Cds5500::ping()
{
    send(Instructions::PING, {});

    return receive().has_value();
}

void Cds5500::set_led(const bool state)
{
    send(Instructions::WRITE, {static_cast<uint8_t>(ControlTableAddress::LED), state});

    if (id_ != 0xfe)
    {
        receive();
    }
}

void Cds5500::queue_move(const double position, const double velocity)
{
    uint16_t position_val = static_cast<uint16_t>((position + offset_) / max_position * 1024) + 512;
    uint16_t velocity_val = static_cast<uint16_t>(velocity / max_velocity * 1024);

    send(Instructions::REG_WRITE, {static_cast<uint8_t>(ControlTableAddress::GOAL_POSITION_L),
                                   static_cast<uint8_t>(position_val & 0xff), static_cast<uint8_t>(position_val >> 8),
                                   static_cast<uint8_t>(velocity_val & 0xff), static_cast<uint8_t>(velocity_val >> 8)});

    if (id_ != 0xfe)
    {
        receive();
    }
}

void Cds5500::execute()
{
    send(Instructions::ACTION, {});

    if (id_ != 0xfe)
    {
        receive();
    }
}

void Cds5500::send(const Instructions instruction, const std::vector<uint8_t> &parameters)
{
    std::vector<uint8_t> packet_out;
    packet_out.reserve(parameters.size() + 6);

    packet_out.push_back(0xff);
    packet_out.push_back(0xff);
    packet_out.push_back(id_);
    packet_out.push_back(parameters.size() + 2);
    packet_out.push_back(static_cast<uint8_t>(instruction));
    packet_out.insert(packet_out.end(), parameters.begin(), parameters.end());
    packet_out.push_back(~std::accumulate(packet_out.begin() + 2, packet_out.end(), 0) & 0xff);

    serial_->write(packet_out);
}

std::optional<std::vector<uint8_t>> Cds5500::receive()
{
    std::vector<uint8_t> packet;

    for (auto data = serial_->read(1); !data.empty(); data = serial_->read(1))
    {
        uint8_t &c = data[0];

        if (packet.size() < 2 && c != 0xff)
        {
            continue;
        }

        if (packet.size() == 2 && c != id_)
        {
            packet.clear();
            continue;
        }

        packet.push_back(c);

        if (packet.size() >= 4 && packet.size() == static_cast<std::size_t>(packet[3] + 4))
        {
            uint8_t checksum = ~std::accumulate(packet.begin() + 2, packet.end() - 1, 0) & 0xff;

            if (packet.back() == checksum)
            {
                return packet;
            }
        }
    }

    return std::nullopt;
}

}  // namespace robot_arm_control
