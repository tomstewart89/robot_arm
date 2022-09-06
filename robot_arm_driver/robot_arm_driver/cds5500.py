import math
from enum import Enum


class CDS5500:
    class ControlTableAddress(Enum):
        MODEL_NUMBER_L = 0
        MODEL_NUMBER_H = 1
        VERSION = 2
        ID = 3
        BAUD_RATE = 4
        RETURN_DELAY_TIME = 5
        CW_ANGLE_LIMIT_L = 6
        CW_ANGLE_LIMIT_H = 7
        CCW_ANGLE_LIMIT_L = 8
        CCW_ANGLE_LIMIT_H = 9
        SYSTEM_DATA2 = 10
        LIMIT_TEMPERATURE = 11
        DOWN_LIMIT_VOLTAGE = 12
        UP_LIMIT_VOLTAGE = 13
        MAX_TORQUE_L = 14
        MAX_TORQUE_H = 15
        RETURN_LEVEL = 16
        ALARM_LED = 17
        ALARM_SHUTDOWN = 18
        OPERATING_MODE = 19
        DOWN_CALIBRATION_L = 20
        DOWN_CALIBRATION_H = 21
        UP_CALIBRATION_L = 22
        UP_CALIBRATION_H = 23
        TORQUE_ENABLE = 24
        LED = 25
        CW_COMPLIANCE_MARGIN = 26
        CCW_COMPLIANCE_MARGIN = 27
        CW_COMPLIANCE_SLOPE = 28
        CCW_COMPLIANCE_SLOPE = 29
        GOAL_POSITION_L = 30
        GOAL_POSITION_H = 31
        GOAL_SPEED_L = 32
        GOAL_SPEED_H = 33
        TORQUE_LIMIT_L = 34
        TORQUE_LIMIT_H = 35
        PRESENT_POSITION_L = 36
        PRESENT_POSITION_H = 37
        PRESENT_SPEED_L = 38
        PRESENT_SPEED_H = 39
        PRESENT_LOAD_L = 40
        PRESENT_LOAD_H = 41
        PRESENT_VOLTAGE = 42
        PRESENT_TEMPERATURE = 43
        REGISTERED_INSTRUCTION = 44
        PAUSE_TIME = 45
        MOVING = 46
        LOCK = 47
        PUNCH_L = 48
        PUNCH_H = 49

    class Instructions(Enum):
        PING = 0x01
        READ = 0x02
        WRITE = 0x03
        REG_WRITE = 0x04
        ACTION = 0x05
        RESET = 0x06
        DIGITAL_RESET = 0x07
        SYSTEM_READ = 0x0C
        SYSTEM_WRITE = 0x0D
        SYNC_WRITE = 0x83
        SYNC_REG_WRITE = 0x84

    RPM_TO_RADS = 2 * math.pi / 60

    def __init__(self, id, offset, serial_port):
        self.serial_port = serial_port
        self.id = id
        self.offset = offset

    def to_position(self, byte_vals):
        return (int.from_bytes(byte_vals, "little") - 512) * math.radians(300.0) / 1024 - self.offset

    def to_velocity(self, byte_vals):
        return int.from_bytes(byte_vals, "little") * 1024 / 62.0 * CDS5500.RPM_TO_RADS

    def to_load(self, byte_vals):
        return int.from_bytes(byte_vals, "little")

    def from_position(self, radians):
        byte_val = int((radians + self.offset) / math.radians(300.0) * 1024) + 512
        return [byte_val & 0xFF, byte_val >> 8]

    def set_led(self, state):
        self.send(CDS5500.Instructions.WRITE.value, CDS5500.ControlTableAddress.LED.value, state)

    def set_angle(self, radians):
        self.send(
            CDS5500.Instructions.WRITE.value,
            CDS5500.ControlTableAddress.GOAL_POSITION_L.value,
            self.from_position(radians),
        )

    def read_state(self):
        self.serial_port.flush()
        self.send(CDS5500.Instructions.READ.value, CDS5500.ControlTableAddress.PRESENT_POSITION_L.value, 6)
        packet = self.serial_port.read(12)

        if len(packet) != 12 or ~sum(packet[2:-1]) & 0xFF != packet[-1]:
            raise IOError()

        return self.to_position(packet[5:7]), self.to_velocity(packet[7:9]), self.to_load(packet[9:11])

    def send(self, instruction, start_address, *parameters):

        payload = [instruction, start_address] + list(parameters)
        header = [self.id, len(payload) + 1]
        checksum = ~sum(header + payload) & 0xFF

        self.serial_port.write(bytearray([0xFF, 0xFF] + header + payload + [checksum]))
