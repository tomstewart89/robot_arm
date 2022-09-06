import serial
from robot_arm_driver.cds5500 import CDS5500
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState


class RobotArmDriver(Node):
    def __init__(self):
        super().__init__("robot_arm_driver")
        self.declare_parameter("port", "/dev/serial/by-id/usb-FTDI_Toms_FTDI_Cable_A8WW0LV8-if00-port0")
        self.declare_parameter("baudrate", 1000000)

        self.serial_port = serial.Serial(
            port=self.get_parameter("port").get_parameter_value().string_value,
            baudrate=self.get_parameter("baudrate").get_parameter_value().integer_value,
            timeout=0.1,
        )

        self.joint_info = [
            ("shoulder_psi", 0x10, 0.0),
            ("shoulder_theta", 0x11, 0.04601942363656919),
            ("elbow", 0x12, -0.0869255779801863),
            ("wrist_theta", 0x13, 0.04090615434361711),
            ("wrist_psi", 0x14, 0.20964404101103762),
        ]

        self.servos = {name: CDS5500(id, offset, self.serial_port) for name, id, offset in self.joint_info}

        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(0.01, self.timer_cb)
        self.follow_joint_trajectory = ActionServer(
            self, FollowJointTrajectory, "follow_joint_trajectory", self.follow_joint_trajectory_cb
        )

    def follow_joint_trajectory_cb(self, goal):
        self.goal = goal

        self.get_logger().info("Executing goal...")
        result = FollowJointTrajectory.Result()
        return result

    def timer_cb(self):

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for name, servo in self.servos.items():
            try:
                position, velocity, load = servo.read_state()
            except IOError:
                break

            msg.name.append(name)
            msg.position.append(position)
            msg.velocity.append(velocity)
            msg.effort.append(load)
        else:
            self.joint_state_pub.publish(msg)


def main():
    rclpy.init()
    robot_arm = RobotArmDriver()
    rclpy.spin(robot_arm)


if __name__ == "__main__":
    main()
