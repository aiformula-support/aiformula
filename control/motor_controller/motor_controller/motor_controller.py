#!/usr/bin/env python
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from can_msgs.msg import Frame
from typing import List


class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.declare_parameter('wheel.tread')
        self.declare_parameter('wheel.diameter')
        self.declare_parameter('wheel.gear_ratio')
        self.declare_parameter('publish_timer_loop_duration')
        self.tread = self.get_parameter('wheel.tread').get_parameter_value().double_value
        self.diameter = self.get_parameter('wheel.diameter').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('wheel.gear_ratio').get_parameter_value().double_value
        publish_timer_loop_duration = self.get_parameter(
            'publish_timer_loop_duration').get_parameter_value().double_value

        buffer_size = 10
        self.twist_sub = self.create_subscription(Twist, 'sub_speed_command', self.twist_callback, buffer_size)
        self.can_pub = self.create_publisher(Frame, 'pub_can', buffer_size)
        self.publish_timer = self.create_timer(publish_timer_loop_duration, self.publish_canframe_callback)
        self.frame_msg = Frame()

    def twist_callback(self, msg):
        rpm = self.toRefRPM(msg.linear.x, msg.angular.z)
        cmd_right = self.toCanCmd(rpm[0])
        cmd_left = self.toCanCmd(rpm[1])
        can_data = cmd_right + cmd_left
        self.frame_msg.header.frame_id = "can0"        # Default can0
        self.frame_msg.id = 0x210                      # MotorControler CAN ID : 0x210
        self.frame_msg.dlc = 8                         # Data length
        self.frame_msg.data = can_data

    def publish_canframe_callback(self):
        self.can_pub.publish(self.frame_msg)

#  Velocity -> RPM Calc
#  V_right = (V + tread/2 * w), V_left = (V - tread / 2 * w ) [m/s]
#  w_right = V/r + (d/2r) * w   [rad/s]
#  w_left = V/r - (d/2r) * w    [rad/s]
#  rpm = w * 60 / 2* pi [rpm]
#  rpm = rpm * gear_ratio  [rpm]

    def toRefRPM(self, linear_velocity, angular_velocity):  # Calc Motor ref rad/s
        wheel_angular_velocities = np.array([(linear_velocity / (self.diameter * 0.5)) + (self.tread / self.diameter) * angular_velocity,  # right[rad/s]
                                             (linear_velocity / (self.diameter * 0.5)) - (self.tread / self.diameter) * angular_velocity])  # left[rad/s]
        minute_to_second = 60
        rpm = wheel_angular_velocities * (minute_to_second / (2 * math.pi))
        return (rpm * self.gear_ratio).tolist()

    @staticmethod
    def toCanCmd(rpm: float) -> List[int]:
        rounded = round(rpm)
        bytes = rounded.to_bytes(4, "little", signed=True)
        return list(bytes)


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
