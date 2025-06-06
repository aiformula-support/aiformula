import numpy as np
from enum import IntEnum
import textwrap

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from common_python.get_ros_parameter import get_ros_parameter
from common_python.util import to_timestamp_float


class Button(IntEnum):
    CROSS = 0
    SQUARE = 1
    CIRCLE = 2
    TRIANGLE = 3
    PADDLE_SHIFT_R = 4
    PADDLE_SHIFT_L = 5
    R2 = 6
    L2 = 7
    SHARE = 8
    OPTION = 9
    R3 = 10
    L3 = 11
    PLUS = 19
    MINUS = 20
    RED_DIAL_CLOCKWISE = 21
    RED_DIAL_COUNTERCLOCKWISE = 22
    ENTER_ICON = 23
    PLAY_STATION = 24


class Axis(IntEnum):
    STEERING = 0          # Full Left Turn: 1.0, Full Right Turn: -1.0
    CLUTCH = 1            # Natural: 1.0, Full Throttle: -1.0
    ACCEL = 2             # Natural: 1.0, Full Throttle: -1.0
    BRAKE = 3             # Natural: 1.0, Full Throttle: -1.0
    CROSS_HORIZONTAL = 4  # Left: 1.0, Right: -1.0
    CROSS_VERTICAL = 5    # Up  : 1.0, Down : -1.0


class TeleopTwistHandleController(Node):
    def __init__(self):
        super().__init__('teleop_twist_handle_controller_node')
        self.get_ros_params()
        # See “Acceleration Model” in `apply_acceleration()` below.
        self.drag_coefficient = self.max_linear_acceleration / self.max_linear_vel ** self.drag_exponent
        self.was_accel_pressed = False
        self.prev_time = 0.0
        self.twist_msg = Twist()
        self.print_params()

        # Publisher & Subscriber
        buffer_size = 10
        self.joy_sub = self.create_subscription(Joy, 'sub_joy', self.joy_callback, buffer_size)
        self.twist_pub = self.create_publisher(Twist, 'pub_cmd_vel', buffer_size)
        self.coasting_twist_pub = self.create_publisher(Twist, 'pub_cmd_vel_coasting', buffer_size)
        self.twist_mux_lock_pub = self.create_publisher(Bool, 'pub_twist_mux_lock', buffer_size)

    def get_ros_params(self):
        self.max_linear_vel = get_ros_parameter(self, "max_linear_vel")
        self.max_angular_vel = get_ros_parameter(self, "max_angular_vel")
        self.max_linear_acceleration = get_ros_parameter(self, "max_linear_acceleration")
        self.drag_exponent = get_ros_parameter(self, "drag_exponent")
        self.stopping_vel = get_ros_parameter(self, "stopping_vel")

    def print_params(self):
        self.get_logger().debug(textwrap.dedent(f"""
        ============================
        (teleop_twist_handle_controller.yaml)
          max_linear_vel          : {self.max_linear_vel:.2f} [m/s]
          max_angular_vel         : {self.max_angular_vel:.2f} [rad/s]
          max_linear_acceleration : {self.max_linear_acceleration:.2f} [m/s^2]
          drag_exponent           : {self.drag_exponent:.2f}
          stopping_vel            : {self.stopping_vel:.2f} [m/s]

        (initialize)
          drag_coefficient        : {self.drag_coefficient:.2f} [m/s]
        ============================\n"""))

    def joy_callback(self, joy_msg):
        brake_ratio = (joy_msg.axes[Axis.BRAKE] + 1.0) * 0.5  # raw:-1.0 ~ 1.0 -> ratio: 0 ~ 1.0
        accel_ratio = (joy_msg.axes[Axis.ACCEL] + 1.0) * 0.5
        steering_ratio = joy_msg.axes[Axis.STEERING]

        current_time = to_timestamp_float(joy_msg.header.stamp)
        if not self.prev_time:
            self.prev_time = current_time
            return
        dt = current_time - self.prev_time
        self.prev_time = current_time

        if brake_ratio:
            self.lock_low_priority_speed_commands()
            self.twist_msg.linear.x = 0.0
        else:
            if accel_ratio:
                self.unlock_low_priority_speed_commands()
            self.apply_acceleration(accel_ratio, dt)
            if accel_ratio == 0.0 and self.twist_msg.linear.x < self.stopping_vel:
                self.twist_msg.linear.x = 0.0

        self.twist_msg.angular.z = self.max_angular_vel * steering_ratio

        if accel_ratio or brake_ratio:
            self.twist_pub.publish(self.twist_msg)
        else:
            self.coasting_twist_pub.publish(self.twist_msg)

    def apply_acceleration(self, accel_ratio, dt):
        """ Acceleration Model
          V(t+1) = V(t) + (k * a - b * V(t)^n) * dt
        As t approaches infinity, V(t) reaches a steady state V(∞) = Vt:
          V(∞) = V(∞) + (k * a - b * V(∞)^n) * dt
          Vt   = Vt   + (k * a - b * Vt  ^n) * dt
        Solving this gives:
          b = k * a / Vt^n
        ------------------------------
        V(t) : velocity at time `t`
        V(t+1) : velocity at time `t + dt`
        Vt : terminal velocity
        a  : max acceleration
        k  : acceleration scaling factor (0 to 1)
        b  : drag coefficient
        n  : drag exponent
        """
        linear_acceleration = accel_ratio * self.max_linear_acceleration - \
            self.drag_coefficient * self.twist_msg.linear.x ** self.drag_exponent
        accelerated_linear_velocity = self.twist_msg.linear.x + linear_acceleration * dt
        self.twist_msg.linear.x = np.clip(accelerated_linear_velocity, 0.0, self.max_linear_vel)

    def lock_low_priority_speed_commands(self):
        lock_msg = Bool()
        lock_msg.data = True
        self.twist_mux_lock_pub.publish(lock_msg)
        self.get_logger().debug("Lock the low-priority speed commands.")

    def unlock_low_priority_speed_commands(self):
        lock_msg = Bool()
        lock_msg.data = False
        self.twist_mux_lock_pub.publish(lock_msg)
        self.get_logger().debug("Unlock the low-priority speed commands.")


def main(args=None):
    rclpy.init(args=args)
    teleop_twist__handle_controller = TeleopTwistHandleController()
    rclpy.spin(teleop_twist__handle_controller)
    teleop_twist__handle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
