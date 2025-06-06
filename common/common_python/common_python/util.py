import os

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time


def abort(node: Node) -> None:
    """Abnormal termination

    Parameters:
    ----------
    `node`: The instance of the `Node` class

    Examples:
    ----------
    >>> abort(self)
    """
    node.destroy_node()
    rclpy.shutdown()
    exit(1)


def get_ros_distro() -> str:
    """Get ROS2 version

    Examples:
    ----------
    >>> if get_ros_distro() == "humble":
    """
    ros_distro = os.getenv("ROS_DISTRO")
    if ros_distro:
        return ros_distro
    else:
        print("Environmental Variable 'ROS Distro' is not set.")
        rclpy.shutdown()
        exit(1)


def to_timestamp_float(timestamp_msg: Time) -> float:
    """Convert timestamp in msg to float value

    Parameters:
    ----------
    `timestamp_msg`: The instance of the `builtin_interfaces.msg` class

    Examples:
    ----------
    >>> timestamp_double = to_timestamp_float(msg.header.stamp)
    """
    return timestamp_msg.sec + timestamp_msg.nanosec / 1e9
