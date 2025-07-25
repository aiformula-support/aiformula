from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    PACKAGE_NAME = "rear_potentiometer"

    rear_potentiometer = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_sensing",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("pub_rear_wheel_yaw", "/aiformula_sensing/rear_potentiometer/yaw"),
            ("sub_can", "/aiformula_sensing/vehicle_info"),
        ],
    )

    return LaunchDescription(
        [
            rear_potentiometer,
        ]
    )
