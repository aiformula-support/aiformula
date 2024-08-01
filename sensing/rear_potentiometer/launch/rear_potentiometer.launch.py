from launch import LaunchDescription
from launch_ros.actions import Node
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = "rear_potentiometer"
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()

    rear_potentiometer = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_sensing",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("pub_rear_wheel_yaw", TOPIC_NAMES["sensing"]["rear_potentiometer"]),
            ("sub_can", TOPIC_NAMES["sensing"]["input_can_data"]),
        ],
    )

    return LaunchDescription([
        rear_potentiometer,
    ])
