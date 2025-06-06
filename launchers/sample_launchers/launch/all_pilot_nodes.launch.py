import os.path as osp
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    compressed_image_viewer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("compressed_image_viewer"),
                     "launch/compressed_image_viewer.launch.py"),
        ),
    )
    # --- Command input from handle controller --- #
    handle_controller_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("sample_launchers"),
                     "launch/handle_controller_joy.launch.py"),
        ),
    )
    # --- Output velocity and angular velocity from handle controller --- #
    handle_controller_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("teleop_twist_handle_controller"),
                     "launch/handle_controller_teleop.launch.py"),
        ),
    )

    return LaunchDescription([
        compressed_image_viewer,
        handle_controller_joy,
        handle_controller_teleop,
    ])
