import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    PACKAGE_NAME = "extremum_seeking_mpc"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)

    ROS_PARAM_CONFIG = (
        osp.join(PACKAGE_DIR, "config", "extremum_seeking_mpc_params.yaml"),
        osp.join(PACKAGE_DIR, "config", "extremum_seeking_controller_params.yaml"),
        osp.join(PACKAGE_DIR, "config", "object_risk_calculator_params.yaml"),
        osp.join(PACKAGE_DIR, "config", "road_risk_calculator_params.yaml"),
        osp.join(PACKAGE_DIR, "config", "pose_predictor_params.yaml"),
        osp.join(PACKAGE_DIR, "config", "back_propagation_params.yaml"),
    )

    extremum_seeking_mpc = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_planning",
        output="screen",
        emulate_tty=True,
        parameters=[
            *ROS_PARAM_CONFIG,
            {
                "base_footprint_frame_id": "base_footprint",
            },
        ],
        remappings=[
            ("pub_twist_command", "/aiformula_control/extremum_seeking_mpc/cmd_vel"),
            ("sub_road_l", "/aiformula_perception/lane_line_publisher/lane_lines/left"),
            ("sub_road_r", "/aiformula_perception/lane_line_publisher/lane_lines/right"),
            ("sub_odom", "/aiformula_sensing/gyro_odometry_publisher/odom"),
            ("sub_object_info", "/aiformula_perception/object_publisher/object_info"),
        ],
    )

    return LaunchDescription(
        [
            extremum_seeking_mpc,
        ]
    )
