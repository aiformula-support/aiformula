import os.path as osp
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    VEHICLE_NAME = "ai_car1"
    CAMERA_NAME = "zedx"
    CAMERA_SN = "SN48442725"
    CAMERA_RESOLUTION = "SVGA"

    tf_static_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("vehicle"),
                     "launch/extrinsic_tfstatic_broadcaster.launch.py"),
        ),
        launch_arguments={
            "vehicle_name": VEHICLE_NAME,
            "use_sim_time": "false",
        }.items(),
    )
    zed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("launchers"),
                     "launch/zedx_camera.launch.py"),
        ),
    )
    # --- IMU, GNSS --- #
    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("launchers"),
                     "launch/vectornav.launch.py"),
        ),
    )
    # --- Perception --- #
    lane_line_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("lane_line_publisher"), "launch/lane_line_publisher.launch.py"),
        ),
        launch_arguments={
            "camera_name": CAMERA_NAME,
            "camera_sn": CAMERA_SN,
            "camera_resolution": CAMERA_RESOLUTION,
        }.items(),
    )
    # --- GamePad --- #
    joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("launchers"),
                     "launch/joy.launch.py"),
        ),
    )
    # --- Output velocity and angular velocity --- #
    teleop_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("launchers"),
                     "launch/teleop.launch.py"),
        ),
    )
    motor_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("motor_controller"),
                     "launch/motor_controller.launch.py"),
        ),
    )
    can_receiver_and_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("launchers"),
                     "launch/socket_can_bridge.launch.py"),
        ),
    )
    gyro_odometry_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("odometry_publisher"),
                     "launch/gyro_odometry_publisher.launch.py"),
        ),
        launch_arguments={
            "use_rviz": "true",
        }.items(),
    )

    rear_potentiometer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("rear_potentiometer"),
                     "launch/rear_potentiometer.launch.py"),
        ),
    )

    return LaunchDescription([
        tf_static_publisher,
        zed_node,
        vectornav,
        lane_line_publisher,
        joy_node,
        teleop_node,
        motor_controller,
        can_receiver_and_sender,
        gyro_odometry_publisher,
        rear_potentiometer,
    ])
