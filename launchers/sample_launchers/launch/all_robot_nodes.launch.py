import os.path as osp
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    VEHICLE_NAME = "ai_car1"
    CAMERA_NAME = "zedx"
    CAMERA_SN = "SN48311510"
    CAMERA_RESOLUTION = "nHD"

    launch_args = (
        DeclareLaunchArgument(
            "autopilot",
            default_value="false",
            description="If true, robot runs autonomously",
        )
    )

    vehicle_tf_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("sample_vehicle"),
                     "launch/vehicle_tf_broadcaster.launch.py"),
        ),
        launch_arguments={
            "vehicle_name": VEHICLE_NAME,
            "use_sim_time": "false",
        }.items(),
    )
    zed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("sample_launchers"),
                     "launch/zedx_camera.launch.py"),
        ),
    )
    # --- IMU, GNSS --- #
    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("sample_launchers"),
                     "launch/vectornav.launch.py"),
        ),
    )
    # --- Perception --- #
    object_road_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("object_road_detector"),
                     "launch/object_road_detector.launch.py"),
        ),
    )
    lane_line_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("lane_line_publisher"),
                     "launch/lane_line_publisher.launch.py"),
        ),
        launch_arguments={
            "camera_name": CAMERA_NAME,
            "camera_sn": CAMERA_SN,
            "camera_resolution": CAMERA_RESOLUTION,
        }.items(),
    )
    # --- Command input from gamepad --- #
    gamepad_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("sample_launchers"),
                     "launch/gamepad_joy.launch.py"),
        ),
    )
    # --- Output velocity and angular velocity from gamepad --- #
    gamepad_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("sample_launchers"),
                     "launch/gamepad_teleop.launch.py"),
        ),
    )
    # --- Multiplex velocities --- #
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("sample_launchers"),
                     "launch/twist_mux.launch.py"),
        ),
        launch_arguments={
            "use_rviz": "false",
            "use_runtime_monitor": "false",
        }.items(),
    )
    motor_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("motor_controller"),
                     "launch/motor_controller.launch.py"),
        ),
    )
    can_receiver_and_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("sample_launchers"),
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
    extremum_seeking_mpc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("extremum_seeking_mpc"),
                     "launch/extremum_seeking_mpc.launch.py"),
        ),
        condition=IfCondition([LaunchConfiguration("autopilot")]),
    )
    # --- Compress Zed Image for AI Formula Pilot --- #
    image_compressor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("image_compressor"),
                     "launch/image_compressor.launch.py"),
        ),
    )
    object_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(get_package_share_directory("object_publisher"),
                     "launch/object_publisher.launch.py"),
        ),
        launch_arguments={
            "camera_name": CAMERA_NAME,
            "camera_sn": CAMERA_SN,
            "camera_resolution": CAMERA_RESOLUTION,
            "use_rviz": "false",
            "debug": "false",
        }.items(),
    )

    return LaunchDescription([
        launch_args,
        vehicle_tf_broadcaster,
        zed_node,
        vectornav,
        object_road_detector,
        lane_line_publisher,
        gamepad_joy,
        gamepad_teleop,
        twist_mux,
        motor_controller,
        can_receiver_and_sender,
        gyro_odometry_publisher,
        rear_potentiometer,
        image_compressor,
        extremum_seeking_mpc,
        object_publisher,
    ])
