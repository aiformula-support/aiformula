import numpy as np
from typing import List

from nav_msgs.msg import Odometry
from rclpy.node import Node

from common_python.get_ros_parameter import get_ros_parameter
from .util import Position2d, Pose, Velocity


class PosePredictor:

    def __init__(self, node: Node, horizon_times: List[float], seek_y_positions: np.ndarray, buffer_size: int):
        self.init_parameters(node)
        self.init_connections(node, buffer_size)
        self.ego_current_velocity = None
        self.horizon_durations = np.diff(horizon_times, prepend=0)
        self.horizon_length = len(horizon_times)
        self.seek_y_positions = seek_y_positions

    def init_parameters(self, node: Node):
        self.curvature_radius_maximum = get_ros_parameter(
            node, "curvature_radius_maximum")

    def init_connections(self, node: Node, buffer_size: int):
        self.actucal_speed_sub = node.create_subscription(
            Odometry, 'sub_odom', self.odometry_callback, buffer_size)

    def odometry_callback(self, odom_msg: Odometry) -> None:
        linear_velocity = np.linalg.norm([odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y])
        self.ego_current_velocity = Velocity(linear=linear_velocity,
                                             angular=odom_msg.twist.twist.angular.z)

    def predict_relative_ego_positions(self, curvatures: np.ndarray) -> np.ndarray:
        # ego_v: lin_x, ang.z, curvature(curvature): 3 horizon
        # Assume omega is 1/curvature instead ang.z

        # Initialize
        predicted_positions = [Pose() for _ in range(self.horizon_length)]
        # predicted_positions = []
        predicted_positions_rotate_transformed = np.zeros((self.horizon_length - 1, 2))

        # Update predict position
        for curvature, horizon_duration, predicted_position in zip(curvatures, self.horizon_durations, predicted_positions):
            # predicted_position.pos, predicted_position.yaw = self.predict_pose(curvature, horizon_duration)
            predicted_pose = self.predict_pose(curvature, horizon_duration)
            predicted_position.pos, predicted_position.yaw = predicted_pose.pos, predicted_pose.yaw

            # Rotate Point
        for horizon_idx in range(self.horizon_length - 1):
            # calculate yaw
            rotation_angle = predicted_positions[horizon_idx].yaw
            if horizon_idx:
                rotation_angle += predicted_positions[horizon_idx+1].yaw

            # apply yaw angle to position
            predicted_positions_rotate_transformed[horizon_idx] = self.rotate_position(
                predicted_positions[horizon_idx+1].pos, rotation_angle)

        # Update yaw angle
        for horizon_idx in range(1, self.horizon_length):
            predicted_positions[horizon_idx].yaw = predicted_positions[horizon_idx - 1].yaw

        first_position_array = np.array([predicted_positions[0].pos.x, predicted_positions[0].pos.y])[np.newaxis, :]
        ego_positions = np.vstack([first_position_array, predicted_positions_rotate_transformed])

        return ego_positions

    @staticmethod
    def create_rotation_matrix(angle: float) -> np.ndarray:
        rotation_matrix = np.array(
            [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        return rotation_matrix

    @staticmethod
    def rotate_position(position: Position2d, angle: float) -> np.ndarray:
        rotation_matrix = PosePredictor.create_rotation_matrix(angle)
        position_transformed = rotation_matrix @ position.as_array()
        return position_transformed

    def predict_pose(self, curvature: float, horizon_time: float) -> Pose:
        max_curvature = 1. / self.curvature_radius_maximum
        radius = 1. / curvature if abs(curvature) > max_curvature else self.curvature_radius_maximum

        travel_distance = self.ego_current_velocity.linear * horizon_time
        arc_angle = float(travel_distance / radius)
        if radius < self.curvature_radius_maximum:  # curve
            x = radius * np.sin(arc_angle)
            y = radius * (1. - np.cos(arc_angle))
        else:  # straight
            x = travel_distance
            y = 0.
            arc_angle = 0.
        return Pose(pos=Position2d(x, y), yaw=arc_angle)

    def predict_relative_seek_positions(self, ego_positions: np.ndarray) -> np.ndarray:
        relative_seek_positions = np.empty(3, dtype=object)
        for horizon_idx in range(self.horizon_length):
            relative_seek_positions[horizon_idx] = (
                ego_positions[horizon_idx] + np.array([np.zeros(len(self.seek_y_positions[0])), np.array(self.seek_y_positions[horizon_idx])]).T).T

        return relative_seek_positions

    def predict_absolute_seek_positions(self, ego_positions: np.ndarray, relative_seek_positions: np.ndarray) -> np.ndarray:
        seek_points_shape = relative_seek_positions[0].shape
        absolute_seek_positions = np.zeros((self.horizon_length,) + seek_points_shape)

        prev_position = ego_positions[0]

        for idx in range(self.horizon_length - 1):
            absolute_seek_positions[idx + 1] = np.array(
                [prev_position + ego_positions[idx]]).T + relative_seek_positions[idx + 1]
            prev_position = ego_positions[idx]

        absolute_seek_positions[0] = relative_seek_positions[0]

        return absolute_seek_positions
