import numpy as np
from scipy.stats import multivariate_normal
import threading
from typing import List

from rclpy.node import Node

from aiformula_msgs.msg import ObjectInfoMultiArray
from common_python.get_ros_parameter import get_ros_parameter


class ObjectRiskCalculator:
    def __init__(self, node: Node, buffer_size: int):
        self.lock = threading.Lock()
        self.init_parameters(node)
        self.init_connections(node, buffer_size)
        self.object_infos = []

    def init_parameters(self, node: Node):
        self.object_risk_variance_x = get_ros_parameter(
            node, "object_risk_potential.variance_x")
        self.object_risk_variance_y = get_ros_parameter(
            node, "object_risk_potential.variance_y")
        self.object_risk_correlation_coefficient = get_ros_parameter(
            node, "object_risk_potential.correlation_coefficient")
        self.object_risk_gain = get_ros_parameter(
            node, "object_risk_potential.gain")

    def init_connections(self, node: Node, buffer_size):
        self.object_position_sub = node.create_subscription(
            ObjectInfoMultiArray, 'sub_object_info', self.object_info_callback, buffer_size)

    def object_info_callback(self, msg: ObjectInfoMultiArray) -> None:
        with self.lock:
            self.object_infos = msg.objects

    def compute_object_risk(self, seek_positions: np.ndarray) -> np.ndarray:
        with self.lock:
            object_infos = self.object_infos

        num_seek_points = seek_positions[0].shape[1]
        num_horizon_steps = len(seek_positions)

        seek_points_risk = []

        if not object_infos:
            return np.zeros((num_horizon_steps, num_seek_points))
        else:
            for seek_position in seek_positions:
                seek_point_risk = self.get_object_risk_value(object_infos, seek_position)
                seek_points_risk.append(seek_point_risk)

            return np.array(seek_points_risk)

    def get_object_risk_value(self, objects: List[float], seek_positions: np.ndarray) -> np.ndarray:
        num_seek_points = seek_positions.shape[1]
        var_x = self.object_risk_variance_x**2
        var_y = self.object_risk_variance_y**2
        cov_xy = self.object_risk_correlation_coefficient*self.object_risk_variance_x*self.object_risk_variance_y
        gaussian_cov = np.array([[var_x, cov_xy],
                                 [cov_xy, var_y]])

        obj_xy = np.array([[obj.x, obj.y] for obj in objects])
        obj_width = np.array([obj.width for obj in objects])
        obj_conf = np.array([obj.confidence for obj in objects])
        obj_weight = self.object_risk_gain * obj_width * (obj_conf ** 2)

        risks = []

        for seek_points_idx in range(num_seek_points):
            gaussian_mean = seek_positions[:, seek_points_idx]
            risk_values = multivariate_normal.pdf(obj_xy, mean=gaussian_mean, cov=gaussian_cov)
            total_risk = np.sum(risk_values * obj_weight)
            risks.append(total_risk)

        return np.array(risks)
