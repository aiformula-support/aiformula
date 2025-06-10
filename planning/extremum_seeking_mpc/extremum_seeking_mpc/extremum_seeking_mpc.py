import numpy as np
from typing import Tuple

from geometry_msgs.msg import Twist, TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from common_python.get_ros_parameter import get_ros_parameter
from .object_risk_calculator import ObjectRiskCalculator
from .path_optimizer import PathOptimizer
from .pose_predictor import PosePredictor
from .road_risk_calculator import RoadRiskCalculator
from .util import Side, Vector2


class ExtremumSeekingMpc(Node):

    def __init__(self):
        super().__init__('extremum_seeking_mpc')
        # initialize
        self.init_parameters()
        self.init_members()
        self.init_connections()

        self.get_logger().info(f'ego_target_velocity : {self.ego_target_velocity}')

        # initialize tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Extremum Seeking MPC parameter
        self.curvatures = np.zeros(len(self.predict_horizon))

        self.timer = self.create_timer(self.control_period, self.publish_cmd_vel_timer_callback)

    def init_parameters(self):
        self.ego_target_velocity = get_ros_parameter(
            self, "planned_speed")
        self.predict_horizon = get_ros_parameter(
            self, "horizon_times")
        self.curvature_gain = get_ros_parameter(
            self, "curvature_gain")
        self.benefit_gain = get_ros_parameter(
            self, "benefit_gain")
        self.deceleration_angle_maximum = get_ros_parameter(
            self, "velocity_control.deceleration_angle_maximum")
        self.deceleration_gain = get_ros_parameter(
            self, "velocity_control.deceleration_gain")
        self.base_footprint_frame_id = get_ros_parameter(
            self, "base_footprint_frame_id")
        self.control_period = get_ros_parameter(
            self, "control_period")
        self.buffer_size = get_ros_parameter(
            self, "buffer_size")

    def init_members(self):
        self.object_risk_calculator = ObjectRiskCalculator(self, self.buffer_size)
        self.road_risk_calculator = RoadRiskCalculator(self, self.buffer_size)
        self.path_optimizer = PathOptimizer(self, self.control_period)
        init_seek_points_array = np.array([
            controller.seek_points for controller in self.path_optimizer.extremum_seeking_controllers
        ])
        self.pose_predictor = PosePredictor(self, self.predict_horizon, init_seek_points_array, self.buffer_size)

    def init_connections(self):
        self.twist_pub = self.create_publisher(Twist, 'pub_twist_command', self.buffer_size)

    def predict_ego_position(self, curvatures: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        ego_positions = self.pose_predictor.predict_relative_ego_positions(curvatures)
        seek_position_relative = self.pose_predictor.predict_relative_seek_positions(ego_positions)
        seek_positions = self.pose_predictor.predict_absolute_seek_positions(ego_positions, seek_position_relative)

        return ego_positions, seek_positions

    def calculate_object_risk(self, seek_positions: np.ndarray) -> np.ndarray:
        object_risk = self.object_risk_calculator.compute_object_risk(seek_positions)
        return object_risk

    def calculate_road_risk(self, seek_positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        left_road_risk, left_y_hat = self.road_risk_calculator.compute_road_risk(seek_positions, Side.LEFT)
        right_road_risk, right_y_hat = self.road_risk_calculator.compute_road_risk(seek_positions, Side.RIGHT)
        benefit = self.road_risk_calculator.get_benefit_value(seek_positions, left_y_hat, right_y_hat)
        return left_road_risk, right_road_risk, benefit

    def calculate_total_risk(self, object_risk: np.ndarray, left_road_risk: np.ndarray, right_road_risk: np.ndarray, benefit: np.ndarray) -> np.ndarray:
        total_risk = object_risk + left_road_risk + right_road_risk - self.benefit_gain * benefit

        return total_risk

    def calculate_yaw_rate(self, total_risk: np.ndarray) -> Tuple[float, float, np.ndarray]:
        curvatures = self.path_optimizer.apply_extremum_seeking_control(total_risk)
        predicted_pose = self.pose_predictor.predict_pose(self.curvatures[0], self.predict_horizon[0])
        yaw_angle = predicted_pose.yaw
        yaw_rate = yaw_angle / self.predict_horizon[0]

        return yaw_angle, yaw_rate, curvatures

    def calculate_linear_velocity(self, yaw_angle: float) -> float:
        target_linear_velocity = self.ego_target_velocity[0]
        # If the yaw angle exceeds the threshold value,
        # the speed command value is lowered according to the excess.
        excess_yaw_angle_for_deceleration = abs(yaw_angle) - self.deceleration_angle_maximum
        if excess_yaw_angle_for_deceleration <= 0.:
            return target_linear_velocity
        else:
            target_linear_velocity -= excess_yaw_angle_for_deceleration * self.deceleration_gain
            return max(target_linear_velocity, 0.)

    def publish_cmd_vel(self, vehicle_linear_velocity: float, yaw_rate: float) -> None:
        twist_msg = Twist()
        twist_msg.linear.x = vehicle_linear_velocity
        twist_msg.angular.z = yaw_rate * self.curvature_gain
        self.twist_pub.publish(twist_msg)

    def tf_viewer(self, ego_positions: np.ndarray) -> None:
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        parent_frame_ids = ("base_footprint", "seek_point_0", "seek_point_1")
        child_frame_ids = ("seek_point_0", "seek_point_1", "seek_point_2")
        for position, parent_frame_id, child_frame_id in zip(ego_positions, parent_frame_ids, child_frame_ids):
            ts.header.frame_id = parent_frame_id
            ts.child_frame_id = child_frame_id
            ts.transform.translation.x = position[Vector2.X]
            ts.transform.translation.y = position[Vector2.Y]
            ts.transform.translation.z = 0.
            ts.transform.rotation.x = 0.
            ts.transform.rotation.y = 0.
            ts.transform.rotation.z = 0.
            ts.transform.rotation.w = 1.
            self.tf_broadcaster.sendTransform(ts)

    def publish_cmd_vel_timer_callback(self):
        # ---- start extremum_seeking_mpc sequence ----
        ego_positions, seek_positions = self.predict_ego_position(self.curvatures)

        object_risk = self.calculate_object_risk(seek_positions)

        left_road_risk, right_road_risk, benefit = self.calculate_road_risk(seek_positions)

        total_risk = self.calculate_total_risk(object_risk, left_road_risk, right_road_risk, benefit)

        yaw_angle, yaw_rate, self.curvatures = self.calculate_yaw_rate(total_risk)

        vehicle_linear_velocity = self.calculate_linear_velocity(yaw_angle)

        self.publish_cmd_vel(vehicle_linear_velocity, yaw_rate)

        self.tf_viewer(ego_positions)

    def stop_vehicle(self):
        twist_stop_msg = Twist()
        twist_stop_msg.linear.x = 0.0
        twist_stop_msg.angular.z = 0.0
        self.twist_pub.publish(twist_stop_msg)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    extremum_seeking_mpc = ExtremumSeekingMpc()
    try:
        rclpy.spin(extremum_seeking_mpc)
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt (Ctrl+C), shutting down...")
        extremum_seeking_mpc.stop_vehicle()
    finally:
        extremum_seeking_mpc.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
