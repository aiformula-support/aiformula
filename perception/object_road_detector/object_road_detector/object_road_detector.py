from typing import Tuple

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import torch
import torchvision.transforms as transforms

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from yolop.lib.config import cfg
from yolop.lib.core.general import non_max_suppression, scale_coords
from yolop.lib.models import get_net
from yolop.lib.utils import letterbox_for_img
from yolop.lib.utils.utils import select_device

from aiformula_msgs.msg import RectMultiArray
from common_python.get_ros_parameter import get_ros_parameter
from .object_road_detector_util import to_rect, draw_lane_lines, draw_bounding_boxes
from .stage_profiler import StageProfiler


class ObjectRoadDetector(Node):

    def __init__(self):
        super().__init__('object_road_detector')
        device, path_to_weights, mean, stdev = self.get_params()
        self.init_detector(device, path_to_weights, mean, stdev)
        self.cv_bridge = CvBridge()
        self.profiler = StageProfiler(
            self, enabled=self.enable_profiling, report_interval_sec=self.profiling_interval,
            cuda_sync=self.profiling_cuda_sync)
        buffer_size = 10
        # Only the newest frame is worth inferring on: a reliable, deep queue lets the middleware
        # hand over frames faster than they can be processed and the output lags behind reality.
        self.image_sub = self.create_subscription(
            Image, 'sub_image', self.image_callback,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.lane_mask_image_pub = self.create_publisher(Image, 'pub_mask_image', buffer_size)
        self.rects_pub = self.create_publisher(RectMultiArray, 'pub_bbox', buffer_size)
        self.annotated_image_pub = self.create_publisher(
            Image, 'pub_annotated_image', buffer_size) if self.debug else None

    def get_params(self) -> Tuple[str, str, float, float]:
        device = str(get_ros_parameter(self, 'use_device'))
        path_to_weights = get_ros_parameter(self, 'weight_path')
        mean = get_ros_parameter(self, 'normalization.mean')
        stdev = get_ros_parameter(self, 'normalization.standard_deviation')
        self.confidence_threshold = get_ros_parameter(self, 'confidence_threshold')
        self.iou_threshold = get_ros_parameter(self, 'iou_threshold')
        self.debug = get_ros_parameter(self, 'debug')
        self.enable_profiling = get_ros_parameter(self, 'profiling.enable')
        self.profiling_interval = get_ros_parameter(self, 'profiling.report_interval_sec')
        self.profiling_cuda_sync = get_ros_parameter(self, 'profiling.cuda_sync')
        return device, path_to_weights, mean, stdev

    def init_detector(self, device: str, path_to_weights: str, mean: float, stdev: float) -> None:
        self.load_detector(device, path_to_weights)
        # Normalization and Tensor
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean, stdev),
        ])

    def load_detector(self, device: str, path_to_weights: str) -> None:
        self.use_device = select_device(device=device)
        self.use_half_precision = (self.use_device.type != 'cpu')  # half precision only supported on CUDA
        self.detector = get_net(cfg)
        checkpoint = torch.load(path_to_weights, map_location=self.use_device)
        self.detector.load_state_dict(checkpoint['state_dict'])
        if self.use_device.type == 'cuda':
            self.detector = self.detector.to(self.use_device)
            self.detector.half()  # to FP16
        self.detector.eval()

    def image_callback(self, msg: Image) -> None:
        with self.profiler.stage('cv_bridge'):
            try:
                undistorted_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            except CvBridgeError as e:
                self.get_logger().warning(f"CvBridgeError occurred: {str(e)}")
                return
        with self.profiler.stage('letterbox'):
            # Padded resize
            padded_image, _, (pad_x_half, pad_y_half) = letterbox_for_img(undistorted_image,
                                                                          new_shape=640, auto=True)
        with self.profiler.stage('normalize_h2d'):
            normalized_tensor = self.transform(padded_image).to(self.use_device)
            # Input image
            input_image = normalized_tensor.half() if self.use_half_precision else normalized_tensor.float()  # uint8 to fp16/32
            input_image = input_image.unsqueeze(0)
            input_image_size = input_image.shape[2:]
        with self.profiler.stage('inference'):
            with torch.no_grad():
                object_raw_outputs, _, ll_seg_raw_outputs = self.detector(input_image)  # ll_seg : lane line segmentation
        with self.profiler.stage('decode_lane_line'):
            ll_seg_mask = self.decode_lane_line_output(
                ll_seg_raw_outputs, *input_image_size, undistorted_image.shape[:2], pad_x_half, pad_y_half)
        with self.profiler.stage('nms'):
            bbox_detections = self.decode_object_output(object_raw_outputs)
        with self.profiler.stage('scale_coords'):
            # `scale_coords` rescales in place, so the boxes are mapped back to the source image once
            # and the result is shared by the message and the visualization.
            bbox_coords = scale_coords(
                input_image_size, bbox_detections[:, :4], undistorted_image.shape).round().cpu().numpy()
        with self.profiler.stage('publish'):
            self.publish_lane_line(ll_seg_mask, msg.header)
            self.publish_rects(bbox_coords, msg.header)
        with self.profiler.stage('annotate_publish'):
            self.publish_result_image(undistorted_image, ll_seg_mask, bbox_coords, msg.header)
        self.profiler.frame_done()

    def decode_lane_line_output(self, ll_seg_raw: torch.Tensor, height: int, width: int,
                                target_size: Tuple[int, int], pad_x_half: np.float64,
                                pad_y_half: np.float64) -> np.ndarray:
        ROUNDING_ADJUSTMENT = 0.1
        top, bottom = round(pad_y_half - ROUNDING_ADJUSTMENT), round(pad_y_half + ROUNDING_ADJUSTMENT)
        left, right = round(pad_x_half - ROUNDING_ADJUSTMENT), round(pad_x_half + ROUNDING_ADJUSTMENT)
        ll_predict = ll_seg_raw[:, :, top:(height-bottom), left:(width-right)]
        # Resizing to the source image size directly keeps the mask aligned with the camera
        # intrinsics for any downscale factor, including non-integer ones.
        ll_seg_mask_raw = torch.nn.functional.interpolate(ll_predict, size=target_size, mode='bilinear')
        ll_seg_map = ll_seg_mask_raw.argmax(dim=1)
        ll_seg_mask = np.array(ll_seg_map.squeeze().to(torch.uint8).cpu().numpy(), dtype=np.uint8)
        return ll_seg_mask

    def decode_object_output(self, object_raw_outputs: Tuple[torch.Tensor, list]) -> torch.Tensor:
        raw_detections, _ = object_raw_outputs
        batched_detections = non_max_suppression(
            raw_detections,
            conf_thres=self.confidence_threshold,
            iou_thres=self.iou_threshold,
            classes=None,
            agnostic=False,
        )
        FIRST_IMAGE_INDEX = 0
        return batched_detections[FIRST_IMAGE_INDEX]

    def publish_lane_line(self, ll_seg_mask: np.ndarray, header: Header) -> None:
        ll_seg_mask_msg = self.cv_bridge.cv2_to_imgmsg(ll_seg_mask, 'mono8')
        ll_seg_mask_msg.header = header
        self.lane_mask_image_pub.publish(ll_seg_mask_msg)

    def publish_rects(self, bbox_coords: np.ndarray, header: Header) -> None:
        # Bounding box coordinates in the source image: top-left, bottom-right
        bbox_msg = RectMultiArray()
        for bbox_coord in bbox_coords:
            bbox_msg.rects.append(to_rect(bbox_coord))
        bbox_msg.header = header
        self.rects_pub.publish(bbox_msg)

    def publish_result_image(self, undistorted_image: np.ndarray, ll_seg_mask: np.ndarray,
                             bbox_coords: np.ndarray, header: Header) -> None:
        if self.annotated_image_pub is None:
            return
        draw_lane_lines(undistorted_image, ll_seg_mask)
        draw_bounding_boxes(undistorted_image, bbox_coords)
        annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(undistorted_image, 'bgr8')
        annotated_image_msg.header = header
        self.annotated_image_pub.publish(annotated_image_msg)


def main(args=None):
    rclpy.init(args=None)
    object_road_detector = ObjectRoadDetector()
    try:
        rclpy.spin(object_road_detector)
    except KeyboardInterrupt:
        print('Caught KeyboardInterrupt (Ctrl+C), shutting down...')
    finally:
        object_road_detector.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
