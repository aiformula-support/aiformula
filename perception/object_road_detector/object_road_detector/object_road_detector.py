from copy import deepcopy
from typing import Tuple

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import torch
import torchvision.transforms as transforms

import rclpy
from rclpy.node import Node
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


class ObjectRoadDetector(Node):

    def __init__(self):
        super().__init__('object_road_detector')
        device, path_to_weights, mean, stdev = self.get_params()
        self.init_detector(device, path_to_weights, mean, stdev)
        self.cv_bridge = CvBridge()
        buffer_size = 10
        self.image_sub = self.create_subscription(
            Image, 'sub_image', self.image_callback, buffer_size)
        self.annotated_image_pub = self.create_publisher(Image, 'pub_annotated_image', buffer_size)
        self.lane_mask_image_pub = self.create_publisher(Image, 'pub_mask_image', buffer_size)
        self.rects_pub = self.create_publisher(RectMultiArray, 'pub_bbox', buffer_size)

    def get_params(self) -> Tuple[str, str, float, float]:
        device = str(get_ros_parameter(self, 'use_device'))
        path_to_weights = get_ros_parameter(self, 'weight_path')
        mean = get_ros_parameter(self, 'normalization.mean')
        stdev = get_ros_parameter(self, 'normalization.standard_deviation')
        self.confidence_threshold = get_ros_parameter(self, 'confidence_threshold')
        self.iou_threshold = get_ros_parameter(self, 'iou_threshold')
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
        try:
            undistorted_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().warning(f"CvBridgeError occurred: {str(e)}")
        # Padded resize
        padded_image, (ratio_to_padded, _), (pad_x_half, pad_y_half) = letterbox_for_img(undistorted_image,
                                                                                         new_shape=640, auto=True)  # ratio_to_padded (width, height)
        normalized_tensor = self.transform(padded_image).to(self.use_device)
        # Input image
        input_image = normalized_tensor.half() if self.use_half_precision else normalized_tensor.float()  # uint8 to fp16/32
        input_image = input_image.unsqueeze(0)
        input_image_size = input_image.shape[2:]
        # Inference
        with torch.no_grad():
            object_raw_outputs, _, ll_seg_raw_outputs = self.detector(input_image)  # ll_seg : lane line segmentation
        ll_seg_mask = self.decode_lane_line_output(
            ll_seg_raw_outputs, *input_image_size, ratio_to_padded, pad_x_half, pad_y_half)
        bbox_detections = self.decode_object_output(object_raw_outputs)
        # Publish
        self.publish_lane_line(ll_seg_mask, msg.header)
        self.publish_rects(input_image_size, undistorted_image.shape, deepcopy(bbox_detections), msg.header)
        self.publish_result_image(undistorted_image, input_image_size, ll_seg_mask,
                                  bbox_detections, msg.header)

    def decode_lane_line_output(self, ll_seg_raw: torch.Tensor, height: int, width: int, ratio_to_padded: float, pad_x_half: np.float64, pad_y_half: np.float64) -> np.ndarray:
        ROUNDING_ADJUSTMENT = 0.1
        top, bottom = round(pad_y_half - ROUNDING_ADJUSTMENT), round(pad_y_half + ROUNDING_ADJUSTMENT)
        left, right = round(pad_x_half - ROUNDING_ADJUSTMENT), round(pad_x_half + ROUNDING_ADJUSTMENT)
        ll_predict = ll_seg_raw[:, :, top:(height-bottom), left:(width-right)]
        ll_seg_mask_raw = torch.nn.functional.interpolate(
            ll_predict, scale_factor=int(1.0/ratio_to_padded), mode='bilinear')
        _, ll_seg_map = torch.max(ll_seg_mask_raw, dim=1)
        ll_seg_mask = np.array(ll_seg_map.int().squeeze().cpu().numpy(), dtype=np.uint8)
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

    def publish_rects(self, input_image_shape: torch.Size, undistorted_image_shape: Tuple[int, int, int], bbox_detections: torch.Tensor, header: Header) -> None:
        # Extract the bounding box coordinates: top-left, bottom-right
        bbox_coords = scale_coords(input_image_shape, bbox_detections[:, :4], undistorted_image_shape).round()
        bbox_msg = RectMultiArray()
        for bbox_coord in bbox_coords:
            bbox_msg.rects.append(to_rect(bbox_coord))
        bbox_msg.header = header
        self.rects_pub.publish(bbox_msg)

    def publish_result_image(self, undistorted_image: np.ndarray, input_image_shape: torch.Size, ll_seg_mask: np.ndarray, bbox_detections: torch.Tensor, header: Header) -> None:
        draw_lane_lines(undistorted_image, ll_seg_mask)
        draw_bounding_boxes(undistorted_image, bbox_detections, input_image_shape)
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
