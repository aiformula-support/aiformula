import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import torch
import torchvision.transforms as transforms

from pathlib import Path
from ament_index_python.packages import get_package_prefix
package = str(Path(__file__).resolve().parent.name)
workspace = Path(get_package_prefix(package)).parents[1]
yolop_module_path = workspace/'..'/'YOLOP'
if str(yolop_module_path) not in sys.path:
    sys.path.insert(0, str(yolop_module_path))  # add ROOT to PATH

from lib.config import cfg
from lib.utils import letterbox_for_img
from lib.utils.utils import select_device
from lib.models import get_net
from lib.utils import show_seg_result
class RoadDetector(Node):

    def __init__(self):
        super().__init__('road_detector')
        buffer_size = 10
        self.cv_bridge = CvBridge()
        architecture, weights, mean, std = self.get_params()
        self.architecture = select_device(device=architecture)
        self.use_half_precision = (self.architecture.type != 'cpu')  # half precision only supported on CUDA
        # Load model
        self.model = get_net(cfg)
        checkpoint = torch.load(weights, map_location=self.architecture)
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model = self.model.to(self.architecture)
        if self.use_half_precision:
            self.model.half()  # to FP16
        # Nomalization and Tensor
        normalize = transforms.Normalize(mean, std)
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            normalize,
        ])

        self.annotated_mask_image_pub = self.create_publisher(Image, 'pub_annotated_mask_image', buffer_size)
        self.lane_mask_image_pub = self.create_publisher(Image, 'pub_mask_image', buffer_size)
        self.image_sub = self.create_subscription(
            Image, 'sub_image', self.image_callback, buffer_size)

    def get_params(self):
        self.declare_parameter('use_architecture')
        self.declare_parameter('weight_path')
        self.declare_parameter('mean')
        self.declare_parameter('standard_deviation')
        architecture = self.get_parameter('use_architecture').get_parameter_value().string_value
        weights = self.get_parameter('weight_path').get_parameter_value().string_value
        mean = np.array(self.get_parameter('mean').get_parameter_value().double_array_value)
        std = np.array(self.get_parameter('standard_deviation').get_parameter_value().double_array_value)
        return architecture, weights, mean, std

    def padding_image(self, image):
        # Padded resize
        padding_image, (ratio_to_padding, _), (dw, dh) = letterbox_for_img(
            image, new_shape=640, auto=True)    # ratio_to_padding (width, height)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        return np.ascontiguousarray(padding_image), ratio_to_padding, top, bottom, left, right

    def image_callback(self, msg):
        try:
            undistorted_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        padding_image, ratio_to_padding, top, bottom, left, right = self.padding_image(undistorted_image)
        normalize_image = self.transform(padding_image).to(self.architecture)
        input_image = normalize_image.half() if self.use_half_precision else normalize_image.float()  # uint8 to fp16/32
        if input_image.ndimension() == 3:
            input_image = input_image.unsqueeze(0)
        # Inference
        _, _, ll_seg_out = self.model(input_image)  # ll_seg_out -> lane line segmentation output
        _, _, height, width = input_image.shape
        ll_predict = ll_seg_out[:, :, top:(height-bottom), left:(width-right)]
        ll_seg_mask_raw = torch.nn.functional.interpolate(
            ll_predict, scale_factor=int(1/ratio_to_padding), mode='bilinear')
        _, ll_seg_poits = torch.max(ll_seg_mask_raw, 1)
        ll_seg_mask = ll_seg_poits.int().squeeze().cpu().numpy()
        self.publish_result(undistorted_image, ll_seg_mask, msg.header.stamp)

    def publish_result(self, image, ll_seg_mask, time_stamp):
        # Publish annotated image
        annotated_image = show_seg_result(image, (ll_seg_mask, None), None, None, is_demo=True)
        self.annotated_mask_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(annotated_image, "bgr8"))

        # Publish mask image
        ll_seg_mask = np.array(ll_seg_mask, dtype='uint8')
        ll_seg_mask_msg = self.cv_bridge.cv2_to_imgmsg(ll_seg_mask, "mono8")
        ll_seg_mask_msg.header.stamp = time_stamp
        self.lane_mask_image_pub.publish(ll_seg_mask_msg)


def main():
    with torch.no_grad():
        rclpy.init()
        road_detector = RoadDetector()
        rclpy.spin(road_detector)
        road_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
