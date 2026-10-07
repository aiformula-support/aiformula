#!/usr/bin/env python3
"""Save a handful of frames from an image topic to disk, to feed the offline comparison tools.

    ros2 run ... # not installed as an entry point, run directly:
    python3 capture_frames.py --topic /aiformula_sensing/zed_node/left_image/undistorted \
        --count 20 --output-dir /tmp/zedx_frames
"""

import argparse
import os
import os.path as osp

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class FrameCapture(Node):

    def __init__(self, topic: str, count: int, every_nth: int, output_dir: str) -> None:
        super().__init__("frame_capture")
        self.cv_bridge = CvBridge()
        self.count = count
        self.every_nth = every_nth
        self.output_dir = output_dir
        self.received = 0
        self.saved = 0
        os.makedirs(output_dir, exist_ok=True)
        self.subscription = self.create_subscription(
            Image, topic, self.image_callback,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info(f"Waiting for {count} frames on '{topic}'")

    def image_callback(self, msg: Image) -> None:
        self.received += 1
        if self.received % self.every_nth != 0:
            return
        image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        path = osp.join(self.output_dir, f"frame_{self.saved:04d}.png")
        cv2.imwrite(path, image)
        self.saved += 1
        self.get_logger().info(f"[{self.saved}/{self.count}] {path} ({image.shape[1]}x{image.shape[0]})")
        if self.saved >= self.count:
            raise SystemExit(0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/aiformula_sensing/zed_node/left_image/undistorted")
    parser.add_argument("--count", type=int, default=20, help="number of frames to save")
    parser.add_argument("--every-nth", type=int, default=5, help="save every Nth received frame")
    parser.add_argument("--output-dir", default="/tmp/zedx_frames")
    args = parser.parse_args()

    rclpy.init()
    node = FrameCapture(args.topic, args.count, args.every_nth, args.output_dir)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
