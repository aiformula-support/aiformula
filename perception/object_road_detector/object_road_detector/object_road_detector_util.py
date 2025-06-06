from typing import Tuple
import cv2
import numpy as np
import torch

from yolop.lib.core.general import scale_coords

from aiformula_interfaces.msg import Rect


def to_rect(object_pose: torch.Tensor) -> Rect:
    rect = Rect()
    x_min, y_min, x_max, y_max = object_pose
    rect.x = float(x_min)
    rect.y = float(y_min)
    rect.height = float(abs(y_min - y_max))
    rect.width = float(abs(x_min - x_max))
    return rect


def draw_lane_lines(image: np.ndarray, ll_seg_mask: np.ndarray, alpha: float = 0.5, color: Tuple[int, int, int] = (0, 255, 0)) -> None:
    overlay = np.zeros_like(image, dtype=np.uint8)
    LANE_LINE_VALUE = 1
    mask = ll_seg_mask == LANE_LINE_VALUE
    overlay[mask] = color
    mask = ll_seg_mask.astype(bool)
    image[mask] = cv2.addWeighted(image[mask], 1.0 - alpha, overlay[mask], alpha, gamma=0.0)


def draw_bounding_boxes(image: np.ndarray, objects: torch.Tensor, input_image_shape: torch.Size) -> None:   # draw bbox with opencv
    bboxes_coords = scale_coords(input_image_shape, objects[:, :4], image.shape).round()
    for bboxes_coord in bboxes_coords:
        plot_one_box(image, bboxes_coord)


# Plots one bounding box on image
def plot_one_box(image: np.ndarray, bbox_coords: torch.Tensor, color: Tuple[int, int, int] = (255, 0, 0)) -> None:
    top_left = (int(bbox_coords[0]), int(bbox_coords[1]))
    bottom_right = (int(bbox_coords[2]), int(bbox_coords[3]))
    thickness = max(int(image.shape[0] * 0.002), 1)
    cv2.rectangle(image, top_left, bottom_right, color, thickness, lineType=cv2.LINE_AA)
