#!/usr/bin/env python3
"""Compare candidate model-input strategies on recorded frames to pick one for the TensorRT node.

The shiho weights were trained at a 640 letterbox, and the detect head anchors are expressed in
pixels at that input scale (largest anchor 68x157). Feeding a larger downscaled frame keeps the
whole field of view but makes every object correspondingly larger in pixels, which walks the near
objects off the top of the anchor range. Cropping at the native sensor scale instead keeps the
anchor scale identical to training while giving distant objects more pixels.

This script quantifies that trade-off: for each strategy it reports how many objects are found in
the far and near halves of the frame, how large they are, and how long inference takes.

    python3 compare_input_scales.py --images /tmp/zedx_frames --output-dir /tmp/scale_ab
"""

import argparse
import glob
import os
import os.path as osp
import statistics
import time
from typing import Callable, List, Tuple

import cv2
import numpy as np
import torch
import torchvision.transforms as transforms

from yolop.lib.config import cfg
from yolop.lib.core.general import non_max_suppression, scale_coords
from yolop.lib.models import get_net
from yolop.lib.utils import letterbox_for_img
from yolop.lib.utils.utils import select_device

IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STDEV = [0.229, 0.224, 0.225]
STRIDE = 32
DEFAULT_STRATEGIES = "resize:640,resize:960,resize:1280,crop:640x384,band:384"


class Strategy:
    """Turns a source frame into a model input, and maps results back to source coordinates."""

    def __init__(self, name: str, prepare: Callable[[np.ndarray], Tuple[np.ndarray, Callable, Callable]]):
        self.name = name
        self.prepare = prepare


def make_resize_strategy(target: int) -> Strategy:
    def prepare(image: np.ndarray):
        letterboxed, _, _ = letterbox_for_img(image, new_shape=target, auto=True)

        def to_source_boxes(boxes: torch.Tensor) -> torch.Tensor:
            return scale_coords(letterboxed.shape[:2], boxes, image.shape)

        def to_source_mask(mask: torch.Tensor) -> np.ndarray:
            return place_resized_mask(mask, letterboxed.shape[:2], image.shape[:2])

        return letterboxed, to_source_boxes, to_source_mask

    return Strategy(f"resize:{target}", prepare)


def make_crop_strategy(name: str, crop_width: int, crop_height: int, horizon_ratio: float) -> Strategy:
    """Native-scale window, so the pixel size of an object matches what the model was trained on."""

    def prepare(image: np.ndarray):
        source_height, source_width = image.shape[:2]
        width = min(crop_width, source_width - source_width % STRIDE)
        height = min(crop_height, source_height - source_height % STRIDE)
        x0 = (source_width - width) // 2
        y0 = int(np.clip(round(horizon_ratio * source_height - height / 2), 0, source_height - height))
        window = np.ascontiguousarray(image[y0:y0 + height, x0:x0 + width])

        def to_source_boxes(boxes: torch.Tensor) -> torch.Tensor:
            boxes[:, [0, 2]] += x0
            boxes[:, [1, 3]] += y0
            return boxes

        def to_source_mask(mask: torch.Tensor) -> np.ndarray:
            return place_cropped_mask(mask, (y0, x0), image.shape[:2])

        return window, to_source_boxes, to_source_mask

    return Strategy(name, prepare)


def place_resized_mask(mask: torch.Tensor, model_shape: Tuple[int, int], source_shape: Tuple[int, int]) -> np.ndarray:
    model_height, model_width = model_shape
    source_height, source_width = source_shape
    gain = min(model_height / source_height, model_width / source_width)
    pad_x = round((model_width - source_width * gain) / 2)
    pad_y = round((model_height - source_height * gain) / 2)
    cropped = mask[:, :, pad_y:model_height - pad_y, pad_x:model_width - pad_x]
    upscaled = torch.nn.functional.interpolate(cropped, size=(source_height, source_width), mode="bilinear")
    return upscaled.argmax(dim=1).squeeze().to(torch.uint8).cpu().numpy()


def place_cropped_mask(mask: torch.Tensor, offset: Tuple[int, int], source_shape: Tuple[int, int]) -> np.ndarray:
    y0, x0 = offset
    window = mask.argmax(dim=1).squeeze().to(torch.uint8).cpu().numpy()
    canvas = np.zeros(source_shape, dtype=np.uint8)
    canvas[y0:y0 + window.shape[0], x0:x0 + window.shape[1]] = window
    return canvas


def build_strategies(spec: str, horizon_ratio: float, band_width: int) -> List[Strategy]:
    strategies = []
    for token in spec.split(","):
        token = token.strip()
        kind, _, value = token.partition(":")
        if kind == "resize":
            strategies.append(make_resize_strategy(int(value)))
        elif kind == "crop":
            width, _, height = value.partition("x")
            strategies.append(make_crop_strategy(token, int(width), int(height), horizon_ratio))
        elif kind == "band":
            strategies.append(make_crop_strategy(token, band_width, int(value), horizon_ratio))
        else:
            raise SystemExit(f"unknown strategy '{token}'")
    return strategies


def load_detector(weights: str, device_spec: str):
    device = select_device(device=device_spec)
    detector = get_net(cfg)
    checkpoint = torch.load(weights, map_location=device, weights_only=False)
    detector.load_state_dict(checkpoint["state_dict"])
    use_half = device.type == "cuda"
    if use_half:
        detector = detector.to(device).half()
    detector.eval()
    return detector, device, use_half


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", required=True, help="directory or glob of source frames")
    parser.add_argument("--weights", default=osp.join(osp.dirname(osp.dirname(osp.abspath(__file__))),
                                                      "weights", "shiho-v1-20250321.pth"))
    parser.add_argument("--device", default="0")
    parser.add_argument("--confidence-threshold", type=float, default=0.80)
    parser.add_argument("--iou-threshold", type=float, default=0.6)
    parser.add_argument("--strategies", default=DEFAULT_STRATEGIES)
    parser.add_argument("--horizon-ratio", type=float, default=0.55,
                        help="vertical center of the native-scale window, as a fraction of image height")
    parser.add_argument("--band-width", type=int, default=1920, help="width of the 'band' strategy window")
    parser.add_argument("--far-row-ratio", type=float, default=0.6,
                        help="boxes whose bottom edge is above this fraction of the height count as far")
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--output-dir", default="", help="if set, write annotated frames here")
    args = parser.parse_args()

    paths = sorted(glob.glob(osp.join(args.images, "*.png")) + glob.glob(osp.join(args.images, "*.jpg"))) \
        if osp.isdir(args.images) else sorted(glob.glob(args.images))
    if not paths:
        raise SystemExit(f"no images found under '{args.images}'")

    detector, device, use_half = load_detector(args.weights, args.device)
    transform = transforms.Compose([transforms.ToTensor(), transforms.Normalize(IMAGENET_MEAN, IMAGENET_STDEV)])
    strategies = build_strategies(args.strategies, args.horizon_ratio, args.band_width)

    first = cv2.imread(paths[0])
    print(f"{len(paths)} frames, source {first.shape[1]}x{first.shape[0]}, device {device}, "
          f"half={use_half}, conf={args.confidence_threshold}\n")

    results = {}
    for strategy in strategies:
        far_counts, near_counts, box_heights, latencies_ms, far_mask_pixels = [], [], [], [], []
        model_shape = None
        for index, path in enumerate(paths):
            image = cv2.imread(path)
            annotated = image.copy() if args.output_dir else None
            window, to_source_boxes, to_source_mask = strategy.prepare(image)
            model_shape = window.shape[:2]

            tensor = transform(window).to(device)
            tensor = (tensor.half() if use_half else tensor.float()).unsqueeze(0)

            if index < args.warmup:
                with torch.no_grad():
                    detector(tensor)
                continue

            if device.type == "cuda":
                torch.cuda.synchronize()
            start = time.perf_counter()
            with torch.no_grad():
                object_raw, _, lane_raw = detector(tensor)
            if device.type == "cuda":
                torch.cuda.synchronize()
            latencies_ms.append((time.perf_counter() - start) * 1e3)

            detections = non_max_suppression(object_raw[0], conf_thres=args.confidence_threshold,
                                             iou_thres=args.iou_threshold, classes=None, agnostic=False)[0]
            boxes = to_source_boxes(detections[:, :4]).round().cpu().numpy() if len(detections) else np.empty((0, 4))
            mask = to_source_mask(lane_raw)

            far_row = args.far_row_ratio * image.shape[0]
            far = sum(1 for box in boxes if box[3] <= far_row)
            far_counts.append(far)
            near_counts.append(len(boxes) - far)
            box_heights.extend(float(box[3] - box[1]) for box in boxes)
            far_mask_pixels.append(int(np.count_nonzero(mask[: int(far_row)])))

            if annotated is not None:
                annotated[mask.astype(bool)] = (0, 255, 0)
                for box in boxes:
                    cv2.rectangle(annotated, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])),
                                  (255, 0, 0), max(int(image.shape[0] * 0.002), 1))
                out_dir = osp.join(args.output_dir, strategy.name.replace(":", "_"))
                os.makedirs(out_dir, exist_ok=True)
                cv2.imwrite(osp.join(out_dir, osp.basename(path)), annotated)

        results[strategy.name] = {
            "input": f"{model_shape[1]}x{model_shape[0]}" if model_shape else "?",
            "far": statistics.mean(far_counts) if far_counts else 0.0,
            "near": statistics.mean(near_counts) if near_counts else 0.0,
            "median_box_h": statistics.median(box_heights) if box_heights else float("nan"),
            "max_box_h": max(box_heights) if box_heights else float("nan"),
            "latency": statistics.median(latencies_ms) if latencies_ms else float("nan"),
            "far_mask_px": statistics.mean(far_mask_pixels) if far_mask_pixels else 0.0,
        }

    header = f"{'strategy':<18}{'input':>11}{'far/frame':>11}{'near/frame':>12}" \
             f"{'med box h':>11}{'max box h':>11}{'far mask px':>13}{'infer ms':>10}"
    print(header)
    print("-" * len(header))
    for name, row in results.items():
        print(f"{name:<18}{row['input']:>11}{row['far']:>11.2f}{row['near']:>12.2f}"
              f"{row['median_box_h']:>11.1f}{row['max_box_h']:>11.1f}"
              f"{row['far_mask_px']:>13.0f}{row['latency']:>10.1f}")
    print("\nLargest trained anchor is 68x157 px at the model input scale; a median box height well "
          "beyond that\nmeans the frame is being fed at a larger scale than the detect head expects.")
    if args.output_dir:
        print(f"Annotated frames written under {args.output_dir}")


if __name__ == "__main__":
    main()
