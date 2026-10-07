#!/usr/bin/env python3
"""Convert the YOLOP checkpoint into a TensorRT engine for `object_road_detector_trt`.

The exported graph does everything the Python node used to do on the host after inference:
the detect head decode is folded in, an `EfficientNMS_TRT` node performs the box filtering and
suppression, and the two lane-line logits are collapsed into a single score channel. That leaves
the C++ node with nothing to do but a resize and a threshold.

Collapsing the lane head to `channel_1 - channel_0` is exact rather than a shortcut: for two
classes, the arg-max of a bilinear interpolation equals the sign of the bilinear interpolation of
the difference, because interpolation is linear. So the node can resize one channel instead of two
and still reproduce the mask the Python implementation produced.

Engines are specific to this GPU, TensorRT version and input size, so they are generated on the
machine that runs them and are not checked in.

    python3 export_engine.py --height 640 --width 1920
"""

import argparse
import os
import os.path as osp
import shutil
import subprocess
import sys
from typing import List

import numpy as np
import onnx
import onnx_graphsurgeon as gs
import torch
import torch.nn as nn

from yolop.lib.config import cfg
from yolop.lib.models import get_net

PACKAGE_DIR = osp.dirname(osp.dirname(osp.abspath(__file__)))
TRTEXEC_CANDIDATES = ("/usr/src/tensorrt/bin/trtexec", "trtexec")


def export_friendly_detect_forward(self, x: List[torch.Tensor]):
    """Detect head decode without the in-place writes that turn into ScatterND on export."""
    decoded = []
    for i in range(self.nl):
        x[i] = self.m[i](x[i])
        batch, _, grid_h, grid_w = x[i].shape
        x[i] = x[i].view(batch, self.na, self.no, grid_h * grid_w).permute(0, 1, 3, 2) \
                   .view(batch, self.na, grid_h, grid_w, self.no)
        grid = self._make_grid(grid_w, grid_h).to(x[i].device).type_as(x[i])
        activated = x[i].sigmoid()
        center = (activated[..., 0:2] * 2.0 - 0.5 + grid) * float(self.stride[i])
        size = (activated[..., 2:4] * 2.0) ** 2 * self.anchor_grid[i]
        decoded.append(torch.cat((center, size, activated[..., 4:]), dim=-1)
                       .view(batch, -1, self.no))
    return torch.cat(decoded, 1), x


class ExportWrapper(nn.Module):
    """Reshapes the YOLOP outputs into what `EfficientNMS_TRT` and the C++ node expect."""

    def __init__(self, detector: nn.Module, num_classes: int) -> None:
        super().__init__()
        self.detector = detector
        self.num_classes = num_classes

    def forward(self, images: torch.Tensor):
        detections, _, lane_line = self.detector(images)
        decoded = detections[0]
        boxes = decoded[..., 0:4]
        objectness = decoded[..., 4:5]
        class_scores = decoded[..., 5:5 + self.num_classes]
        scores = objectness * class_scores
        lane_score = lane_line[:, 1:2] - lane_line[:, 0:1]
        return boxes, scores, lane_score


def build_model(weights: str, num_classes: int) -> nn.Module:
    detector = get_net(cfg)
    checkpoint = torch.load(weights, map_location="cpu", weights_only=False)
    detector.load_state_dict(checkpoint["state_dict"])
    detector.eval()
    # The YOLOP sources are reachable both as `yolop.lib.*` and as `lib.*`, which makes them two
    # distinct class objects, so the head is matched by name and patched through its own type.
    detect_head = next(module for module in detector.modules() if type(module).__name__ == "Detect")
    if detect_head.nc != num_classes:
        raise SystemExit(f"checkpoint has {detect_head.nc} classes, expected {num_classes}")
    type(detect_head).forward = export_friendly_detect_forward
    return ExportWrapper(detector, num_classes).eval()


def export_onnx(model: nn.Module, height: int, width: int, path: str, opset: int) -> None:
    dummy_input = torch.zeros(1, 3, height, width)
    with torch.no_grad():
        torch.onnx.export(
            model, dummy_input, path, opset_version=opset, input_names=["images"],
            output_names=["boxes", "scores", "lane_score"], do_constant_folding=True)
    onnx.checker.check_model(onnx.load(path))


def append_nms(path: str, max_detections: int, score_threshold: float, iou_threshold: float) -> None:
    graph = gs.import_onnx(onnx.load(path))
    outputs = {tensor.name: tensor for tensor in graph.outputs}
    boxes, scores, lane_score = outputs["boxes"], outputs["scores"], outputs["lane_score"]

    num_detections = gs.Variable("num_detections", dtype=np.int32, shape=(1, 1))
    detection_boxes = gs.Variable("detection_boxes", dtype=np.float32, shape=(1, max_detections, 4))
    detection_scores = gs.Variable("detection_scores", dtype=np.float32, shape=(1, max_detections))
    detection_classes = gs.Variable("detection_classes", dtype=np.int32, shape=(1, max_detections))

    graph.nodes.append(gs.Node(
        op="EfficientNMS_TRT",
        name="efficient_nms",
        attrs={
            "background_class": -1,
            # The decode head emits center/size boxes, and the plugin returns corners.
            "box_coding": 1,
            "iou_threshold": iou_threshold,
            "max_output_boxes": max_detections,
            "plugin_version": "1",
            # Scores are already sigmoid activated and multiplied by the objectness.
            "score_activation": False,
            "score_threshold": score_threshold,
        },
        inputs=[boxes, scores],
        outputs=[num_detections, detection_boxes, detection_scores, detection_classes],
    ))
    graph.outputs = [num_detections, detection_boxes, detection_scores, detection_classes, lane_score]
    onnx.save(gs.export_onnx(graph.cleanup().toposort()), path)


def find_trtexec() -> str:
    for candidate in TRTEXEC_CANDIDATES:
        resolved = candidate if osp.isfile(candidate) else shutil.which(candidate)
        if resolved:
            return resolved
    raise SystemExit("trtexec not found; expected it at /usr/src/tensorrt/bin/trtexec")


def build_engine(onnx_path: str, engine_path: str, workspace_mib: int, extra_args: List[str]) -> None:
    command = [
        find_trtexec(),
        f"--onnx={onnx_path}",
        f"--saveEngine={engine_path}",
        "--fp16",
        f"--memPoolSize=workspace:{workspace_mib}",
        *extra_args,
    ]
    print("+ " + " ".join(command), flush=True)
    completed = subprocess.run(command)
    if completed.returncode != 0:
        raise SystemExit(f"trtexec failed with exit code {completed.returncode}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", default=osp.join(PACKAGE_DIR, "weights", "shiho-v1-20250321.pth"))
    parser.add_argument("--height", type=int, default=640,
                        help="engine input height, must be a multiple of 32")
    parser.add_argument("--width", type=int, default=1920,
                        help="engine input width, must be a multiple of 32")
    parser.add_argument("--num-classes", type=int, default=1)
    parser.add_argument("--max-detections", type=int, default=100)
    parser.add_argument("--score-threshold", type=float, default=0.80)
    parser.add_argument("--iou-threshold", type=float, default=0.6)
    parser.add_argument("--opset", type=int, default=13)
    parser.add_argument("--workspace-mib", type=int, default=2048)
    parser.add_argument("--output-dir", default=osp.join(PACKAGE_DIR, "weights"))
    parser.add_argument("--onnx-only", action="store_true", help="skip the trtexec engine build")
    parser.add_argument("trtexec_args", nargs="*", help="extra arguments forwarded to trtexec")
    args = parser.parse_args()

    for name, value in (("height", args.height), ("width", args.width)):
        if value % 32 != 0:
            raise SystemExit(f"--{name} must be a multiple of 32, got {value}")

    os.makedirs(args.output_dir, exist_ok=True)
    stem = f"{osp.splitext(osp.basename(args.weights))[0]}-{args.width}x{args.height}"
    onnx_path = osp.join(args.output_dir, stem + ".onnx")
    engine_path = osp.join(args.output_dir, stem + ".engine")

    print(f"Loading {args.weights}", flush=True)
    model = build_model(args.weights, args.num_classes)

    print(f"Exporting ONNX for {args.width}x{args.height} to {onnx_path}", flush=True)
    export_onnx(model, args.height, args.width, onnx_path, args.opset)

    print(f"Appending EfficientNMS_TRT (score>{args.score_threshold}, iou={args.iou_threshold}, "
          f"max={args.max_detections})", flush=True)
    append_nms(onnx_path, args.max_detections, args.score_threshold, args.iou_threshold)

    if args.onnx_only:
        print(f"Wrote {onnx_path}")
        return

    print(f"Building FP16 engine at {engine_path}", flush=True)
    build_engine(onnx_path, engine_path, args.workspace_mib, args.trtexec_args)
    print(f"\nEngine ready: {engine_path}")
    print(f"Point the node at it with 'engine_path:={engine_path}' "
          f"and use roi height {args.height}, width {args.width}.")


if __name__ == "__main__":
    sys.exit(main())
