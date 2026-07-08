"""
Pure object-detection backends (cv2 + numpy only, no ROS imports).

Two interchangeable detectors behind a common ``detect(bgr) ->
list[Detection]`` interface:

- YoloV8OnnxDetector: cv2.dnn wrapper around a YOLOv8 ONNX export (COCO
  classes). Raises ModelUnavailable when the weights file is absent so
  callers can degrade instead of crashing; weights are fetched by the
  download_models console script and never committed.
- HsvBlobDetector: deterministic HSV color-blob detector that finds the
  distinctly-colored primitives rendered by fortis_sim_support. This is
  the CI backend -- no model weights required.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    """One detected object: class name, confidence, and pixel box (x1, y1, x2, y2)."""

    class_name: str
    score: float
    xyxy: tuple[int, int, int, int]


class ModelUnavailable(RuntimeError):
    """Raised when a detector's model weights are not on disk."""


#: COCO class names in YOLOv8 output order.
COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


class YoloV8OnnxDetector:
    """YOLOv8 ONNX detector run through cv2.dnn (COCO classes)."""

    def __init__(self, model_path: str, conf_th: float = 0.35,
                 nms_th: float = 0.45, input_size: int = 640):
        path = os.path.expanduser(str(model_path))
        if not os.path.isfile(path):
            raise ModelUnavailable(f"YOLOv8 ONNX weights not found: {path}")
        self._net = cv2.dnn.readNetFromONNX(path)
        self._conf_th = float(conf_th)
        self._nms_th = float(nms_th)
        self._input_size = int(input_size)

    def detect(self, bgr: np.ndarray) -> list[Detection]:
        """Run the network on a BGR frame and return NMS-filtered detections."""
        img_h, img_w = bgr.shape[:2]
        size = self._input_size

        # Letterbox: scale to fit, pad with the YOLO-conventional grey 114.
        scale = min(size / img_w, size / img_h)
        new_w, new_h = int(round(img_w * scale)), int(round(img_h * scale))
        pad_x, pad_y = (size - new_w) // 2, (size - new_h) // 2
        canvas = np.full((size, size, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv2.resize(bgr, (new_w, new_h))

        blob = cv2.dnn.blobFromImage(canvas, 1.0 / 255.0, (size, size),
                                     swapRB=True, crop=False)
        self._net.setInput(blob)
        out = np.squeeze(self._net.forward())
        if out.ndim != 2:
            return []
        # Canonical YOLOv8 layout is (84, 8400) = (cx,cy,w,h + 80 scores,
        # anchors); some exporters emit the transpose. Normalize to rows =
        # anchors.
        if out.shape[0] == 4 + len(COCO_CLASSES):
            out = out.T

        cls_scores = out[:, 4:4 + len(COCO_CLASSES)]
        ids = np.argmax(cls_scores, axis=1)
        confs = cls_scores[np.arange(len(ids)), ids]
        keep = confs >= self._conf_th

        boxes: list[list[int]] = []
        scores: list[float] = []
        class_ids: list[int] = []
        for (bcx, bcy, bw, bh), conf, cid in zip(out[keep, :4], confs[keep], ids[keep]):
            x = (bcx - bw / 2.0 - pad_x) / scale
            y = (bcy - bh / 2.0 - pad_y) / scale
            boxes.append([int(x), int(y), int(bw / scale), int(bh / scale)])
            scores.append(float(conf))
            class_ids.append(int(cid))
        if not boxes:
            return []

        picked = cv2.dnn.NMSBoxes(boxes, scores, self._conf_th, self._nms_th)
        detections = []
        for i in np.array(picked).flatten():
            i = int(i)
            x, y, bw, bh = boxes[i]
            x1, y1 = max(0, x), max(0, y)
            x2, y2 = min(img_w - 1, x + bw), min(img_h - 1, y + bh)
            detections.append(
                Detection(COCO_CLASSES[class_ids[i]], scores[i], (x1, y1, x2, y2))
            )
        return detections


#: Saturation/value floors for "vivid" pixels, in OpenCV's 0-255 scale.
#: 115 ~= 0.45 * 255; kept forgiving so jpeg round-trips of the synthetic
#: primitives still land inside the bands.
_SAT_MIN = 115
_VAL_MIN = 40

#: Full-scale area (px) that maps to score 1.0.
_SCORE_FULL_AREA = 5000.0


class HsvBlobDetector:
    """Deterministic HSV blob detector for the synthetic-scene primitives.

    Fixed hue bands (OpenCV hue is 0-179): red < 10 or > 170, green 40-85,
    blue 95-135; each band becomes one class named after the primitive it
    detects in fortis_sim_support scenes.
    """

    def __init__(self, min_area_px: int = 120):
        self._min_area = int(min_area_px)

    def detect(self, bgr: np.ndarray) -> list[Detection]:
        """Segment vivid hue bands into connected blobs and return their boxes."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        vivid = (sat >= _SAT_MIN) & (val >= _VAL_MIN)
        bands = {
            "red_sphere": (hue < 10) | (hue > 170),
            "green_box": (hue > 40) & (hue < 85),
            "blue_box": (hue > 95) & (hue < 135),
        }
        kernel = np.ones((3, 3), np.uint8)
        detections = []
        for name, band in bands.items():
            mask = (band & vivid).astype(np.uint8) * 255
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            n, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            for i in range(1, n):
                area = int(stats[i, cv2.CC_STAT_AREA])
                if area < self._min_area:
                    continue
                x = int(stats[i, cv2.CC_STAT_LEFT])
                y = int(stats[i, cv2.CC_STAT_TOP])
                w = int(stats[i, cv2.CC_STAT_WIDTH])
                h = int(stats[i, cv2.CC_STAT_HEIGHT])
                score = min(1.0, area / _SCORE_FULL_AREA)
                detections.append(Detection(name, score, (x, y, x + w, y + h)))
        # Deterministic output order regardless of dict/band iteration.
        detections.sort(key=lambda d: (d.xyxy[0], d.xyxy[1], d.class_name))
        return detections
