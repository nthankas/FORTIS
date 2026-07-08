"""
Pure detector tests: no rclpy, no DDS.

The HSV blob detector is driven with shapes drawn by cv2 (same colors the
fortis_sim_support synthetic scenes use); the YOLO wrapper is checked for
its ModelUnavailable contract and, when weights happen to be cached
locally, a smoke inference.
"""

from __future__ import annotations

import os
from pathlib import Path

import cv2
import numpy as np
import pytest

from fortis_perception.detectors import (
    HsvBlobDetector,
    ModelUnavailable,
    YoloV8OnnxDetector,
)

MODEL_PATH = Path(os.path.expanduser("~/.cache/fortis/models/yolov8n.onnx"))


def _scene() -> np.ndarray:
    """Draw the synthetic test frame: red disc + green rectangle on grey."""
    img = np.full((200, 320, 3), 128, dtype=np.uint8)
    cv2.circle(img, (80, 100), 30, (0, 0, 255), -1)  # red disc (BGR)
    cv2.rectangle(img, (200, 60), (260, 120), (0, 200, 0), -1)  # green box
    return img


def test_blob_detector_finds_colored_shapes():
    """Both primitives are found with the right class names and covering boxes."""
    detections = HsvBlobDetector().detect(_scene())
    by_class = {d.class_name: d for d in detections}
    assert set(by_class) == {"red_sphere", "green_box"}

    # Red disc: center (80, 100), radius 30 -> box ~ (50, 70)-(110, 130).
    x1, y1, x2, y2 = by_class["red_sphere"].xyxy
    assert x1 <= 55 and x2 >= 105
    assert y1 <= 75 and y2 >= 125

    # Green rectangle drawn at (200, 60)-(260, 120).
    x1, y1, x2, y2 = by_class["green_box"].xyxy
    assert x1 <= 205 and x2 >= 255
    assert y1 <= 65 and y2 >= 115

    for det in detections:
        assert 0.0 < det.score <= 1.0


def test_blob_detector_empty_on_grey_image():
    """A featureless grey frame produces no detections."""
    img = np.full((200, 320, 3), 128, dtype=np.uint8)
    assert HsvBlobDetector().detect(img) == []


def test_yolo_missing_weights_raises_model_unavailable():
    """The yolo wrapper refuses to construct without weights on disk."""
    with pytest.raises(ModelUnavailable):
        YoloV8OnnxDetector("/nonexistent/path/yolov8n.onnx")


@pytest.mark.skipif(not MODEL_PATH.exists(),
                    reason="yolov8n.onnx not downloaded (run download_models)")
def test_yolo_runs_on_real_weights():
    """Smoke-run real weights when cached: output shape/contract only."""
    detector = YoloV8OnnxDetector(str(MODEL_PATH))
    detections = detector.detect(_scene())
    assert isinstance(detections, list)
    for det in detections:
        assert 0.0 <= det.score <= 1.0
        x1, y1, x2, y2 = det.xyxy
        assert x1 <= x2 and y1 <= y2
