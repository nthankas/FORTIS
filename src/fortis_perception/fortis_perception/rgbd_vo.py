"""
RGBD visual odometry core: ORB + depth-lifted PnP RANSAC. No ROS imports.

Frame conventions
-----------------
All geometry lives in the camera OPTICAL frame: X right, Y down, Z
forward. Poses and motions are 4x4 homogeneous matrices. ``RgbdVo.pose``
is the pose of the current camera expressed in the frame of the FIRST
tracked camera (the VO world), so a fixed camera<->body extrinsic maps
the whole trajectory into any body frame.

Estimation is KEYFRAME-anchored: every frame is matched against the
current keyframe, never against the previous frame, and the integrated
pose is ``keyframe_pose @ T_delta``. Per-frame matching noise therefore
does not compound between keyframe refreshes -- drift accrues per
keyframe, not per frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import cv2
import numpy as np


def rotation_matrix_to_quaternion(rot: np.ndarray) -> tuple:
    """Return the (x, y, z, w) unit quaternion for a 3x3 rotation matrix."""
    trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


def rotation_angle_rad(rot: np.ndarray) -> float:
    """Return the magnitude in radians of the rotation encoded by a 3x3 matrix."""
    cos_angle = (float(np.trace(rot)) - 1.0) / 2.0
    return math.acos(min(1.0, max(-1.0, cos_angle)))


@dataclass
class Keyframe:
    """Reference frame every incoming frame is matched and solved against."""

    gray: np.ndarray
    keypoints: Sequence
    descriptors: np.ndarray
    depth_mm: np.ndarray
    K: np.ndarray


@dataclass
class VoResult:
    """Outcome of one ``RgbdVo.process`` call.

    ``T_delta`` is the 4x4 keyframe->current camera motion (pose of the
    current camera expressed in the keyframe camera frame), or None when
    ``tracking`` is False.
    """

    T_delta: Optional[np.ndarray]
    n_inliers: int
    tracking: bool


class RgbdVo:
    """Keyframe-anchored RGBD odometry: ORB -> ratio-test BFMatcher -> PnP RANSAC."""

    #: Lowe ratio-test threshold on the two nearest Hamming distances.
    RATIO_TEST = 0.75
    #: cv2.solvePnPRansac settings (spec'd by the sprint plan).
    PNP_ITERATIONS = 300
    PNP_REPROJ_ERR_PX = 3.0

    def __init__(
        self,
        K: Optional[np.ndarray] = None,
        n_features: int = 800,
        min_inliers: int = 12,
        keyframe_trans_m: float = 0.12,
        keyframe_rot_rad: float = 0.2,
        min_depth_m: float = 0.15,
        max_depth_m: float = 6.0,
        max_delta_trans_m: float = 0.5,
        max_delta_rot_rad: float = 0.6,
    ) -> None:
        self.K = None if K is None else np.asarray(K, dtype=np.float64)
        self.min_inliers = int(min_inliers)
        self.keyframe_trans_m = float(keyframe_trans_m)
        self.keyframe_rot_rad = float(keyframe_rot_rad)
        self.max_delta_trans_m = float(max_delta_trans_m)
        self.max_delta_rot_rad = float(max_delta_rot_rad)
        self.min_depth_mm = float(min_depth_m) * 1000.0
        self.max_depth_mm = float(max_depth_m) * 1000.0
        #: Keyframes adopted so far (bootstrap included) -- observable
        #: evidence that the refresh policy fired, used by tests.
        self.keyframe_count = 0
        self._orb = cv2.ORB_create(nfeatures=int(n_features))
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self._keyframe: Optional[Keyframe] = None
        self._keyframe_pose = np.eye(4)
        self._pose = np.eye(4)

    @property
    def pose(self) -> np.ndarray:
        """Return the integrated 4x4 camera pose in the first-camera frame."""
        return self._pose.copy()

    def reset(self) -> None:
        """Drop all state: pose back to identity, next frame re-bootstraps."""
        self._keyframe = None
        self._keyframe_pose = np.eye(4)
        self._pose = np.eye(4)
        self.keyframe_count = 0

    def process(
        self,
        image: np.ndarray,
        depth_mm: np.ndarray,
        K: Optional[np.ndarray] = None,
    ) -> VoResult:
        """Estimate the keyframe->current camera motion for one RGBD frame.

        ``image`` is uint8 BGR or already-gray; ``depth_mm`` is uint16
        depth in millimetres registered to ``image`` (the OAK
        aligned-depth contract). ``K`` overrides the constructor camera
        matrix. On tracking loss the result carries ``T_delta=None`` and
        ``tracking=False``; the caller should publish nothing and let
        downstream filters coast.
        """
        cam_k = self.K if K is None else np.asarray(K, dtype=np.float64)
        if cam_k is None:
            raise ValueError("no camera matrix: pass K to __init__ or process()")
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self._orb.detectAndCompute(gray, None)
        if descriptors is None or len(keypoints) < self.min_inliers:
            return self._lose(0)
        if self._keyframe is None:
            # Bootstrap (or re-anchor after a loss): this frame becomes the
            # keyframe at the last known pose; motion resumes from there.
            self._adopt_keyframe(gray, keypoints, descriptors, depth_mm, cam_k)
            return VoResult(np.eye(4), 0, True)

        matches = self._matcher.knnMatch(self._keyframe.descriptors, descriptors, k=2)
        object_pts = []
        image_pts = []
        for pair in matches:
            if len(pair) < 2:
                continue
            best, second = pair
            if best.distance > self.RATIO_TEST * second.distance:
                continue
            point = self._lift_keyframe_point(best.queryIdx)
            if point is None:
                continue
            object_pts.append(point)
            image_pts.append(keypoints[best.trainIdx].pt)
        # solvePnPRansac's default iterative solver needs >= 6 points.
        if len(object_pts) < max(6, self.min_inliers):
            return self._lose(0)

        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            np.asarray(object_pts, dtype=np.float32),
            np.asarray(image_pts, dtype=np.float32),
            cam_k,
            None,
            iterationsCount=self.PNP_ITERATIONS,
            reprojectionError=self.PNP_REPROJ_ERR_PX,
        )
        n_inliers = 0 if inliers is None else int(len(inliers))
        if not ok or n_inliers < self.min_inliers:
            return self._lose(n_inliers)

        # LM refinement on the RANSAC inlier set: the minimal-set RANSAC
        # solution alone leaves ~px-scale residuals that integrate into
        # metres of drift over a run.
        idx = inliers.ravel()
        rvec, tvec = cv2.solvePnPRefineLM(
            np.asarray(object_pts, dtype=np.float32)[idx],
            np.asarray(image_pts, dtype=np.float32)[idx],
            cam_k,
            None,
            rvec,
            tvec,
        )

        # CONVENTION: solvePnP's (rvec, tvec) maps POINTS from the object
        # (keyframe camera) frame into the current camera frame:
        #     p_current = R @ p_keyframe + t
        # i.e. it is T_current<-keyframe acting on points. The camera
        # MOTION keyframe->current -- the pose of the current camera
        # expressed in the keyframe frame, which is what we integrate --
        # is the inverse: T_delta = [R^T | -R^T @ t].
        rot = cv2.Rodrigues(rvec)[0]
        t_delta = np.eye(4)
        t_delta[:3, :3] = rot.T
        t_delta[:3, 3] = (-rot.T @ tvec).ravel()

        # Plausibility gate: a RANSAC solve on a degenerate point set (flat
        # wall, near-max-range depth) can pass the inlier count with a wild
        # pose. One such frame would poison the integrated pose forever, so
        # reject anything a keyframe-to-frame step could not physically be.
        if (
            float(np.linalg.norm(t_delta[:3, 3])) > self.max_delta_trans_m
            or rotation_angle_rad(t_delta[:3, :3]) > self.max_delta_rot_rad
        ):
            return self._lose(n_inliers)

        self._pose = self._keyframe_pose @ t_delta

        refresh = (
            float(np.linalg.norm(t_delta[:3, 3])) > self.keyframe_trans_m
            or rotation_angle_rad(t_delta[:3, :3]) > self.keyframe_rot_rad
            or n_inliers < 2 * self.min_inliers
        )
        if refresh:
            self._adopt_keyframe(gray, keypoints, descriptors, depth_mm, cam_k)
        return VoResult(t_delta, n_inliers, True)

    def _adopt_keyframe(self, gray, keypoints, descriptors, depth_mm, cam_k) -> None:
        """Make the given frame the keyframe, anchored at the current pose."""
        self._keyframe = Keyframe(
            gray=gray,
            keypoints=tuple(keypoints),
            descriptors=descriptors,
            depth_mm=np.asarray(depth_mm),
            K=cam_k,
        )
        self._keyframe_pose = self._pose.copy()
        self.keyframe_count += 1

    def _lose(self, n_inliers: int) -> VoResult:
        """Register a tracking loss and re-arm for a fresh keyframe.

        Dropping the keyframe (rather than keeping a stale one) lets the
        next feature-rich frame re-anchor at the last known pose, so a
        brief occlusion costs only the motion during the gap -- during
        which the caller publishes nothing and the EKF coasts.
        """
        self._keyframe = None
        return VoResult(None, n_inliers, False)

    def _lift_keyframe_point(self, kp_index: int) -> Optional[np.ndarray]:
        """Back-project one keyframe keypoint to 3D via its depth patch.

        Use the median of the valid values in the 3x3 depth patch around
        the keypoint pixel -- an ORB corner sits exactly where stereo
        depth speckles, so single-pixel reads are unreliable. Zeros and
        out-of-range readings are rejected; None means no usable depth.
        """
        kf = self._keyframe
        u, v = kf.keypoints[kp_index].pt
        col, row = int(round(u)), int(round(v))
        height, width = kf.depth_mm.shape[:2]
        if not (0 <= col < width and 0 <= row < height):
            return None
        patch = kf.depth_mm[max(0, row - 1):row + 2, max(0, col - 1):col + 2]
        valid = patch[(patch >= self.min_depth_mm) & (patch <= self.max_depth_mm)]
        if valid.size == 0:
            return None
        z = float(np.median(valid)) / 1000.0
        fx, fy = kf.K[0, 0], kf.K[1, 1]
        cx, cy = kf.K[0, 2], kf.K[1, 2]
        return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
