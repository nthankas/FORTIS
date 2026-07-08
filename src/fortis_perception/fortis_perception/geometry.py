"""
Shared frame geometry for fortis_perception. Pure numpy, no ROS imports.

Home of the URDF front chassis camera extrinsic, previously hardcoded
separately in rgbd_vo_node and detection_node. The constants mirror
fortis_description's URDF rather than a live TF lookup so consumers
work before robot_state_publisher is up -- the mount is bolted, so a
constant beats a TF dependency.

Frame conventions
-----------------
- Camera OPTICAL frame: X right, Y down, Z forward.
- base_link: X forward, Y left, Z up (REP-103). The FORTIS chassis
  "front" -- where the front camera points -- is base_link -X.

Derivation (fortis_chassis.urdf.xacro + fortis_constants.xacro): the
front camera link is mounted at xyz=(-cam_front_x, 0, cam_height_z),
rpy=(0, -0.524, pi) -- the base_link -X face, pitched 30 deg up -- and
its optical joint adds rpy=(-pi/2, 0, -pi/2), with
cam_front_x = chassis_length/2 + cam_edge_to_housing + oak_lite_z/2
            = 0.332/2 + 0.01933 + 0.017/2 = 0.19383
cam_height_z = 0.21514.
"""

from __future__ import annotations

import math

import numpy as np


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the 3x3 rotation for URDF fixed-axis rpy (R = Rz @ Ry @ Rx)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


#: Front camera position in base_link (m); see the derivation above.
FRONT_CAM_XYZ = np.array([-(0.332 / 2 + 0.01933 + 0.017 / 2), 0.0, 0.21514])

#: base_link <- front-camera-optical rotation: URDF mount, then optical joint.
FRONT_CAM_R = (rpy_matrix(0.0, -0.524, math.pi)
               @ rpy_matrix(-math.pi / 2, 0.0, -math.pi / 2))


def _t_base_cam() -> np.ndarray:
    """Assemble the 4x4 base_link -> front-camera-optical extrinsic."""
    t = np.eye(4)
    t[:3, :3] = FRONT_CAM_R
    t[:3, 3] = FRONT_CAM_XYZ
    return t


#: base_link -> camera-optical extrinsic: the X in the similarity
#: T_base = X @ T_cam @ X^-1 that maps camera motion into base_link motion.
T_BASE_CAM = _t_base_cam()
T_CAM_BASE = np.linalg.inv(T_BASE_CAM)


def optical_to_base(p_optical) -> np.ndarray:
    """Map a point from the front camera's optical frame into base_link."""
    return FRONT_CAM_R @ np.asarray(p_optical, dtype=float) + FRONT_CAM_XYZ
