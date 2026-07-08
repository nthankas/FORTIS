"""
Shared PointCloud2 helpers for fortis_perception. No rclpy imports.

One XYZRGB point layout for every cloud publisher in the package
(depth_to_cloud_node, cloud_fusion_node, voxel_map_node), the
packed-rgb codec that goes with it, and the geometry_msgs Transform ->
4x4 matrix conversion the TF-consuming nodes share.

rgb is the classic packed convention (uint32 0x00RRGGBB bit-cast to
float32) that RViz and Foxglove decode natively. The packed bit
patterns are float32 denormals: keep them float32 end to end and never
let them round-trip through arithmetic.
"""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

#: XYZRGB point layout shared by every cloud publisher in fortis_perception.
FIELDS_XYZRGB = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
]

#: Bytes per point in the FIELDS_XYZRGB layout.
_POINT_STEP = 16


def pack_rgb(colors):
    """Pack an Nx3 uint8 RGB array into packed-float32 rgb field values."""
    c = np.asarray(colors, dtype=np.uint32)
    return ((c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]).view(np.float32)


def unpack_rgb(packed):
    """Unpack packed-float32 rgb field values into an Nx3 uint8 RGB array."""
    u = np.ascontiguousarray(packed, dtype=np.float32).view(np.uint32)
    out = np.empty((u.shape[0], 3), dtype=np.uint8)
    out[:, 0] = (u >> 16) & 0xFF
    out[:, 1] = (u >> 8) & 0xFF
    out[:, 2] = u & 0xFF
    return out


def make_xyzrgb_cloud(header, xyz, rgb_packed):
    """Build an XYZRGB PointCloud2 from Nx3 float xyz and N packed-rgb floats."""
    n = int(xyz.shape[0])
    if n == 0:
        # create_cloud's unstructured_to_structured rejects empty input.
        return PointCloud2(
            header=header, height=1, width=0, fields=FIELDS_XYZRGB,
            is_bigendian=False, point_step=_POINT_STEP, row_step=0,
            data=b"", is_dense=True)
    data = np.empty((n, 4), dtype=np.float32)
    data[:, :3] = xyz
    data[:, 3] = rgb_packed
    return point_cloud2.create_cloud(header, FIELDS_XYZRGB, data)


def transform_to_matrix(transform):
    """Convert a geometry_msgs Transform into a 4x4 homogeneous matrix."""
    q = transform.rotation
    t = transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4, dtype=np.float64)
    m[0, :3] = (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w))
    m[1, :3] = (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w))
    m[2, :3] = (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y))
    m[:3, 3] = (t.x, t.y, t.z)
    return m
