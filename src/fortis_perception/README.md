# fortis_perception

FORTIS perception stack: RGBD point clouds, multi-camera fusion, voxel mapping
with cross-run diff, RGBD visual odometry, object detection, targeting, and
system health.

`fortis_sim_support` is the hardware-free test path for everything here -- the
OAK-D cameras cannot stream inside the WSL2 dev container, so perception nodes
are developed and tested against synthetic sources and only verified live on
the Jetson. `fortis_bringup/launch/perception.launch.py` composes the whole
chain against either source with one command.

## Topic registry (single source of truth for this sprint)

| Topic | Type | Publisher |
|---|---|---|
| `/fortis/perception/<cam>/points` | `sensor_msgs/PointCloud2` (optical frame) | depth_to_cloud_node |
| `/fortis/perception/points_fused` | `PointCloud2` (base_link) | cloud_fusion_node |
| `/fortis/perception/map/cloud` | `PointCloud2` (odom) | voxel_map_node |
| `/fortis/perception/map_diff/markers` | `visualization_msgs/MarkerArray` | map_diff_node |
| `/fortis/perception/map_diff/summary` | `fortis_msgs/MapDiffSummary` | map_diff_node |
| `/fortis/perception/detections` | `vision_msgs/Detection2DArray` | detection_node |
| `/fortis/perception/detections3d` | `vision_msgs/Detection3DArray` | detection_node |
| `/fortis/perception/detection_markers` | `MarkerArray` | detection_node |
| `/fortis/perception/annotations/<cam>` | `foxglove_msgs/ImageAnnotations` | detection_node |
| `/fortis/perception/grasp_candidate` | `fortis_msgs/GraspCandidate` | detection_node |
| `/fortis/vo` | `nav_msgs/Odometry` | rgbd_vo_node |
| `/fortis/target_pose` | `geometry_msgs/PoseStamped` | target_selector_node |
| `/fortis/context/{target_pose_valid, grasp_candidate_ok}` | `std_msgs/Bool` | target_selector / detection nodes |
| `/fortis/context/ik_ok` | `std_msgs/Bool` | arm_motion (fortis_arm) |
| `/fortis/sim/ground_truth` | `nav_msgs/Odometry` | oak_replayer_node (fortis_sim_support) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | system_health_node + teensy_bridge |

## Pipeline data flow

Each camera (a real OAK via `fortis_bringup`, or an `oak_replayer` from
`fortis_sim_support` -- same topic contract) publishes device-encoded JPEG RGB,
rgb-aligned 16UC1 depth, and CameraInfo. One `depth_to_cloud` per camera
back-projects the depth image into an XYZRGB cloud in that camera's optical
frame. `cloud_fusion` TF-transforms every per-camera cloud into `base_link` on
a fixed-rate timer, then merges and voxel-downsamples them. `voxel_map`
integrates the fused cloud into a persistent voxel grid in `odom` and publishes
the occupied centers latched; a grid saved via `~/save_map` becomes a later
run's reference for `map_diff`, which publishes added/removed voxels as markers
plus a machine-readable summary. In parallel, off the FRONT camera only:
`rgbd_vo` estimates camera motion (keyframe-anchored ORB + PnP) and publishes
`/fortis/vo` for the EKF's twist fusion, and `detection` runs the configured
backend at a fixed rate, lifts boxes to 3D through the depth median and into
`base_link` through the shared front-mount extrinsic (`geometry.py`), and
nominates the nearest graspable object as the grasp candidate.
`target_selector` turns an operator's Foxglove 3D click into the mission target
pose and FSM event, and `system_health` grades every stream's rate onto
`/diagnostics`.

## Nodes

### depth_to_cloud (one instance per camera)

Back-projects one camera's aligned depth through the pinhole model and colours
each point from the JPEG RGB stream. A pair arriving while the previous one is
still reprojecting is dropped, not queued.

| Param | Default | Meaning |
|---|---|---|
| `camera_name` | `oak_chassis_front` | Camera topic namespace to consume. |
| `stride` | `4` | Pixel subsampling step (16x fewer points at 4). |
| `min_range_m` / `max_range_m` | `0.15` / `6.0` | Depth band kept in the cloud. |

In: `/<cam>/rgb/image_raw/compressed`, `/<cam>/stereo/image_raw`,
`/<cam>/stereo/camera_info`. Out: `/fortis/perception/<cam>/points`
(optical frame).

### cloud_fusion

Buffers the newest cloud per input topic and merges them on a timer; a camera
whose TF is missing is skipped for that cycle instead of stalling the rest.

| Param | Default | Meaning |
|---|---|---|
| `input_topics` | the four `/fortis/perception/oak_chassis_*/points` | Per-camera clouds to fuse. |
| `target_frame` | `base_link` | Output frame. |
| `voxel_size` | `0.05` | Downsample cell size (m); `<= 0` disables. |
| `publish_rate_hz` | `5.0` | Fusion timer rate. |

In: each input topic + TF. Out: `/fortis/perception/points_fused`.

### voxel_map

Persistent occupancy map; the pure grid math lives in `voxel_grid.py`.

| Param | Default | Meaning |
|---|---|---|
| `voxel_size` | `0.05` | Grid resolution (m); must match `map_diff`'s. |
| `target_frame` | `odom` | Map frame. |
| `min_hits` | `3` | Hits before a voxel counts as occupied. |
| `integrate_rate_hz` | `5.0` | How often the buffered fused cloud is folded in. |
| `publish_rate_hz` | `1.0` | Latched map-cloud publish rate. |
| `map_dir` | `~/fortis_maps` | Where empty-path saves land (timestamped .npz). |

In: `/fortis/perception/points_fused` + TF. Out:
`/fortis/perception/map/cloud` (latched). Services: `~/save_map`
(`fortis_msgs/SaveMap`), `~/load_map` (`fortis_msgs/LoadMap`), `~/clear`
(`std_srvs/Trigger`).

### map_diff

Diffs the live map against a saved reference run; idles until a reference is
configured (parameter or `~/load_reference`).

| Param | Default | Meaning |
|---|---|---|
| `reference_map` | `""` | Path to a saved `.npz`; empty = idle until the service. |
| `voxel_size` | `0.05` | Must equal the reference's (enforced on load). |
| `min_hits` | `3` | Occupancy threshold used for the diff. |
| `compute_rate_hz` | `0.5` | Diff recompute rate. |

In: `/fortis/perception/map/cloud` (latched). Out:
`/fortis/perception/map_diff/markers` (CUBE_LIST: id 0 = added, green; id 1 =
removed, red), `/fortis/perception/map_diff/summary` (latched). Service:
`~/load_reference` (`fortis_msgs/LoadMap`).

### rgbd_vo

Front-camera RGBD visual odometry; the pure core is `rgbd_vo.py`. Pairs each
RGB frame with the latest cached depth (bounded-skew cache instead of a strict
synchronizer: on-device MJPEG encoding latency wobbles). On tracking loss it
publishes NO Odometry -- the EKF coasts on wheels + IMU -- and keeps
`/fortis/vo/inliers` alive at 0.0 as the cheap health signal. Covariance
scales with 1/inliers so the EKF's trust follows feature support.

| Param | Default | Meaning |
|---|---|---|
| `camera_name` | `oak_chassis_front` | Camera to track. Must be the front mount: the base_link lift uses the front extrinsic. |
| `n_features` | `800` | ORB features per frame. |
| `min_inliers` | `12` | PnP inlier floor below which tracking is lost. |

In: `/<cam>/rgb/image_raw/compressed`, `/<cam>/stereo/image_raw`,
`/<cam>/rgb/camera_info`. Out: `/fortis/vo` (pose advisory; only the twist is
fused, by `ekf_vio.yaml`), `/fortis/vo/inliers` (`std_msgs/Float32`).

### detection

Rate-limited detection on the front camera via a `detectors.py` backend. With
`detector:=yolo` and no weights on disk it degrades: one warning, empty
`Detection2DArray` at rate plus a WARN on `/diagnostics`, and stays alive.

| Param | Default | Meaning |
|---|---|---|
| `camera_name` | `oak_chassis_front` | Camera to run detection on (front: see `geometry.py`). |
| `detector` | `blob` | `blob` (HSV, CI/synthetic) \| `yolo` (YOLOv8 ONNX; needs weights + OpenCV >= 4.7). |
| `model_path` | `~/.cache/fortis/models/yolov8n.onnx` | YOLO weights (`download_models` fills it). |
| `target_rate_hz` | `2.0` | Detection pass rate; frames between passes are dropped. |
| `graspable_classes` | `bottle, cup, sports ball, red_sphere` | Classes eligible to become the grasp candidate. |
| `candidate_timeout_s` | `3.0` | No fresh candidate for this long drops `grasp_candidate_ok`. |
| `min_score` | `0.3` | Detections below this confidence are discarded. |

In: `/<cam>/rgb/image_raw/compressed`, `/<cam>/stereo/image_raw`,
`/<cam>/stereo/camera_info`. Out: `/fortis/perception/detections`,
`detections3d` (optical frame), `detection_markers` (base_link),
`annotations/<cam>`, `grasp_candidate` (nearest graspable within 2.5 m),
`/fortis/context/grasp_candidate_ok` (latched), `/diagnostics` (degraded
mode only).

### target_selector

Turns a Foxglove 3D-panel click (`/clicked_point`) into the mission target:
validates against a horizontal range annulus around the robot, anchors the
pose in `odom` when TF allows (falls back to `base_link`), then fires the
FSM's click event (`Event.CHASSIS_CAM_CLICK`: ORBIT -> TARGETING when the
`target_pose_valid` guard holds).

| Param | Default | Meaning |
|---|---|---|
| `min_target_range_m` | `0.3` | Inner annulus radius. |
| `max_target_range_m` | `3.5` | Outer annulus radius. |

In: `/clicked_point` + TF. Out: `/fortis/target_pose` (latched),
`/fortis/context/target_pose_valid` (latched),
`/fortis/events/chassis_cam_click`.

### system_health

1 Hz `/diagnostics` roll-up of the perception / VIO / mission / arm stack.
Every input is optional: a missing publisher shows up as an ERROR/stale
entry, never a crash, so this node runs fine on a partially-launched stack.

| Param | Default | Meaning |
|---|---|---|
| `cameras` | the four `oak_chassis_*` names | RGB streams to rate-check. |
| `expected_camera_rate_hz` | `15.0` | Full-rate reference for camera grading. |
| `expected_vo_rate_hz` | `10.0` | Full-rate reference for `/fortis/vo`. |
| `expected_fused_rate_hz` | `5.0` | Full-rate reference for `points_fused`. |

In: per-camera RGB, `/fortis/vo`, `/fortis/vo/inliers`,
`/fortis/perception/points_fused`, `/fortis/perception/map/cloud`,
`/fortis/mission_state`, `/fortis/arm/status`. Out: `/diagnostics`
(rate > 50% of expected = OK, > 0 = WARN, silent = ERROR).

### download_models (console script)

`ros2 run fortis_perception download_models [--dest DIR] [--force]` fetches
the pinned YOLOv8n ONNX export (SHA256-verified, atomic write) into
`~/.cache/fortis/models/`.

## Shared modules (no ROS node)

| Module | Role |
|---|---|
| `voxel_grid.py` | Sparse voxel occupancy grid: integrate / occupied / save / load / diff. Pure numpy. |
| `rgbd_vo.py` | Keyframe-anchored VO core: ORB -> ratio test -> PnP RANSAC + LM refine. Pure cv2/numpy. |
| `detectors.py` | Detection backends (`YoloV8OnnxDetector`, `HsvBlobDetector`) behind one `detect(bgr)` interface. |
| `cloud_utils.py` | Shared XYZRGB PointCloud2 layout, packed-rgb codec, Transform -> 4x4 conversion. |
| `geometry.py` | URDF front-camera extrinsic (base_link <-> optical) + fixed-axis rpy helper. |

## Conventions

- Latched topics use `fortis_comms.qos_profiles.latched_qos_profile`
  (TRANSIENT_LOCAL + RELIABLE); do not hand-roll QoS profiles.
- Tests pin `ROS_DOMAIN_ID=96` (see the registry in `test/conftest.py`).
