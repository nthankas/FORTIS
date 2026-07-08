# fortis_perception

FORTIS perception stack: RGBD point clouds, multi-camera fusion, voxel mapping
with cross-run diff, RGBD visual odometry, object detection, targeting, and
system health.

All modules are currently skeleton stubs: each node starts, registers its
name, and spins, so launch plumbing, entry points, and CI land ahead of the
algorithms. `fortis_sim_support` is the hardware-free test path for
everything here -- the OAK-D cameras cannot stream inside the WSL2 dev
container, so perception nodes are developed and tested against synthetic
sources and only verified live on the Jetson.

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
| `/fortis/context/{target_pose_valid, grasp_candidate_ok, ik_ok}` | `std_msgs/Bool` | target_selector / detection nodes |
| `/fortis/sim/ground_truth` | `nav_msgs/Odometry` | oak_replayer_node (fortis_sim_support) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | system_health_node + teensy_bridge |

## Planned modules

| Module | Role |
|---|---|
| `depth_to_cloud_node.py` | Node `depth_to_cloud`: per-camera depth + CameraInfo -> PointCloud2 in the optical frame. |
| `cloud_fusion_node.py` | Node `cloud_fusion`: TF-transforms per-camera clouds into `base_link` and merges them. |
| `voxel_map_node.py` | Node `voxel_map`: accumulates fused clouds into a voxel map in `odom`. |
| `voxel_grid.py` | Pure voxel-grid data structure and update math (no ROS imports). |
| `map_diff_node.py` | Node `map_diff`: diffs the live map against a saved run; markers + summary msg. |
| `rgbd_vo_node.py` | Node `rgbd_vo`: RGBD visual odometry publishing `/fortis/vo`. |
| `rgbd_vo.py` | Pure VO core (feature tracking + depth-informed pose solve). |
| `detection_node.py` | Node `detection`: 2D/3D object detection, annotations, grasp candidate. |
| `detectors.py` | Pure detection-backend wrappers behind a common interface. |
| `target_selector_node.py` | Node `target_selector`: picks the active target, publishes pose + context flags. |
| `system_health_node.py` | Node `system_health`: aggregates stack health onto `/diagnostics`. |
| `download_models.py` | `download_models` console script: fetches/caches detection model weights. |

## Conventions

- Latched topics use `fortis_comms.qos_profiles.latched_qos_profile`
  (TRANSIENT_LOCAL + RELIABLE); do not hand-roll QoS profiles.
- Tests pin `ROS_DOMAIN_ID=96` (see the registry in `test/conftest.py`).
