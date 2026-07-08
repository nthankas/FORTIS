# Aggregate result of comparing the live voxel map against a reference map
# from a previous run (fortis_perception map_diff_node). The per-voxel detail
# ships separately as visualization markers; this is the machine-readable
# roll-up for logging, UI raw views, and mission logic.
#
# reference_map     Filesystem path of the reference map the live map was
#                   compared against.
# voxel_size        Edge length (m) of one voxel in both maps.
# added_voxels      Occupied in the live map, absent from the reference.
# removed_voxels    Occupied in the reference, absent from the live map.
# added_volume_m3   added_voxels * voxel_size^3.
# removed_volume_m3 removed_voxels * voxel_size^3.
# stamp             Time the diff was computed.

builtin_interfaces/Time stamp
string reference_map
float32 voxel_size
uint32 added_voxels
uint32 removed_voxels
float32 added_volume_m3
float32 removed_volume_m3
