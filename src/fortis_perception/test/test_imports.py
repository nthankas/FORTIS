"""Smoke tests: every fortis_perception module imports, each node stub exposes main."""

import fortis_perception
from fortis_perception import (
    cloud_fusion_node,
    depth_to_cloud_node,
    detection_node,
    detectors,
    download_models,
    map_diff_node,
    rgbd_vo,
    rgbd_vo_node,
    system_health_node,
    target_selector_node,
    voxel_grid,
    voxel_map_node,
)

NODE_MODULES = (
    depth_to_cloud_node,
    cloud_fusion_node,
    voxel_map_node,
    map_diff_node,
    rgbd_vo_node,
    detection_node,
    target_selector_node,
    system_health_node,
)

PURE_MODULES = (voxel_grid, rgbd_vo, detectors)


def test_package_imports():
    assert fortis_perception is not None


def test_pure_modules_import():
    for module in PURE_MODULES:
        assert module is not None


def test_every_node_module_has_main():
    for module in NODE_MODULES:
        assert callable(module.main), module.__name__


def test_download_models_has_main():
    assert callable(download_models.main)
