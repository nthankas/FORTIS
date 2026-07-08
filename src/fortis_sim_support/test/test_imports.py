"""Smoke tests: every fortis_sim_support module imports, the node stub exposes main."""

import fortis_sim_support
from fortis_sim_support import (
    oak_replayer_node,
    raycaster,
    synthetic_scene,
    trajectory,
)


def test_package_imports():
    assert fortis_sim_support is not None


def test_pure_modules_import():
    for module in (synthetic_scene, raycaster, trajectory):
        assert module is not None


def test_node_module_has_main():
    assert callable(oak_replayer_node.main)
