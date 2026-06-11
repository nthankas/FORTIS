"""Shared loader for the OAK chassis camera capture config.

The four chassis OAK-D Lites are identical units, so they all run ONE tested
capture config -- config/oak_chassis_cameras.yaml -- the single source of truth
for pipeline / resolution / fps / depth tuning / image_transport. Both the
multi-camera bring-up (oak_chassis_cameras.launch.py, the primary path) and the
single-camera front debug launch (oak_chassis_front.launch.py) load it through
this one helper; the launches add only per-camera TF / serial overrides on top.

Why a dict, not the yaml file path
----------------------------------
depthai-ros v3 keys a params FILE by the node name -- the file is silently
ignored if the top-level key does not match. Rather than tie the shared config
to one node name, we load it here and return a NODE-AGNOSTIC param dict, which
launch_ros applies under whatever node name the launch gives it. That is the
path the multi-cam launch has always used and that was verified live on all
four cameras (2026-06-05); routing the single-cam launch through it too removes
the last node-name coupling. The yaml's top-level key is therefore a positional
placeholder (read by position, name ignored) -- see the yaml header.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

#: Capture config filename under fortis_bringup/config. Shared by every OAK.
CAMERA_PARAMS_FILE = "oak_chassis_cameras.yaml"

#: Key in the yaml holding the image_transport plugin enable-lists. It is
#: stripped out and merged up into the node params before anything is applied,
#: so it is keyed for clarity, NOT as a node name -- the loader removes it.
_TRANSPORT_KEY = "image_transport"


def _deep_merge(base, extra):
    """Recursively merge ``extra`` into ``base`` (nested dicts merged, not replaced)."""
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value


def load_camera_params():
    """Return the shared OAK capture config as a node-agnostic param dict.

    Reads config/oak_chassis_cameras.yaml, takes its single ros__parameters
    block (the top-level key name is irrelevant -- read by position), and folds
    the image_transport plugin block up to the top level so it applies under any
    camera node name.
    """
    params_file = os.path.join(
        get_package_share_directory("fortis_bringup"),
        "config",
        CAMERA_PARAMS_FILE,
    )
    with open(params_file) as handle:
        raw = yaml.safe_load(handle)
    cfg = next(iter(raw.values()))["ros__parameters"]
    transport = cfg.pop(_TRANSPORT_KEY, {})
    _deep_merge(cfg, transport)
    return cfg
