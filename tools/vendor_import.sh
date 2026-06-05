#!/usr/bin/env bash
# =============================================================================
# vendor_import.sh - canonical vendoring command for FORTIS.
#
# Wraps `vcs import src < tools/vendor_repos.yaml` and then prunes the unused
# upstream example package(s) so they never add local colcon build time.
#
# vcstool imports whole repos and cannot sub-select packages. We prune the two
# upstream packages we never build: odrive_botwheel_explorer (an EXAMPLE,
# reference-only) and odrive_node (a standalone CAN node we don't use).
# odrive_ros2_control + odrive_base are what fortis_control actually loads.
#
# Usage (from anywhere; the script resolves the repo root itself):
#     ./tools/vendor_import.sh
# =============================================================================

set -euo pipefail

# Resolve the repo root from this script's own location (tools/ -> repo root).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

# Pull in the vendored upstream packages.
vcs import src < tools/vendor_repos.yaml

# Prune the unused upstream packages so they never add colcon build time.
echo "vendor_import: imported src/ from tools/vendor_repos.yaml"
for pkg in odrive_botwheel_explorer odrive_node; do
    rm -rf "src/ros_odrive/${pkg}"
    echo "vendor_import: pruned unused package src/ros_odrive/${pkg}"
done
