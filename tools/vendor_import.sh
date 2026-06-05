#!/usr/bin/env bash
# =============================================================================
# vendor_import.sh - canonical vendoring command for FORTIS.
#
# Wraps `vcs import src < tools/vendor_repos.yaml` and then prunes the unused
# upstream example package(s) so they never add local colcon build time.
#
# vcstool imports whole repos and cannot sub-select packages. The only package
# we prune is odrive_botwheel_explorer, an upstream EXAMPLE (reference-only)
# that we never build; odrive_ros2_control + odrive_base are what fortis_control
# actually loads.
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

# Prune the unused upstream example so it never adds colcon build time.
PRUNED="src/ros_odrive/odrive_botwheel_explorer"
rm -rf "${PRUNED}"

echo "vendor_import: imported src/ from tools/vendor_repos.yaml"
echo "vendor_import: pruned unused example package ${PRUNED}"
