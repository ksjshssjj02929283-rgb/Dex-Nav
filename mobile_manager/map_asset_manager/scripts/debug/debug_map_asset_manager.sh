#!/bin/bash

set -eo pipefail

# Run map_asset_manager_node under GDB with the same node name and parameter
# file as map_asset_manager.launch.py.
# Usage: debug_map_asset_manager.sh [params_file]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${SCRIPT_DIR}" == *"/install/map_asset_manager/lib/map_asset_manager" ]]; then
    WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
else
    WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"
fi

SETUP_FILE="${WORKSPACE_DIR}/install/setup.bash"
if [[ ! -f "${SETUP_FILE}" ]]; then
    echo "ERROR: ROS2 workspace setup not found: ${SETUP_FILE}" >&2
    exit 2
fi

# shellcheck disable=SC1090
source "${SETUP_FILE}"

PACKAGE_PREFIX="$(ros2 pkg prefix map_asset_manager)"
EXECUTABLE="${PACKAGE_PREFIX}/lib/map_asset_manager/map_asset_manager_node"
DEFAULT_PARAMS_FILE="${PACKAGE_PREFIX}/share/map_asset_manager/params/map_asset_manager_params.yaml"
PARAMS_FILE="${1:-${DEFAULT_PARAMS_FILE}}"
NODE_NAME="map_asset_manager"

if ! command -v gdb >/dev/null 2>&1; then
    echo "ERROR: gdb is not installed or not available in PATH" >&2
    exit 2
fi
if [[ ! -x "${EXECUTABLE}" ]]; then
    echo "ERROR: map_asset_manager executable not found: ${EXECUTABLE}" >&2
    exit 2
fi
if [[ ! -f "${PARAMS_FILE}" ]]; then
    echo "ERROR: parameter file not found: ${PARAMS_FILE}" >&2
    exit 2
fi

# A second same-name provider makes service calls nondeterministic and can hide
# the real crash. Stop the launch/supervisor-owned instance before debugging.
while IFS= read -r existing_node; do
    if [[ "${existing_node}" == "/${NODE_NAME}" ]]; then
        echo "ERROR: /${NODE_NAME} is already running." >&2
        echo "Stop its launch or supervisor owner before starting the GDB instance." >&2
        exit 3
    fi
done < <(ros2 node list 2>/dev/null || true)

echo "========================================"
echo "Map Asset Manager Debug Configuration"
echo "========================================"
echo "Workspace:  ${WORKSPACE_DIR}"
echo "Prefix:     ${PACKAGE_PREFIX}"
echo "Executable: ${EXECUTABLE}"
echo "Params:     ${PARAMS_FILE}"
echo "Node name:  /${NODE_NAME}"
echo "========================================"
echo "Starting GDB. Reproduce the crash from RViz, then run:"
echo "  bt"
echo "  bt full"
echo "  thread apply all bt full"
echo "  info threads"
echo "  q"
echo "========================================"

exec gdb \
    -ex "set pagination off" \
    -ex "set print thread-events off" \
    -ex "handle SIGPIPE nostop noprint pass" \
    -ex run \
    -args "${EXECUTABLE}" \
    --ros-args \
    -r "__node:=${NODE_NAME}" \
    --params-file "${PARAMS_FILE}"
