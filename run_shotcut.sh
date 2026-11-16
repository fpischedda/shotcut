#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

if [ ! -f "${BUILD_DIR}/plugins/libmltosv.so" ]; then
    echo "[run_shotcut] Error: ${BUILD_DIR}/plugins/libmltosv.so not found! Run 'cmake --build build' first."
    exit 1
fi

export MLT_REPOSITORY="${BUILD_DIR}/plugins"
export MLT_DATA="${BUILD_DIR}/share/mlt-7"
export FREI0R_PATH="${BUILD_DIR}/plugins/frei0r:${FREI0R_PATH}"
export OSV_SHADER_PATH="${BUILD_DIR}/shaders/osv_stitch.spv"

echo "=========================================================="
echo " Launching Shotcut with Native OSV Vulkan Plugin (v4.4)"
echo " MLT_REPOSITORY : ${MLT_REPOSITORY}"
echo " MLT_DATA       : ${MLT_DATA}"
echo " OSV_SHADER_PATH: ${OSV_SHADER_PATH}"
echo " FREI0R_PATH    : ${FREI0R_PATH}"
echo "=========================================================="

exec shotcut "$@"
