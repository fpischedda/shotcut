#!/usr/bin/env bash
set -e

BUILD_DIR="$1"
if [ -z "$BUILD_DIR" ]; then
    echo "Usage: $0 <build_directory>"
    exit 1
fi

mkdir -p "${BUILD_DIR}/plugins"
mkdir -p "${BUILD_DIR}/share/mlt-7/core"

# Symlink system MLT plugins into build/plugins
for f in /usr/lib/mlt-7/*.so; do
    if [ -e "$f" ]; then
        ln -sf "$f" "${BUILD_DIR}/plugins/"
    fi
done

# Symlink system MLT data except core into build/share/mlt-7
for d in /usr/share/mlt-7/*; do
    name="$(basename "$d")"
    if [ "$name" != "core" ]; then
        rm -rf "${BUILD_DIR}/share/mlt-7/${name}"
        ln -sfn "$d" "${BUILD_DIR}/share/mlt-7/${name}"
    fi
done

# Symlink core files except loader.dict
for f in /usr/share/mlt-7/core/*; do
    name="$(basename "$f")"
    if [ "$name" != "loader.dict" ]; then
        rm -rf "${BUILD_DIR}/share/mlt-7/core/${name}"
        ln -sfn "$f" "${BUILD_DIR}/share/mlt-7/core/${name}"
    fi
done

# Create combined loader.dict with OSV routing prepended
printf '*.osv=osv\n*.OSV=osv\n' > "${BUILD_DIR}/share/mlt-7/core/loader.dict"
cat /usr/share/mlt-7/core/loader.dict >> "${BUILD_DIR}/share/mlt-7/core/loader.dict"

echo "[Setup-MLT-Env] Environment populated in ${BUILD_DIR}"
