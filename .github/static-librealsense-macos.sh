#!/usr/bin/env bash
# Build librealsense as static archives, with its own static libusb, and write a
# pkg-config file that links all of it, so the macOS release binary carries the
# RealSense driver inside it and needs nothing installed (no brew, no nix).
#
# Upstream's own .pc is unusable for this: its libdir is hard-coded to
# lib/x86_64-linux-gnu and it lists none of the static dependencies.
set -euo pipefail

version="$1"
prefix="$2"

workdir=$(mktemp -d)
git clone --quiet --depth 1 --branch "v$version" https://github.com/IntelRealSense/librealsense.git "$workdir/src"
build="$workdir/src/build"
# The bundled libusb's include dir is declared one level too shallow upstream,
# so its header is put on the path by hand.
cmake -S "$workdir/src" -B "$build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=OFF \
    -DFORCE_RSUSB_BACKEND=ON \
    -DUSE_EXTERNAL_USB=ON \
    -DCMAKE_CXX_FLAGS="-I$build/libusb_install/include/libusb-1.0" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_UNIT_TESTS=OFF \
    -DBUILD_WITH_OPENMP=OFF \
    -DCHECK_FOR_UPDATES=OFF \
    -DBUILD_EASYLOGGINGPP=OFF \
    -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0 \
    -DCMAKE_INSTALL_PREFIX="$prefix"
cmake --build "$build" -j"$(sysctl -n hw.ncpu)"
cmake --install "$build"
cp "$build/libusb_install/lib/libusb-1.0.a" "$prefix/lib/"

cat > "$prefix/lib/pkgconfig/realsense2.pc" <<EOF
prefix=$prefix
libdir=\${prefix}/lib
includedir=\${prefix}/include

Name: realsense2
Description: librealsense, static
Version: $version
Libs: -L\${libdir} -lrealsense2 -lrealsense-file -lrsutils -lfw -lusb-1.0 -lc++ -lobjc -framework CoreFoundation -framework IOKit -framework Security
Cflags: -I\${includedir}
EOF
rm -rf "$workdir"
