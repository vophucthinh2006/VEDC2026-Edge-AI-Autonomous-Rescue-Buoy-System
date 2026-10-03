#!/bin/bash
# Build ArduPilot Rover SITL, the ArduPilot Gazebo plugin and the wave
# simulation. No sudo; run sim/install_deps.sh once before.
#   bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/build.sh [ardupilot|gazebo|waves]
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
set -u
JOBS="${JOBS:-$(nproc)}"
WHAT="${1:-all}"

build_ardupilot() {
    echo "=== ArduPilot Rover SITL"
    cd "$SIM_DIR/ardupilot"
    ./waf configure --board sitl
    ./waf rover -j"$JOBS"
}

build_gazebo_plugin() {
    echo "=== ardupilot_gazebo"
    cmake -S "$SIM_DIR/ardupilot_gazebo" -B "$SIM_DIR/ardupilot_gazebo/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build "$SIM_DIR/ardupilot_gazebo/build" -j"$JOBS"
}

build_waves() {
    echo "=== asv_wave_sim"
    # OGRE-Next ships inside the ROS vendor package, outside pkg-config's default search.
    export PKG_CONFIG_PATH="/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
    mkdir -p "$SIM_DIR/wave_ws"
    cd "$SIM_DIR/wave_ws"
    colcon build --merge-install \
        --base-paths "$SIM_DIR/asv_wave_sim" \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF -DCMAKE_CXX_STANDARD=17
    echo "=== waves_control GUI plugin"
    local gui="$SIM_DIR/asv_wave_sim/gz-waves/src/gui/plugins/waves_control"
    cmake -S "$gui" -B "$gui/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build "$gui/build" -j"$JOBS"
}

case "$WHAT" in
    ardupilot) build_ardupilot ;;
    gazebo)    build_gazebo_plugin ;;
    waves)     build_waves ;;
    all)       build_ardupilot; build_gazebo_plugin; build_waves ;;
    *) echo "usage: $0 [all|ardupilot|gazebo|waves]" >&2; exit 2 ;;
esac
echo "BUILD_OK $WHAT"
