#!/bin/bash
# One-time system packages for the boat simulator, run INSIDE WSL Ubuntu 24.04.
# Needs sudo (asks for your password). Gazebo Harmonic itself comes from the
# ROS 2 Jazzy vendor packages that are already installed.
#
#   bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/install_deps.sh
set -euo pipefail

SIM_DIR="${SIM_DIR:-$HOME/vedc_sim}"

sudo apt-get update
# asv_wave_sim (waves + hull hydrodynamics), ardupilot_gazebo (JSON link, camera stream)
sudo apt-get install -y \
    libcgal-dev libfftw3-dev rapidjson-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    python3-colcon-common-extensions

# ArduPilot SITL toolchain: apt packages, ~/venv-ardupilot with MAVProxy and
# pymavlink, and PATH lines in ~/.profile. Upstream script, unchanged.
if [ ! -d "$SIM_DIR/ardupilot" ]; then
    echo "Clone ArduPilot into $SIM_DIR/ardupilot first." >&2
    exit 1
fi
cd "$SIM_DIR/ardupilot"
Tools/environment_install/install-prereqs-ubuntu.sh -y

echo
echo "Done. Close this terminal and open a new one so ~/.profile is reloaded."
