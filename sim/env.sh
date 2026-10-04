# Environment for the boat simulator. Source it, do not run it:
#   source /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/env.sh

SIM_DIR="${SIM_DIR:-$HOME/vedc_sim}"
VEDC_SIM_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SIM_DIR VEDC_SIM_REPO

# Gazebo Harmonic comes from the ROS 2 Jazzy vendor packages.
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
export GZ_IP=127.0.0.1

# WSLg renders on the CPU (llvmpipe) by default, far too slow for waves.
# d3d12 hands OpenGL to the Windows GPU driver.
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

# ArduPilot SITL tools (sim_vehicle.py, MAVProxy) from install-prereqs.
if [ -f "$HOME/venv-ardupilot/bin/activate" ]; then source "$HOME/venv-ardupilot/bin/activate"; fi
export PATH="$SIM_DIR/ardupilot/Tools/autotest:$PATH"

# Models and worlds: ours first so they win over upstream ones of the same name.
export GZ_SIM_RESOURCE_PATH="$VEDC_SIM_REPO/models:$VEDC_SIM_REPO/worlds:\
$SIM_DIR/ardupilot_gazebo/models:$SIM_DIR/ardupilot_gazebo/worlds:\
$SIM_DIR/SITL_Models/Gazebo/models:$SIM_DIR/SITL_Models/Gazebo/worlds:\
$SIM_DIR/asv_wave_sim/gz-waves-models/models:\
$SIM_DIR/asv_wave_sim/gz-waves-models/world_models:\
$SIM_DIR/asv_wave_sim/gz-waves-models/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# System plugins: ArduPilot JSON link and the wave / hydrodynamics plugins.
export GZ_SIM_SYSTEM_PLUGIN_PATH="$SIM_DIR/ardupilot_gazebo/build:\
$SIM_DIR/wave_ws/install/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_GUI_PLUGIN_PATH="$SIM_DIR/asv_wave_sim/gz-waves/src/gui/plugins/waves_control/build${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"
if [ -f "$SIM_DIR/wave_ws/install/setup.bash" ]; then source "$SIM_DIR/wave_ws/install/setup.bash"; fi
