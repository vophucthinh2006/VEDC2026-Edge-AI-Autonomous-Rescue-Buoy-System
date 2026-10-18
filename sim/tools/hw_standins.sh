# Sourced by run_boat.sh --hw and tools/hw_path_test.sh, under sim/env.sh: starts the stand-ins
# for the boat's LiDAR and helper board in a run directory and writes the overlay with their ports.
#
#   start_hw_standins <run dir> <Gazebo LiDAR topic> <Gazebo camera pan topic>
#
# Sets HW_PIDS (both), LIDAR_SIM_PID and HELPER_SIM_PID.
start_hw_standins() {
    local run="$1" lidar_topic="$2" pan_topic="$3"
    local firmware="$VEDC_SIM_REPO/../STM32F103"
    # The helper board's logic, compiled from the firmware sources for this machine.
    gcc -std=c11 -O1 -Wall -D_POSIX_C_SOURCE=200809L -DGPIO_PIN_SET=1 -DGPIO_PIN_RESET=0 \
        -I"$firmware/App" -I"$firmware/Modules/PiLink" \
        "$firmware/Tests/helper_sim.c" "$firmware/App/helper_logic.c" "$firmware/App/lora_frame.c" \
        "$firmware/Modules/PiLink/pi_link.c" -o "$run/helper_sim" || return 1
    local ingest=()
    if [ -n "$VEDC_INGEST_TOKEN" ]; then
        ingest=(--ingest-url "${VEDC_INGEST_URL:-http://127.0.0.1:8787/api/ingest}")
    fi
    : > "$run/helper_events.log"
    # The boat's LiDAR is the LD14; VEDC_LIDAR_MODEL=lds008 brings back the LDS-008 stand-in.
    local lidar_model="${VEDC_LIDAR_MODEL:-ld14}"
    python3 -u "$VEDC_SIM_REPO/tools/${lidar_model}_emulator.py" --pty "$run/lidar_pty" --topic "$lidar_topic" \
        > "$run/lidar_sim.log" 2>&1 &
    LIDAR_SIM_PID=$!
    python3 -u "$VEDC_SIM_REPO/tools/helper_board_sim.py" --pty "$run/helper_pty" --binary "$run/helper_sim" --dir "$run" \
        --pan-topic "$pan_topic" "${ingest[@]}" > "$run/helper_sim.log" 2>&1 &
    HELPER_SIM_PID=$!
    HW_PIDS="$LIDAR_SIM_PID $HELPER_SIM_PID"
    printf 'serial:\n  lidar_port: %s\n  lidar_model: %s\nhelper:\n  port: %s\n' "$run/lidar_pty" "$lidar_model" "$run/helper_pty" > "$run/hw_ports.yaml"
    # Both must have made their pseudo-terminal before the Pi code looks for it.
    for _ in $(seq 20); do
        [ -e "$run/lidar_pty" ] && [ -e "$run/helper_pty" ] && return 0
        sleep 0.5
    done
    echo "hardware stand-ins did not start: see $run/lidar_sim.log and $run/helper_sim.log" >&2
    return 1
}
