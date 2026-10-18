# Sourced by run_boat.sh --hw / --stm32 and tools/hw_path_test.sh, under sim/env.sh: starts the
# stand-ins for the boat's LiDAR and helper board (Pixhawk boat) or STM32F407 board (STM32 boat)
# in a run directory and writes the overlay with their ports.
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

# The Pi + STM32F407 boat (Rasp_Pi/main.py): the F407 board and the LiDAR (LD14, or VEDC_LIDAR_MODEL), each behind a
# pseudo-terminal, the overlay with their ports, and the ESP32 shore station on the simulated LoRa
# link (tools/lora_radio.py), which hands the board the route an admin sends from the dashboard
# (docs/lora-route-downlink.md). The station reads that route from the Worker with
# VEDC_INGEST_TOKEN set (VEDC_CLOUD_URL, default http://127.0.0.1:8787), or from the YAML file
# VEDC_ROUTE_FILE.
#
#   start_stm32_standins <run dir> <home lat,lon> <Gazebo world name> [<east,north,heading>]
#
# Sets HW_PIDS (all three), LIDAR_SIM_PID, STM32_SIM_PID and STATION_SIM_PID.
start_stm32_standins() {
    local run="$1" home="$2" world="$3" start="$4"
    local start_args=() station_args=()
    local repo="$VEDC_SIM_REPO/.."
    [ -n "$start" ] && start_args=(--start "$start")
    # The route code of both firmwares, compiled for this machine and run through ctypes.
    gcc -std=c11 -O1 -Wall -shared -fPIC -I"$repo/STM32F4/App" -I"$repo/esp32-lora-station/main" \
        "$VEDC_SIM_REPO/tools/route_radio_shim.c" "$repo/STM32F4/App/route_link.c" \
        "$repo/esp32-lora-station/main/route_downlink.c" -lm -o "$run/librouteradio.so" || return 1
    if [ -n "$VEDC_ROUTE_FILE" ]; then
        station_args=(--route-file "$VEDC_ROUTE_FILE")
    elif [ -n "$VEDC_INGEST_TOKEN" ]; then
        station_args=(--cloud "${VEDC_CLOUD_URL:-http://127.0.0.1:8787}")
    fi
    : > "$run/stm32_events.log"
    local lidar_model="${VEDC_LIDAR_MODEL:-ld14}"
    python3 -u "$VEDC_SIM_REPO/tools/${lidar_model}_emulator.py" --pty "$run/lidar_pty" --topic /vedc_buoy/lidar \
        > "$run/lidar_sim.log" 2>&1 &
    LIDAR_SIM_PID=$!
    python3 -u "$VEDC_SIM_REPO/tools/shore_station_sim.py" --radio-lib "$run/librouteradio.so" "${station_args[@]}" \
        > "$run/station_sim.log" 2>&1 &
    STATION_SIM_PID=$!
    python3 -u "$VEDC_SIM_REPO/tools/stm32_board_sim.py" --pty "$run/stm32_pty" --dir "$run" --radio-lib "$run/librouteradio.so" \
        --home "$home" --world "$world" "${start_args[@]}" ${STM32_SIM_ARGS:-} > "$run/stm32_sim.log" 2>&1 &
    STM32_SIM_PID=$!
    HW_PIDS="$LIDAR_SIM_PID $STATION_SIM_PID $STM32_SIM_PID"
    printf 'serial:\n  stm32_port: %s\n  lidar_port: %s\n  lidar_model: %s\n' "$run/stm32_pty" "$run/lidar_pty" "$lidar_model" > "$run/stm32_ports.yaml"
    for _ in $(seq 30); do
        [ -e "$run/lidar_pty" ] && [ -e "$run/stm32_pty" ] && return 0
        sleep 0.5
    done
    echo "stand-ins did not start: see $run/lidar_sim.log, $run/stm32_sim.log and $run/station_sim.log" >&2
    return 1
}
