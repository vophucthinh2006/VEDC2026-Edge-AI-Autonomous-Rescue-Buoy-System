#!/bin/bash
# The boat's Pi program (run.sh: main.py for the STM32 boat, after .boat_profile) as the systemd
# service vedc-boat: it starts when the Pi boots and again 3 s after it stops for any reason (a
# crash, a lost port at boot, killed). The Pi rebooting on a pack swap no longer leaves the boat
# without its program until somebody logs in. Without main.py the F407 stops the motors and keeps
# the route; once main.py is back it hands the route over again, and the boat drives on.
#
#   bash tools/install_service.sh              install or refresh the unit, enable it at boot
#   bash tools/install_service.sh --restart    the same, then (re)start it now (run_all.bat step 5)
#   ... --restart --overlay config/stand_demo.yaml    options after the action go to run.sh (run_all.bat
#                                              stand: the stand demo; --viewer: the viewer on port
#                                              8090); kept at every boot until the next install or
#                                              --restart without them. None: no viewer, so every
#                                              camera frame goes to the detector
#   bash tools/install_service.sh --stop       stop it until the next boot or --restart (the tests
#                                              that need the F407's UART, the LiDAR or the camera)
#   bash tools/install_service.sh --remove     stop, disable and delete it
#
# Its output: journalctl -u vedc-boat -f, and logs/main_*.log as before. Needs sudo (no password on
# the boat's Pi). Mind the motors: started with a route on the F407, the boat drives at once.
set -euo pipefail
SERVICE=vedc-boat
UNIT="/etc/systemd/system/$SERVICE.service"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USER_NAME="$(id -un)"
ACTION="${1:-install}"
shift || true
RUN_ARGS="$*"                 # options for run.sh; none given: no viewer (it costs the detector fps)

stop_manual() {
    # A main.py started by hand (ssh window of an older run_all) would fight the service for the UART.
    pkill -INT -f "[m]ain.py --overlay" 2>/dev/null || true
    pkill -INT -f "[r]escue_main.py --overlay" 2>/dev/null || true
    sleep 2
}

install_unit() {
    chmod +x "$DIR/run.sh"
    sudo tee "$UNIT" > /dev/null <<EOF
[Unit]
Description=VEDC boat program on the Pi (run.sh $RUN_ARGS, controller from .boat_profile)
After=network-online.target sound.target
Wants=network-online.target
# Keep restarting however often it fails: a USB adapter late at boot, the F407 off for a while.
StartLimitIntervalSec=0

[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$DIR
ExecStart=$DIR/run.sh $RUN_ARGS
Restart=always
RestartSec=3
# main.py sends the F407 a STOP on SIGTERM before it exits.
TimeoutStopSec=10
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF
    sudo systemctl daemon-reload
    sudo systemctl enable "$SERVICE" > /dev/null 2>&1
    echo "$SERVICE: installed and enabled at boot ($DIR/run.sh $RUN_ARGS as $USER_NAME)"
}

case "$ACTION" in
    install)
        install_unit
        echo "$SERVICE: not started now; --restart starts it" ;;
    --restart)
        install_unit
        sudo systemctl stop "$SERVICE" 2>/dev/null || true
        stop_manual
        sudo systemctl start "$SERVICE"
        sleep 1
        echo "$SERVICE: $(systemctl is-active "$SERVICE")" ;;
    --stop)
        sudo systemctl stop "$SERVICE" 2>/dev/null || true
        echo "$SERVICE: stopped until the next boot or install_service.sh --restart" ;;
    --remove)
        sudo systemctl disable --now "$SERVICE" 2>/dev/null || true
        sudo rm -f "$UNIT"
        sudo systemctl daemon-reload
        echo "$SERVICE: removed" ;;
    -h|--help)
        sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' ;;
    *)
        echo "unknown option '$ACTION'" >&2; exit 2 ;;
esac
