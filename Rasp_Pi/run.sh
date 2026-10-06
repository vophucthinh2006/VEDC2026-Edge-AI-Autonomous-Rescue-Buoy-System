#!/bin/bash
# Start the Pi runtime for one of the two boat controllers. Both use the same Pi UART
# (/dev/serial0), so only the one that is actually wired can run.
#
#   ./run.sh pixhawk [options of rescue_main.py]   Pixhawk + ArduPilot Rover, MAVLink on TELEM2
#   ./run.sh stm32   [options of main.py]          STM32F407, $NAV/$HBT/$CAM packets on UART4
#
#   ./run.sh pixhawk --viewer
#   ./run.sh stm32 --no-lidar
#
# The simulator is started with sim/run_boat.sh --pi, not from here.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

usage() { sed -n '2,11p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

case "${1:-}" in
    pixhawk) PROGRAM=rescue_main.py; KEYS="mavlink url baud" ;;
    stm32)   PROGRAM=main.py;        KEYS="serial stm32_port stm32_baud" ;;
    -h|--help|"") usage; exit 0 ;;
    *) echo "unknown controller '$1'" >&2; usage >&2; exit 2 ;;
esac
CONTROLLER="$1"
shift

if [ -f .venv/bin/activate ]; then
    # shellcheck disable=SC1091
    . .venv/bin/activate
fi

# Port and baud of the default config, to show them and to catch a missing UART early.
# A --config or --overlay given on the command line may point elsewhere; the program then decides.
read -r PORT BAUD < <(python3 - $KEYS <<'EOF'
import sys, yaml
section, port, baud = sys.argv[1:4]
config = yaml.safe_load(open("config/settings.yaml", encoding="utf-8"))[section]
print(config[port], config[baud])
EOF
)
echo "controller: $CONTROLLER ($PROGRAM), link: $PORT @ $BAUD"
case " $* " in
    *" --config "*|*" --overlay "*|*" --mavlink "*) ;;
    *) if [[ "$PORT" == /dev/* && ! -e "$PORT" ]]; then
           echo "$PORT does not exist: enable the UART (enable_uart=1) and check the wiring" >&2
           exit 1
       fi ;;
esac

exec python3 -u "$PROGRAM" "$@"
