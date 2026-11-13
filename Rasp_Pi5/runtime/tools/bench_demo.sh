#!/bin/bash
# Bench demo of the STM32 boat on its stand: (re)starts main.py with the stand settings
# (config/cam_rapoo.yaml + config/bench_demo.yaml) and shows its viewer full screen on the HDMI
# screen. run_normal.bat on the PC calls it over SSH.
#   tools/bench_demo.sh          log: /tmp/vedc_demo.log
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."

if pkill -INT -f "main.py --overlay"; then sleep 3; fi
setsid nohup ./run.sh stm32 --overlay config/cam_rapoo.yaml --overlay config/bench_demo.yaml --viewer \
    > /tmp/vedc_demo.log 2>&1 < /dev/null &
for _ in $(seq 30); do
    grep -q "viewer: http" /tmp/vedc_demo.log 2>/dev/null && break
    sleep 1
done
grep -E "UART open|viewer: http" /tmp/vedc_demo.log

# The desktop session's display, from an SSH login too. --password-store=basic: no keyring prompt.
export XDG_RUNTIME_DIR="/run/user/$(id -u)" WAYLAND_DISPLAY=wayland-0
pkill -f "chromium.*vedc_kiosk" && sleep 2
setsid nohup chromium --kiosk --noerrdialogs --disable-infobars --no-first-run --password-store=basic \
    --ozone-platform=wayland --force-device-scale-factor=0.95 --user-data-dir=/tmp/vedc_kiosk \
    http://127.0.0.1:8090/stream > /tmp/vedc_kiosk.log 2>&1 < /dev/null &
sleep 3
if pgrep -f "chromium.*vedc_kiosk" > /dev/null; then
    echo "viewer on the HDMI screen"
else
    echo "the viewer did not open on the HDMI screen: see /tmp/vedc_kiosk.log" >&2
fi
