#!/usr/bin/env bash
# Pi 5 on the boat: keep it cool and bring it back by itself when it hangs. Run once with sudo, then reboot.
#   sudo bash tools/setup_pi5_cooling.sh
# - Active Cooler: on from 40 C and full speed from 65 C. The firmware's own curve starts at 50 C and
#   steps up only at 60/67.5/75 C, so the fan stopped and started and the Pi ran hot under the detector.
# - Hardware watchdog: a hung Pi (2026-11-22: answered ping, not ssh; only a power cycle helped) reboots
#   after 15 s, and vedc-boat.service starts the boat program again.
# - Journal kept on disk, so the log before a hang can be read after the reboot (journalctl -b -1).
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "run with sudo"; exit 1; }

CONFIG=/boot/firmware/config.txt
if ! grep -q "^dtparam=fan_temp0=" "$CONFIG"; then
    cat >> "$CONFIG" <<'EOF'

[pi5]
# VEDC boat (tools/setup_pi5_cooling.sh): the Active Cooler from 40 C, full speed from 65 C
dtparam=fan_temp0=40000,fan_temp0_hyst=3000,fan_temp0_speed=100
dtparam=fan_temp1=50000,fan_temp1_hyst=3000,fan_temp1_speed=160
dtparam=fan_temp2=58000,fan_temp2_hyst=3000,fan_temp2_speed=210
dtparam=fan_temp3=65000,fan_temp3_hyst=3000,fan_temp3_speed=255
[all]
EOF
    echo "fan curve added to $CONFIG"
fi

mkdir -p /etc/systemd/system.conf.d
printf '[Manager]\nRuntimeWatchdogSec=15\nRebootWatchdogSec=2min\n' > /etc/systemd/system.conf.d/vedc-watchdog.conf
systemctl daemon-reexec
echo "watchdog: $(systemctl show -p RuntimeWatchdogUSec --value)"

mkdir -p /var/log/journal
systemd-tmpfiles --create --prefix /var/log/journal
systemctl restart systemd-journald
echo "journal kept on disk; reboot for the fan curve"
