#!/bin/bash
# Bluetooth for manual control from the phone (modules/bt_manual.py): the Pi offers a Serial Port
# (SPP) on RFCOMM channel 1, which Android apps such as Serial Bluetooth Terminal look for.
# Every step checks first and skips what is done, so it can run at every run_all.bat.
#
#   bash tools/setup_bluetooth.sh          install: bluez/sdptool, bluetoothd --compat, adapter on,
#                                          SPP record (run_all.bat step 5)
#   bash tools/setup_bluetooth.sh sdp      the SPP record only; lost whenever bluetoothd restarts, so
#                                          the vedc-boat service runs this before main.py at every boot
#   bash tools/setup_bluetooth.sh pair     the Pi visible for 120 s: pair the phone from its Bluetooth
#                                          settings (once per phone), then the phone is trusted
#
# Needs sudo (no password on the boat's Pi). Only the STM32 boat uses it (.boat_profile stm32).
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ACTION="${1:-install}"
CHANNEL="${CHANNEL:-1}"            # manual.channel in config/settings.yaml
DROPIN=/etc/systemd/system/bluetooth.service.d/compat.conf
SUDO=""; [ "$(id -u)" -eq 0 ] || SUDO="sudo"

stm32_boat() {
    [ "$(tr -d '[:space:]' < "$DIR/.boat_profile" 2>/dev/null || echo stm32)" = "stm32" ]
}

ensure_sdptool() {
    command -v sdptool > /dev/null && return
    echo "bluetooth: installing bluez (sdptool)"
    $SUDO apt-get install -y bluez > /dev/null
    command -v sdptool > /dev/null || { echo "bluetooth: no sdptool after installing bluez" >&2; exit 1; }
}

ensure_compat() {
    # sdptool talks to bluetoothd over the old SDP socket, which only --compat opens.
    local daemon="" path want
    # Not ls a b | head: one missing path fails ls, and pipefail ended the script without a word.
    for path in /usr/libexec/bluetooth/bluetoothd /usr/lib/bluetooth/bluetoothd; do
        if [ -x "$path" ]; then daemon="$path"; break; fi
    done
    [ -n "$daemon" ] || { echo "bluetooth: bluetoothd not found" >&2; exit 1; }
    want="$(printf '[Service]\nExecStart=\nExecStart=%s --compat\n' "$daemon")"
    if [ "$(cat "$DROPIN" 2>/dev/null)" = "$want" ]; then
        echo "bluetooth: bluetoothd --compat already set"
        return
    fi
    echo "bluetooth: bluetoothd --compat (restarting bluetooth)"
    $SUDO mkdir -p "$(dirname "$DROPIN")"
    printf '%s\n' "$want" | $SUDO tee "$DROPIN" > /dev/null
    $SUDO systemctl daemon-reload
    $SUDO systemctl restart bluetooth
    sleep 2
}

adapter_on() {
    $SUDO rfkill unblock bluetooth 2>/dev/null || true
    bluetoothctl power on > /dev/null 2>&1 || true
}

ensure_sdp() {
    # The record is lost at every bluetoothd restart; added twice, the phone sees two ports.
    local i
    for i in 1 2 3 4 5; do       # at boot bluetoothd may still be starting
        if $SUDO sdptool browse local 2>/dev/null | grep -q "Serial Port"; then
            echo "bluetooth: SPP record already there"
            return
        fi
        if $SUDO sdptool add --channel="$CHANNEL" SP > /dev/null 2>&1; then
            echo "bluetooth: SPP record added on RFCOMM channel $CHANNEL"
            return
        fi
        sleep 2
    done
    echo "bluetooth: could not add the SPP record (bluetoothd running with --compat?)" >&2
    exit 1
}

pair() {
    adapter_on
    echo "bluetooth: '$(hostname)' visible for 120 s: pair it from the phone's Bluetooth settings"
    bluetoothctl pairable on > /dev/null
    bluetoothctl discoverable-timeout 120 > /dev/null
    bluetoothctl discoverable on > /dev/null
    # An agent that accepts the pairing. bluetoothctl --agent fed through a pipe accepted nothing
    # (2026-11-17, BlueZ 5.82: the phone saw pi5 but never paired); bt-agent is a real one.
    command -v bt-agent > /dev/null || $SUDO apt-get install -y bluez-tools > /dev/null
    timeout 120 bt-agent -c NoInputNoOutput || true
    bluetoothctl discoverable off > /dev/null
    # Trusted: the phone reconnects later without asking. "devices Paired" from BlueZ 5.65 on.
    local devices
    devices="$(bluetoothctl devices Paired 2>/dev/null || bluetoothctl paired-devices 2>/dev/null || true)"
    if [ -z "$devices" ]; then
        echo "bluetooth: no phone paired; run it again" >&2
        exit 1
    fi
    echo "$devices" | awk '/^Device/ {print $2}' | while read -r mac; do
        bluetoothctl trust "$mac" > /dev/null && echo "bluetooth: trusted $(echo "$devices" | grep "$mac")"
    done
}

case "$ACTION" in
    install)
        stm32_boat || { echo "bluetooth: not the STM32 boat, skipped"; exit 0; }
        ensure_sdptool
        ensure_compat
        adapter_on
        ensure_sdp ;;
    sdp)
        stm32_boat || exit 0
        adapter_on
        ensure_sdp ;;
    pair)
        pair ;;
    -h|--help)
        sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' ;;
    *)
        echo "unknown option '$ACTION'" >&2; exit 2 ;;
esac
