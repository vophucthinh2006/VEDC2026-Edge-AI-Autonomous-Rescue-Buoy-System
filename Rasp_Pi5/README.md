# RedgeSCUE for Raspberry Pi 5

This directory is a Raspberry Pi 5 deployment layout placed alongside
`Github`. It uses only Raspberry Pi + STM32F407. The model weights are at `YOLO_Movenet/weights`,
where `runtime/config/settings.yaml` expects them.

There are two launchers:

- `python3 run_system.py [main.py options]` runs the complete system: buoy,
  STM32F407, shore station, and web/dashboard protocol.
- `python3 run_buoy_test.py [main.py options]` runs a buoy-only test: camera
  detection/processing plus `$CAM` for the servo and `$NAV` for steering and
  motors via the STM32F407. It does not require a shore route or LiDAR.

Any `--overlay` supplied after the launcher overrides its Pi 5 defaults.  For
example, a USB-to-TTL device path may be supplied without changing a source
file:

```bash
python3 run_system.py --overlay config/my_buoy_ports.yaml --viewer
python3 run_buoy_test.py --overlay config/my_buoy_ports.yaml --viewer
```

## Pi 5 serial preparation

On Raspberry Pi 5, `/dev/serial0` normally points to the dedicated debug UART,
not to GPIO14/15.  The launchers therefore use `/dev/ttyAMA0` directly for the
Pi-to-controller UART.  In Raspberry Pi OS, use `raspi-config` to enable the
serial hardware and disable the serial login shell, then reboot.  Verify that
the controller is wired cross-over (Pi TX/GPIO14 to device RX, Pi RX/GPIO15 to
device TX, and a common ground).

The complete-system launcher expects the LiDAR on `/dev/ttyAMA1`. For a USB
LiDAR, add its stable `/dev/serial/by-id/...` path to a later overlay:

```yaml
serial:
  lidar_port: /dev/serial/by-id/<your-adapter>
```

Install the dependencies from `runtime/requirements.txt` in a virtual
environment before starting either launcher.
