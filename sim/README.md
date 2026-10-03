# Giả lập 3D phao cứu hộ — ArduPilot + Gazebo + Mission Planner

Phao chạy trong **Gazebo Harmonic** (mặt nước có sóng, lực nổi, chân vịt), được điều khiển bởi
**ArduPilot Rover 4.7.1** chạy giả lập (SITL), và **Mission Planner** trên Windows nối vào qua MAVLink —
giống hệt khi nối với phao thật.

```
Windows                         WSL Ubuntu 24.04
┌────────────────┐  UDP 14550   ┌──────────┐  TCP 5760  ┌────────────┐  UDP 9002   ┌──────────────────┐
│ Mission Planner│◄────────────►│ MAVProxy │◄──────────►│ ardurover  │◄───────────►│ Gazebo           │
└────────────────┘  (MAVLink)   └──────────┘            │ (SITL)     │ (lock-step) │ sóng + vedc_buoy │
                                                        └────────────┘             │ camera + LiDAR   │
                                                                                   └──────────────────┘
```

Mã nguồn bên thứ ba nằm ở `~/vedc_sim` trong WSL (không đưa vào repo): `ardupilot`, `ardupilot_gazebo`,
`SITL_Models` (mô hình BlueBoat), `asv_wave_sim` (sóng và thủy động lực học).

## Cài đặt (một lần)

```bash
wsl -d Ubuntu-24.04 -- bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/install_deps.sh
wsl -d Ubuntu-24.04 -- bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/build.sh
```

- `install_deps.sh` cần sudo: CGAL, FFTW, GStreamer, colcon và script cài đặt chính thức của ArduPilot
  (tạo `~/venv-ardupilot` có MAVProxy, pymavlink).
- `build.sh` không cần sudo, khoảng 10 phút: ArduPilot SITL, plugin Gazebo, plugin sóng.
  Build lại một phần: `build.sh ardupilot | gazebo | waves`.
- Gazebo Harmonic có sẵn từ ROS 2 Jazzy, không cài riêng.
- `.wslconfig` phải có `networkingMode=mirrored` để Windows thấy `127.0.0.1` của WSL.

## Chạy

```bash
wsl -d Ubuntu-24.04 -- bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/run_boat.sh
```

Mở cửa sổ Gazebo 3D, một cửa sổ xterm của ArduPilot, và dấu nhắc `MAV>` của MAVProxy trong terminal.
`Ctrl+C` ở terminal để tắt tất cả.

| Tùy chọn | Tác dụng |
|---|---|
| `--headless` | Gazebo không mở cửa sổ (máy yếu, hoặc kiểm thử tự động) |
| `--wipe` | Xóa tham số đã lưu, nạp lại mặc định + `params/vedc_buoy.parm` |
| `HOME_LOCATION=lat,lon,alt,hướng` | Đổi điểm xuất phát. Mặc định giữa **Hồ đá 01, Làng Đại học ĐHQG-HCM** `10.883438,106.796019,10,90` (cách bờ 217 m, phía đông trống khoảng 265 m) |

**Mission Planner:** góc trên phải chọn **UDP** → **Connect** → cổng **14550**.

## Thử nhanh trong Mission Planner

1. Tab **DATA**, đợi HUD hết báo lỗi EKF (khoảng 20–30 giây sau khi khởi động)
2. **Actions** → **Arm/Disarm**
3. Chuột phải lên bản đồ → **Fly to Here** (chế độ GUIDED) — phao chạy tới đó, xem trong Gazebo
4. Tab **PLAN**: click để thêm waypoint → **Write WPs** → về DATA, đổi chế độ sang **Auto**
5. Đổi sang **RTL** — phao quay về điểm xuất phát
6. Thử failsafe mất liên lạc: khi phao đang chạy, ngắt Mission Planner (**Disconnect**) và gõ
   `set heartbeat 0` ở dấu nhắc `MAV>` (MAVProxy cũng phát heartbeat GCS). Sau 5 giây phao tự RTL.
   Gõ `set heartbeat 1` để bật lại.

## Xem trên dashboard web

`run_boat.sh` cho MAVProxy phát thêm ra cổng **14551**. Cầu nối
`esp32-lora-station/tools/mavlink_bridge.py` đọc cổng đó và đẩy lên Worker như một phao `SIM-01`, có HUD
kiểu Mission Planner. Chạy Worker cục bộ (`npx wrangler dev` trong `esp32-lora-station/cloud`), rồi trong WSL:

```bash
export VEDC_INGEST_TOKEN=...   # INGEST_TOKEN trong esp32-lora-station/cloud/.dev.vars
python3 /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/esp32-lora-station/tools/mavlink_bridge.py --url http://127.0.0.1:8787/api/ingest
```

Mở `http://127.0.0.1:8787`, vào bằng chế độ khách, chọn `SIM-01`. Xem `esp32-lora-station/README.md`.

Kiểm thử tự động (không cần Mission Planner): chạy `--headless`, rồi trong WSL
`python3 sim/tools/smoke_test.py` — arm, GUIDED 20 m về phía đông, báo PASS/FAIL.

## Thư mục

| File | Nội dung |
|---|---|
| `env.sh` | Biến môi trường: ROS/Gazebo, GPU qua d3d12, đường dẫn mô hình và plugin |
| `models/vedc_buoy/` | Catamaran 1.10 × 0.60 m (hình hộp, chờ file CAD): 2 motor sau trên pod lái, 1 motor trước ở giữa đảo chiều được, pod trước quay ngược pod sau như firmware STM32. Cột cảm biến: camera 100° (`/vedc_buoy/camera`), LiDAR 360° kiểu LD14 (`/vedc_buoy/lidar`) |
| `worlds/vedc_lake.sdf` | Mặt nước sóng FFT, ba phao tiêu làm vật cản, bước vật lý 2 ms |
| `models/vedc_waves/` | Sóng của asv_wave_sim trên lưới 128 ô (gốc 256) để chạy kịp thời gian thực; texture lấy từ `model://waves` |
| `params/vedc_buoy.parm` | `FRAME_CLASS 2` (tàu), gán kênh motor/pod, tốc độ, failsafe mất GCS → RTL |
| `tools/smoke_test.py` | Kiểm thử nhanh qua pymavlink: arm, GUIDED 20 m |
| `tools/integration_test.sh` | Kiểm thử đầy đủ trên instance 1 (cổng 9012/5770/14561), chạy song song được với phiên đang mở: tham số, nhiễu IMU, arm, GUIDED, chiều kênh khi rẽ |

### Ánh xạ kênh ArduPilot → phao

| SERVO | Chức năng | Trên phao | Firmware STM32 tương ứng |
|---|---|---|---|
| 1 | ThrottleLeft (73) | motor sau trái | `ESC_LEFT`, DShot PA5 |
| 2 | GroundSteering (26) | hai pod sau | `SERVO_LEFT` / `SERVO_RIGHT` |
| 3 | ThrottleRight (74) | motor sau phải | `ESC_RIGHT`, DShot PA3 |
| 4 | Throttle (70) | motor trước, 1500 µs = dừng | `ESC_FRONT`, PWM PA1 |
| 5 | GroundSteering, đảo chiều | pod trước | `SERVO_FRONT`, `RUDDER_DIR_FRONT = -1` |

Rẽ phải: motor trái mạnh hơn phải, pod sau > 1500 µs, pod trước < 1500 µs.

### Thay mô hình hình hộp bằng file CAD

Gazebo đọc `.stl`, `.obj`, `.dae`, `.glb`. Xuất từ SolidWorks/Fusion/Onshape:
- **đơn vị mét**, gốc tọa độ ở tâm phao tại mặt nước, trục x hướng mũi, y sang trái, z lên;
- tách riêng thân (2 thân + sàn), từng pod, từng chân vịt;
- một bản **collision đơn giản** cho thân (vài trăm tam giác) — lực nổi tính trên từng tam giác
  mỗi bước 2 ms, mesh chi tiết sẽ làm giả lập chậm hẳn.

### Những số đang giả định

Khối lượng 8 kg, thân 0.14 × 0.20 m, lực đẩy ±15 N mỗi motor sau và ±10 N motor trước, pod ±45° trên
1000–2000 µs. Thay bằng số đo thật để giả lập sát phao.

Hai chỗ khác phao thật: motor sau thật (BLHeli_S DShot) chỉ quay một chiều, ở đây đảo chiều được;
quán tính chân vịt để 0.1 kg·m² (lớn hơn thật nhiều) như mô hình BlueBoat, vì link quay nhẹ làm
bộ giải vật lý rung tới ±50 g và ArduPilot từ chối arm ("Accels inconsistent").

## Lưu ý

- **Tốc độ giả lập:** góc dưới phải cửa sổ Gazebo là real-time factor (RTF). ArduPilot chạy khóa nhịp
  với Gazebo nên RTF thấp thì phao phản ứng chậm theo. Đo trên laptop dev (headless): 0.45 với bước
  1 ms + lưới sóng 256 + barrage; khoảng 1.0 với cấu hình hiện tại. Chi phí chính là lực nổi tính trên
  từng tam giác thân tàu mỗi bước — mỗi vật nổi thêm vào thế giới đều làm chậm. Camera/LiDAR gần như
  không ảnh hưởng.
- **GPU:** WSLg mặc định render bằng CPU (llvmpipe), sóng sẽ rất giật. `env.sh` đặt
  `GALLIUM_DRIVER=d3d12` để dùng card NVIDIA. Kiểm tra: `glxinfo -B` phải ra `D3D12 (NVIDIA ...)`.
- **Sóng:** gió 2.5 m/s (sóng khoảng 0.13 m), hợp với hồ kín như Hồ Đá. Gió 5 m/s của bản gốc là sóng
  biển khoảng 0.5 m, quá lớn cho phao 1.1 m.
- **Test song song:** đừng chạy thêm một Gazebo có `ArduPilotPlugin` cổng 9002 khi `run_boat.sh` đang
  mở — ArduPilot sẽ nói chuyện nhầm sang đó. Dùng `tools/integration_test.sh` (instance 1, cổng riêng).
- Khi tắt Gazebo có thể thấy `OGRE EXCEPTION ... mHlmsGlobalIndex` — lỗi lúc hủy cảnh của Gazebo,
  vô hại.
