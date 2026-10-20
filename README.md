# VEDC2026-Edge-AI-Autonomous-Rescue-Buoy-System

Phao cứu hộ tự hành RedgeSCUE. Repo chứa **hai kiến trúc** chạy song song, dùng chung code nhận
thức trên Raspberry Pi:

| Tên | Phần cứng | Dùng cho | Chương trình Pi | Firmware |
|---|---|---|---|---|
| `pixhawk` | Raspberry Pi + Pixhawk (ArduPilot Rover) + STM32F103 | giả lập, phao thật, video demo | `Rasp_Pi/rescue_main.py` | `STM32F103/` |
| `stm32` | Raspberry Pi + STM32F407 | trình diễn trên cạn | `Rasp_Pi/main.py` | `STM32F4/` |

```
pixhawk:  camera + LiDAR ─► Pi ─ MAVLink ─► Pixhawk ─► motor, bánh lái
                            └─ USB-TTL ─► STM32F103 ─► servo camera, relay, LoRa ─► trạm bờ
stm32:    camera + LiDAR ─► Pi ─ $NAV/$HBT ─► STM32F407 ─► motor, bánh lái
                                                  └─ LoRa ─► trạm bờ ESP32 ─► dashboard
```

## Chọn kiến trúc bằng một tên

Từ thư mục gốc, trên Windows:

```
boat <pixhawk|stm32> <build|flash|deploy|run|test> [tham số]
```

| Lệnh | Việc |
|---|---|
| `boat pixhawk build` | build firmware STM32F103 (`boat stm32 build` thì STM32F4); thêm `Release`, `clean` như `build.bat` |
| `boat pixhawk flash` | nạp firmware đó qua ST-LINK |
| `boat pixhawk deploy` | chép `Rasp_Pi/` lên Pi qua SSH và ghi lại phao đang chọn; từ đó trên Pi chỉ cần `./run.sh` |
| `boat pixhawk run --viewer` | chạy chương trình Pi của phao đó qua SSH; tham số phía sau chuyển cho `run.sh` |
| `boat pixhawk test` | unit test của Pi và test firmware của phao đó trên máy tính; `boat pixhawk test sim` chạy thêm kịch bản cứu hộ trong giả lập (WSL, vài phút) |

`boat` chỉ gọi lại script của từng thư mục: vào `STM32F103\` hay `STM32F4\` chạy `build.bat`,
`flash.bat` vẫn như cũ.

Địa chỉ Pi: chép `boat.local.example.cmd` thành `boat.local.cmd` (không commit) rồi sửa `PI_HOST`,
`PI_DIR`. Nên cài khóa SSH để không phải gõ mật khẩu mỗi lần.

Đổi kiến trúc trên cùng một Pi: nối dây đúng bộ điều khiển (cả hai dùng `/dev/serial0`), rồi
`boat <tên> deploy`.

## Phao `stm32`: thử từng chức năng và chạy toàn bộ

`scripts\stm32\test_*.bat` thử từng chức năng (IMU, pin, LoRa, trạm, dashboard, Pi ↔ F407, loa, camera, LiDAR,
đèn, motor), `scripts\stm32\test_host.bat` mọi test trên máy tính, `scripts\stm32\test_sim.bat` cả nhiệm vụ trong
giả lập, `scripts\stm32\run_all.bat` nạp và chạy cả hệ thống. Máy trạng thái của phao, bảng chức năng → file
test: [docs/stm32-system.md](docs/stm32-system.md).

## Từ giả lập lên phao thật

- `sim/run_boat.sh --pi --hw`: giả lập đi qua đúng đường phần cứng của phao `pixhawk` (LiDAR qua cổng nối
  tiếp, bo STM32F103 chạy chính logic firmware). Xem `sim/README.md`.
- `docs/porting-checklist.md`: thứ tự đấu dây, nạp, và giá trị phải thấy ở từng bước.
- `pixhawk/params/boat.parm`: tham số ArduPilot khởi đầu cho phao thật.

## Thư mục

| Thư mục | Nội dung |
|---|---|
| `Rasp_Pi/` | code Pi của cả hai kiến trúc; cấu hình chung `config/settings.yaml`, phần riêng `config/boat_<tên>.yaml` |
| `STM32F4/` | firmware STM32F407 (kiến trúc `stm32`) |
| `STM32F103/` | firmware STM32F103 (kiến trúc `pixhawk`) |
| `sim/` | giả lập Gazebo + ArduPilot SITL cho kiến trúc `pixhawk` |
| `esp32-lora-station/` | trạm bờ ESP32, Cloudflare Worker, dashboard, cầu nối MAVLink |
| `pixhawk/` | tham số ArduPilot của phao thật |
| `docs/` | danh sách kiểm tra khi đưa lên phần cứng |
| `papers/` | bài báo nguồn của các thuật toán né vật cản |
