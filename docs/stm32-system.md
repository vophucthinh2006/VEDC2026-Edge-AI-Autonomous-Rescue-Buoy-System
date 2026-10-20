# Hệ thống phao `stm32` (Raspberry Pi + STM32F407): toàn bộ

Một trang cho cả hệ thống: khâu nào làm gì, phao xử lý thế nào, các khâu nói với nhau ra sao, và **thử từng
chức năng bằng file nào**. Chi tiết từng khâu: [STM32F4/README.md](../STM32F4/README.md),
[Rasp_Pi/README.md](../Rasp_Pi/README.md), [esp32-lora-station/README.md](../esp32-lora-station/README.md),
[lora-route-downlink.md](lora-route-downlink.md).

## 1. Phân vai

```
camera ─► Pi: YOLOv8n + MoveNet ─► mức rủi ro ─┐
LiDAR LD14 ─► Pi: cụm vật cản ─────────────────┼─► máy trạng thái ─► BAPF ─► $NAV hướng + tốc độ ─┐
                                               │   (modules/mission.py)                          │
             loa (mỗi 100 s, nút xác nhận) ◄───┘                                                 ▼
STM32F407: PID hướng (BNO055, mũi = −x) ─► ESC + bánh lái; phát hiện lật; INA226 % pin; nút PE4; 2 đèn
     │ LoRa 433 MHz: beacon 5 s, sự kiện ngay (người / lật / PE4), nghe 1.5 s sau mỗi khung
     ▼
ESP32 trạm bờ: trả lời mọi khung (RTE hoặc ACK + lệnh đèn + GPS trạm) ─► Worker ─► dashboard
```

| Khâu | Làm | Không làm |
|---|---|---|
| **Pi** `Rasp_Pi/main.py` | nhận diện và xếp hạng người, né vật cản, máy trạng thái phao, quét camera ±45°, loa | không điều khiển đèn; không nói LoRa |
| **STM32F407** `STM32F4/` | giữ hướng PID, watchdog (NAV 300 ms, HBT 500 ms, IMU 100 ms), lật → dừng motor + báo trạm, LoRa, pin, PE4, relay đèn | không tự chọn đích: chỉ giữ hướng và tốc độ Pi gửi |
| **ESP32** `esp32-lora-station/main` | cầu nối LoRa ↔ Worker, ACK mọi khung kèm lệnh đèn và vị trí trạm | không quyết định gì |
| **Worker + web** `esp32-lora-station/cloud`, `web` | lưu telemetry, người, cảnh báo, lộ trình, lệnh đèn; vẽ bản đồ | — |

## 2. Phao xử lý thế nào (máy trạng thái trên Pi)

```
            nhận lộ trình (rev mới)                 nút PE4 / mất người lost_s
 IDLE ─────────────────────────────► PATROL ◄──────────────────────────────── FOLLOW
 (đứng yên)                          (waypoint, quét ±45°, né)   thấy người   (bám người ưu tiên cao nhất)
   ▲  lộ trình rỗng / hết lộ trình      │                     ─────────────►      │
   └────────────────────────────────────┤ $LNK shore_ok = 0 (trạm im 60 s)        │
                                        ▼                                         ▼
                                     RETURN ── tới trạm ──► HOME_HOLD (đứng yên)
                                        │ trạm nghe lại → PATROL, tiếp waypoint đang dở
```

- **An toàn đè lên mọi trạng thái:** F407 dừng motor khi lật, IMU cũ, Pi im; Pi gửi STOP khi `attitude_fault`,
  `imu_stale`, `outside_geofence`.
- **Điểm về** là GPS của trạm, đi trong mỗi ACK. Trạm nằm ngoài geofence thì Pi kéo đích vào trong fence. Chưa
  từng có GPS trạm: vị trí phao lúc nhận lộ trình.
- **Đèn** không thuộc máy trạng thái: chỉ admin bật/tắt; mất liên lạc thì giữ nguyên.

**Mức rủi ro** (sơ đồ phát hiện): người nằm trong khung thuyền > 0.4 → **An toàn** (xanh lá, không bám); thấy
khớp thân dưới → **Trung bình** (vàng); dưới nước → **Cao** (cam), thêm hai cổ tay cao hơn vai hoặc chỉ còn thấy
đầu → **Nguy kịch** (đỏ). Bám theo: Nguy kịch > Cao > Trung bình, cùng mức thì người gần nhất.

## 3. Bảng chức năng → cách thử

`scripts\stm32\` (đọc `boat.local.cmd`; chép `boat.local.example.cmd` và điền). Thử theo thứ tự, **tháo chân vịt**.

| # | Chức năng | File | Cần | Phải thấy |
|---|---|---|---|---|
| 0 | mọi test trên máy tính | `test_host.bat` | — | `[TEST] OK` (Pi unittest, 7 host test C, Worker) |
| 0 | cả nhiệm vụ trong giả lập | `test_sim.bat` | WSL + Gazebo | 7 bước `ok`, `PASS` |
| 1 | IMU: mũi = −x, phát hiện lật | `test_imu.bat` | F407 + BNO055 | mũi (trục −x của BNO055) về Bắc: yaw ≈ 0; nghiêng > 35°: `arm_block 2`, còi 2 tiếng |
| 2 | pin INA226 | `test_battery.bat` | + INA226 | áp lệch đồng hồ < 0.1 V, `%` hợp lý |
| 3 | LoRa, ACK, mất liên lạc | `test_lora.bat` | + RA-02 + trạm | `ACK` tăng, `home` = GPS trạm; tắt trạm 60 s: `station LOST` |
| 4 | trạm bờ | `test_station.bat` | ESP32 | log `[NHAN]`, `[ACK]`, `[CLOUD] da gui` |
| 5 | Worker + dashboard | `test_cloud.bat [mock]` | Node.js | `npm test` đạt; `?mock` có chấm người 4 màu, mũi tên, pin, đèn |
| 6 | Pi ↔ F407 | `test_pi_link.bat` | Pi + F407 | `$IMU/$GPS/$SYS` 10/s, `$LNK` 1/s, `PASS` |
| 7 | loa | `test_audio.bat` | Pi + loa | nghe cả hai câu |
| 8 | camera, quét ±45°, mức rủi ro | `test_camera.bat` | Pi + camera + F407 | viewer `:8090`: camera quét; đứng trước: khung + mức rủi ro, servo bám |
| 9 | LiDAR | `test_lidar.bat` | Pi + LD14 | tay trước mũi 0.5 m: có điểm ở góc mũi |
| 10 | hai đèn từ dashboard | `test_light.bat [mask]` | trạm + Worker + F407 | relay theo 1 → 2 → 3 → 0, mỗi lần ≤ 10 s |
| 10b | nút DỪNG PHAO / CHẠY TIẾP | dashboard (admin), hoặc `POST /api/hold` | trạm + Worker + F407 | `test_lora.bat`: `STOPPED from the dashboard` sau ≤ 10 s; Pi log `dashboard_hold` |
| 11 | motor | `test_motor.bat` | gá | đúng chiều quay |
| — | **toàn bộ** | `run_all.bat [noflash] [nostation] [notest] [sim]` | tất cả | xem cuối file |

**`run_all.bat`**: test trên máy tính → nạp F407 → nạp trạm (cửa sổ log riêng) → `boat stm32 deploy` → `main.py`
trên Pi (cửa sổ riêng, `--viewer`) → mở dashboard và viewer. Sau đó:

1. Dashboard: `PHAO-01` online, % pin, mũi tên mũi tàu (IMU) và hướng đi (GPS). Pi: `IDLE`, thuyền đứng yên.
2. Vẽ lộ trình, **Gửi tới phao**: 5–40 s sau thuyền chạy (`PATROL`), camera quét ±45°, loa "Đây là tàu cứu hộ" mỗi 100 s.
3. Đứng trước camera: chấm màu theo mức rủi ro trên bản đồ, kèm độ tin cậy; thuyền bám theo (`FOLLOW`).
4. Nhấn PE4: loa "Đã xác nhận lấy đồ", dashboard báo xanh, thuyền về lộ trình.
5. Nghiêng phao > 35°: motor dừng, dashboard báo lật kèm vị trí.
6. Tắt trạm 60 s: thuyền tự chạy về trạm (`RETURN`, beacon `m=R`), tới nơi thì đứng (`HOME_HOLD`).
7. Bật đèn trên dashboard: relay đóng sau ≤ 10 s, thẻ phao báo "đã bật".
8. Bấm **DỪNG PHAO**: motor khóa sau 5–10 s (F407 tự chặn, kể cả khi Pi vẫn ra lệnh chạy), thẻ phao báo "ĐÃ DỪNG";
   **CHẠY TIẾP** đi tiếp lộ trình đang dở. Gửi lộ trình rỗng: bỏ hẳn lộ trình.

## 4. Giao thức mới (tóm tắt)

| Đường | Thêm |
|---|---|
| Pi → F407 | `$TXD,seq,VICTIM_FOUND\|VICTIM_UPDATE,lat,lon,conf,risk` (risk 0..3; FOUND người mới, UPDATE người đang bám: mức tăng báo ngay, mức giảm sau 20 s) |
| F407 → Pi | `$SYS` thêm `battery_pct` cuối, trường 1 là áp thật; `$LNK,seq,shore_ok,heard_ago_s,home_valid,lat,lon` mỗi giây |
| phao → trạm | beacon thêm `H` đang bị dừng, `L` đèn, `b` pin, `g` hướng đi GPS, `s` tốc độ, `m=R`; khung sự kiện `ev=<id>,<eseq>,V\|T\|C,...` |
| trạm → phao | `ACK,<id>,<eseq>,<lệnh>,<lat trạm>,<lon trạm>*CS` sau mọi khung không cần RTE; lệnh bit 0–1 đèn, bit 2 DỪNG |
| trạm → Worker | `course`, `speed`, `battery_pct`, `lights`, nhóm `event` |
| Worker | `GET /api/victims`, `POST /api/victims/clear`, `POST /api/lights`, `POST /api/hold`, `/api/downlink` thêm `lights`, `hold` |

Chi tiết khung LoRa: [STM32F4/README.md](../STM32F4/README.md) mục 14, [lora-route-downlink.md](lora-route-downlink.md) mục 2.

## 5. Chỉ kiểm được trên phần cứng

- Trục BNO055: heading fusion có thể lấy theo trục Y của module; lệch 90° thì đặt `IMU_MOUNTING_OFFSET_DEG`.
- Shunt INA226: module phổ biến là 0.1 Ω (đo tới 0.8 A); % pin chỉ cần điện áp.
- Mức relay: đèn sáng lúc khởi động thì module đóng ở mức thấp, đặt `RELAY_ON_LEVEL 0`.
- Airtime thật của khung sự kiện + ACK (giả lập: khoảng 1 s mỗi lượt, trong cửa sổ 1.5 s).
- Firmware trạm ESP32 chưa build trên máy này (chưa cài ESP-IDF): `test_station.bat build` là lần biên dịch đầu.
