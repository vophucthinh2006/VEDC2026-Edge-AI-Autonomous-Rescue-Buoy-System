# Thử phao ở hồ: đo khả năng lái

Đo ba thứ: phao **giữ hướng** tốt tới đâu (PID trên F407), **bám lộ trình** ra sao (sai lệch ngang, tới
điểm), và **tốc độ thật** so với lệnh. GPS tĩnh, pin, LoRa theo khoảng cách, LiDAR/camera/Bluetooth: buổi sau.

## Dữ liệu từ đâu

Không phải bật gì: mỗi lần `main.py` khởi động (dịch vụ `vedc-boat`, tức là mỗi lần `run_all.bat`), nó ghi
`~/vedc/Rasp_Pi/logs/nav_<ngày>_<giờ>.csv` cạnh `main_*.log`, 10 dòng/giây ở chế độ AUTO: vị trí, tốc độ và
hướng đi GPS, HDOP, yaw/roll/pitch BNO055, lệnh `$NAV` (chế độ, tốc độ, hướng, lý do), waypoint đang tới
(`k/n`, tọa độ). Giữ 30 lần chạy gần nhất (`logging.keep_runs`), khoảng 7 MB/giờ. Ở MANUAL (điện thoại)
không ghi.

## Các bước

**Trước khi ra hồ**
1. Trên bờ, `test_imu.bat`: mũi phao về Bắc thì yaw ≈ 0 (sai nhiều: hiệu chuẩn lại BNO055 trước khi thả).
2. Pin đầy; `vehicle.max_speed_mps` 0.5 (`config/boat_stm32.yaml`), `arrive_radius_m` 3.0.
3. Một người cầm điện thoại MANUAL (Bluetooth), một người ở dashboard với nút **DỪNG PHAO**.

**Ở hồ**
1. `run_all.bat` như mọi khi (`run_all noflash nostation` nếu không đổi firmware). Pi báo
   `nav log: logs/nav_...csv` trong cửa sổ `Pi main.py`.
2. Chờ GPS có fix (dashboard có vị trí), phao đứng yên 1 phút.
3. Chạy từng bài, mỗi bài là một lộ trình vẽ trên dashboard rồi **Gửi tới phao**. Ghi giờ bắt đầu, gió, sóng.

| Bài | Lộ trình | Để đo |
|---|---|---|
| 1. Chạy thẳng | 2 điểm cách 40 m, đi rồi về, ×2 (4 điểm A-B-A-B) | sai số hướng ổn định, la bàn vs GPS, tốc độ, 3 lần quay đầu 180° |
| 2. Hình vuông | 4 góc 20 × 20 m | đáp ứng khi rẽ 90°, bám đoạn, tới điểm |
| 3. Hình vuông ngược chiều | như bài 2, đi ngược | so ảnh hưởng của gió/dòng nước |

4. Hết lộ trình phao đứng (`IDLE`); muốn dừng giữa chừng: **DỪNG PHAO** hoặc gửi lộ trình rỗng.

**Sau buổi** (laptop nối mạng với Pi): `scripts\stm32\lake_report.bat`. Nó chép các `nav_*.csv` về
`lake_logs\`, làm báo cáo của lần chạy mới nhất và mở trong trình duyệt. `lake_report all`: gộp mọi lần chạy;
`lake_report nocopy`: dùng CSV đã chép. Muốn tách từng bài: chạy tay
`python Rasp_Pi\tools\lake_report.py lake_logs\nav_...csv -o bai1.html` (một lần `run_all` = một file; nếu cả
buổi là một file, các bài vẫn tách được trong bảng **Legs** theo waypoint).

## Báo cáo đọc thế nào

| Chỉ số | Cách tính | Đạt |
|---|---|---|
| heading RMS / p95 | yaw − hướng lệnh `$NAV`, lúc đang chạy, bỏ các khúc đang rẽ | RMS < 10°, p95 < 20° |
| turn settling / overshoot | mỗi lần hướng lệnh đổi ≥ 30° trong 1 s: thời gian tới khi sai số < 10° giữ 2 s; độ vọt qua hướng mới | < 8 s, < 15° |
| compass offset | yaw − hướng đi GPS khi chạy thẳng > 0.3 m/s | trong ±10° |
| cross-track | khoảng cách tới đường thẳng waypoint trước → waypoint đang tới | TB < 2 m, max < 5 m |
| distance at waypoint | cách waypoint bao xa khi phao chuyển sang điểm sau | ≤ 3 m |
| path / straight | quãng đi thật / đường thẳng của đoạn (< 1 là bình thường: đoạn kết thúc ở vòng 3 m) | ≤ 1.3 |
| speed ratio | tốc độ GPS / lệnh, khi lệnh đã giữ 3 s | 0.7 – 1.2 |

Hằng ngưỡng ở đầu `Rasp_Pi/tools/lake_report.py` (`LIMITS`). Biểu đồ: đường đi với các đoạn và vòng 3 m,
hướng lệnh vs yaw và sai số (vạch cam: các lần rẽ), tốc độ lệnh vs GPS và sai lệch ngang.

**Gợi ý khi FAIL**
- RMS lớn, sai số dao động đều quanh 0: PID giữ hướng quá gắt (giảm Kp/Kd trên F407).
- Sai số lệch hẳn một phía: compass offset lớn → hiệu chuẩn lại BNO055, hoặc nhiễu từ motor/pin gần cảm biến.
- Rẽ lâu xác lập, không vọt: lái yếu (góc lái nhỏ, tốc độ thấp) hoặc Kp nhỏ.
- Cross-track lớn mà sai số hướng nhỏ: gió/dòng đẩy ngang (lệnh hướng chỉ nhắm điểm, không bù trôi).
- Speed ratio < 0.7: ga 0.5 m/s không đủ trên nước thật, hoặc GPS báo tốc độ thấp khi chạy chậm.

Thử công cụ khi chưa có dữ liệu thật: `python Rasp_Pi\tools\lake_report.py --sample s.csv` rồi
`python Rasp_Pi\tools\lake_report.py s.csv` (phao giả chạy hình vuông 20 m, mọi chỉ số PASS).
