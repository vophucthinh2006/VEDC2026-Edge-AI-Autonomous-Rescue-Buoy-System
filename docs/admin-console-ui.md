# Giao diện Trạm điều hành (/admin) kiểu bảng lái tàu

Bản đồ toàn màn hình. Thông số đọc bằng số và hình, ít câu chữ. Design đã duyệt nằm ở
`esp32-lora-station/web/dev/redesign/` (canvas, không deploy).

## Bố cục

| Vùng | Phần tử | Nội dung |
|---|---|---|
| Thanh trên | `#pill`, `#st-sats`, `#conn` | Phao đang chọn và trạng thái của nó (một dòng), số vệ tinh trạm bờ, kết nối |
| Hàng thẻ | `#gauges` (`js/features/gauges.js`, `css/gauges.css`) | Mưa 24 giờ, lắc ngang, chúi dọc, hướng, pin, LoRa |
| Rail phải | `#rail` | Cần cứu, Sức khoẻ, Đội phao, Sự kiện, Mưa/ngập, Chú giải. Mỗi nút mở một ngăn trong `#drawer` |
| Giữa đáy | `#ctl` | Đổi phao ‹ ›, tốc độ GPS, gọi về bờ (`/api/return`) |
| Thanh dưới | `#dock` | Giám sát / Lộ trình / Đội tàu, đèn, DỪNG PHAO (`#d-hold`) |

Ở chế độ Lộ trình và Đội tàu, hàng thẻ thu thành dải mỏng, ngăn bên phải hiện trình sửa lộ trình hoặc giao
nhiệm vụ. Trên điện thoại, rail và ngăn thành bottom sheet, các thẻ cuộn ngang, thanh dưới chỉ còn icon.

## Số liệu, chỉ những gì hệ thống thật sự gửi

- Mưa: `forecast()` của `features/weather.js` (Open-Meteo qua `/api/weather`). Không có nhiệt độ, không có gió
  đo tại chỗ (phao không có cảm biến gió).
- Lắc ngang, chúi dọc: `roll`, `pitch` (IMU). Ngưỡng 20° / 30° như `features/health.js`.
- Hướng: mũi tàu hiện tại `yaw` và hướng Pi ra lệnh `target_yaw` (-1 là chưa ra lệnh), đọc bằng chữ
  (`compassWord` của `util/geo.js`, 8 hướng), số độ để nhỏ. Độ lệch giữ bằng độ, ngưỡng 10° / 30°.
- Pin: `battery_pct`, kèm V và A của INA226 (khung chẩn đoán).
- LoRa: `signalOf` (RSSI, SNR, tuổi gói) và khoảng cách tới trạm bờ.
- Số liệu cũ (IMU trễ, phao mất tín hiệu): thẻ viền nét đứt kèm nhãn "số cũ".

Bảng thời tiết và chú giải vẫn là control của Leaflet. `app.js` (`intoDrawer`) chuyển chúng vào ngăn tương ứng.

## Chuyển động

Token trong `css/console.css`: `--dur-1..4` (120 / 200 / 320 / 650 ms), `--ease-out`, `--ease-spring`,
`--ease-needle`. Chỉ dùng transform và opacity. Khi đóng nhanh hơn khi mở.

- Hàng thẻ trượt xuống lần lượt khi tải trang. Kim la bàn, thuyền lắc/chúi quay mượt theo số mới, kim luôn
  quay đường ngắn.
- Vòng pin chạy theo %, cột mưa mọc lên. Giọt mưa chỉ rơi khi dự báo có mưa. Vạch nét đứt "hướng Pi" chạy
  liên tục.
- Ngăn trượt vào từ phải, các dòng hiện lần lượt (40 ms/dòng). Huy hiệu trên rail nảy lên khi số tăng.
- Trạng thái nguy hiểm (lật, lệch hướng lớn, pin cạn) có viền đỏ phập phồng: luôn kèm số và chữ, không chỉ màu.
- `prefers-reduced-motion`: `tokens.css` tắt mọi animation và transition.

## Thử

`admin.html?mock` trên server tĩnh (`dashboard-static` trong `.claude/launch.json`), ở 1440×900 và 375×812,
cả nền sáng và tối.
