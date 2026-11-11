# Trang người thân (/): quê nhà là trung tâm

Người xem là người thân ở xa, đọc trên điện thoại, đang lo. Trang để nền sáng, màu dịu, khác hẳn trạm điều hành
tối (`docs/admin-console-ui.md`). Thông tin hiện bằng số và hình, ít câu. Design đã duyệt nằm trong canvas ở
`esp32-lora-station/web/dev/redesign/` (khung `Guest` và `GuestPhone`).

## Bố cục

Máy tính: cột thông tin bên trái (tối đa 480 px, cuộn), bản đồ bên phải. Dưới 820 px trang cuộn liền một dòng:
quê nhà, người gần nhà, bản đồ (320 px), rồi phần còn lại (`.g-scroll` dùng `display: contents` cùng `order`).

| Thẻ | Phần tử | Số liệu |
|---|---|---|
| Chọn quê nhà | `#pick` | Hiện khi chưa có quê nhà, hoặc khi bấm "Đổi" (có nút "Giữ nơi cũ") |
| Quê nhà | `#area` | Vòng trạng thái, 4 ô số, mưa 24 giờ, mực nước sông |
| Người gần nhà | `#people-card`, `#near-list` | Mức nguy hiểm, cách nhà, thấy lần cuối, tình trạng cứu |
| Toàn vùng | `#s-buoys`, `#s-people`, `#s-aid` | Phao liên lạc, người phát hiện, người được giúp |
| 12 giờ qua | `#trend` | Cột phát hiện / được cứu theo giờ |
| Tin mới | `#feed` | Icon theo loại tin, thời gian ngắn (giờ chính xác khi trỏ chuột) |
| Các phao | `#buoys` | Chip: chấm trạng thái, tên, chữ ngắn (`SHORT`) |

## Vòng trạng thái (trong 5 km quanh nhà)

Theo thứ tự ưu tiên:
1. Đỏ "Có người nguy kịch": có người mức 3 chưa được cứu.
2. Cam "Có người cần cứu" hoặc "N người cần cứu": có người mức 1–2 chưa được cứu. Người mức "An toàn" không tính.
3. Xanh "Phao đang tìm kiếm" hoặc "Có phao gần nhà": có phao hoạt động trong 5 km.
4. Xám "Chưa có phao gần nhà" hoặc "Chưa có phao báo về".

Chip liên lạc dưới vòng tính theo khoảng cách tới trạm bờ: ≤ `LORA_GOOD_M` là tốt, ≤ `LORA_WEAK_M` là rìa, xa hơn là
ngoài vùng.

## Mực nước

Trạm KTTV có mực nước gần nhà nhất. Thang có 4 nấc: dưới báo động, BĐ1, BĐ2, BĐ3. Kim chỉ vào **nấc** hiện tại,
không phải vị trí chính xác trong nấc, vì ngưỡng của từng trạm không được công bố. Nếu các trạm khác trong 30 km
đang vượt báo động thì hiện thêm một dòng đỏ, tên các trạm nằm trong tooltip.

## Chuyển động

Token giống `css/console.css` nhưng chậm hơn: `--dur-1..4` là 120 / 220 / 380 / 800 ms.

- Các thẻ hiện lần lượt khi tải trang.
- Số đếm lên khi đổi giá trị (`countTo`). Luôn chốt đúng số cuối, kể cả ở tab chạy nền.
- Vòng trạng thái "thở" (trừ khi xám) và đổi màu mượt.
- Cột mưa và cột 12 giờ mọc lên. Biểu đồ chỉ vẽ lại khi số liệu đổi.
- Kim mực nước và mũi tên hướng phao gần nhất trượt tới vị trí bằng hiệu ứng nảy.
- Người và tin mới trượt vào. Chỉ những dòng chưa thấy mới chạy hiệu ứng.
- Trước khi có dự báo, ô mưa và ô mực nước hiện khung xương lấp lánh thay cho câu chữ.
- `prefers-reduced-motion`: tắt hết.

Thẻ quê nhà dựng sẵn trong `index.html`. `drawArea` chỉ đổi số và thuộc tính, không dựng lại DOM, nên animation
không chạy lại mỗi 15 giây. `render()` dùng `requestAnimationFrame` và có timer dự phòng, nên tab chạy nền vẫn được
vẽ.

## Thử

`index.html?mock=guest` trên server tĩnh (`dashboard-static`): `dev/mock.js` đứng thay Worker, và trang không mở
phiên khách. Có thể đặt sẵn quê nhà bằng `&p=lat,lon&n=Tên`. Thử ở 1440×900 và 375×812.
