# Bản đồ thời tiết, mưa và ngập (vấn đề 4 của giám khảo)

Giám khảo đề nghị đổi bản đồ thành bản đồ thời tiết, có hiển thị độ ngập, mưa. Dashboard (trang admin `/admin` và
trang người thân `/`) có thêm ô **Thời tiết & ngập** ở góc dưới bên trái bản đồ. Ô này chứa dự báo và bốn lớp bật/tắt
được; lựa chọn được nhớ trên trình duyệt.

## Các lớp

| Lớp | Nguồn | Là gì | Cập nhật |
|---|---|---|---|
| **Dự báo 24 giờ** | [Open-Meteo](https://open-meteo.com/) | Lượng mưa từng giờ (cột), khả năng mưa, gió mạnh nhất tại trạm bờ | Worker giữ 30 phút |
| **Radar mưa** | [RainViewer](https://www.rainviewer.com/) | Mưa đang rơi, 2 giờ qua, 10 phút một khung; có thanh thời gian và nút phát | Worker lấy danh sách khung 5 phút một lần |
| **Vùng ngập từ vệ tinh** | NASA GIBS, `VIIRS_Combined_Flood_2-Day` | Chỗ có nước ngoài sông hồ thường ngày, quan sát từ vệ tinh, gộp 2 ngày; chọn ngày lùi 1–7 ngày | 1 lần/ngày |
| **Mực nước trạm thủy văn** | KTTV, qua [Hệ thống giám sát thiên tai](https://vndms.gov.vn/) | Khoảng 200 trạm có số liệu: mực nước, giờ đo, mức báo động (chấm xanh / BĐ1 vàng / BĐ2 cam / BĐ3 đỏ) | Worker giữ 15 phút |
| **Độ sâu ngập (ước lượng)** | Mực nước − độ cao địa hình ([Terrain Tiles](https://registry.opendata.aws/terrain-tiles/), SRTM ~30 m) | Tô 4 mức: < 0,5 / 0,5–1 / 1–2 / > 2 m; chỉ hiện khi phóng to từ mức 10 | Theo mực nước đang chọn |

**Mực nước cho lớp độ sâu:**
- Mặc định là trạm thủy văn gần trạm bờ nhất.
- Admin đổi được bằng một trong hai cách:
  - bấm một trạm rồi chọn "Dùng mực nước này cho lớp độ sâu";
  - nhập tay một con số (m).
- "Theo trạm gần nhất" quay về mặc định.
- Lựa chọn được lưu trong Hub, nên trang người thân và mọi cửa sổ admin đều thấy cùng một lớp.

## Đường đi của dữ liệu

```
Trình duyệt ──/api/weather?lat&lon──▶ Worker ─▶ Hub (weather.js, cache) ─▶ Open-Meteo, RainViewer, VNDMS
            ──/api/gauges─────────▶ Worker ─▶ Hub
            ──POST /api/flood-level (admin) ─▶ Hub (lưu, phát "flood" qua WebSocket)
Trình duyệt ──tile trực tiếp──▶ RainViewer, NASA GIBS, Terrain Tiles
```

- `cloud/src/weather.js` (C thuần JS, có test `cloud/test/weather.test.mjs`) lấy dữ liệu, giữ cache và gộp các yêu cầu
  trùng. Khi một nguồn lỗi, Worker vẫn trả bản cũ (tối đa 6 giờ) kèm cờ `stale`.
- `web/js/features/weather.js`: ô điều khiển và các lớp. Các lớp nằm trong một pane riêng, trên bản đồ nền và dưới các
  phao, nên chế độ tối (đảo màu bản đồ nền) không làm sai màu chúng.

## Bản đồ khi mạng yếu

Trước đây mỗi tile bản đồ nền đi thẳng từ trình duyệt sang máy chủ OSM ở châu Âu (DEM thì sang AWS ở Mỹ); mạng yếu
là bản đồ xám, không có chi tiết. Giờ:

- `cloud/src/tiles.js`: Worker phục vụ `/tiles/osm/{z}/{x}/{y}.png` và `/tiles/dem/...`. Lần đầu lấy từ nguồn (có
  User-Agent riêng, đúng chính sách tile của OSM), sau đó cache của Cloudflare gần người dùng trả lời. Trang web khác
  gọi vào (`Sec-Fetch-Site: cross-site`) bị từ chối.
- `web/sw.js`: Service Worker chỉ đụng tới `/tiles/`. Tile đã xem một lần được trả ngay từ máy, kể cả khi mất mạng;
  tile cũ hơn 7 ngày được làm mới ngầm khi có mạng; giữ tối đa 4000 tile (~60–80 MB).
- `web/js/util/basemap.js`: lớp nền dùng chung cho trang admin và trang người thân.
  - Một lớp thô (zoom 13, vài tile) nằm dưới lớp chi tiết: khi chi tiết chưa tải xong bản đồ mờ chứ không xám.
  - Tile nào Worker không trả được thì xin thẳng OSM; trượt 4 lần mà chưa được lần nào (máy chủ tĩnh lúc phát triển)
    thì đi thẳng luôn.
  - Có vị trí trạm bờ: tải ngầm tile zoom 11–16 trong bán kính 3 km (tối đa 500 tile), chỉ khi bản đồ đang hiện đã
    tải xong, mỗi nơi một lần trên mỗi máy, bỏ qua khi trình duyệt bật tiết kiệm dữ liệu. Đến hiện trường thì vùng
    quanh trạm đã có sẵn trên máy.

## Giới hạn (nói rõ với giám khảo)

- **Không có nguồn công khai nào cho độ sâu ngập thật.** Lớp độ sâu chỉ là ước lượng:
  - DEM 30 m đo cả mái nhà và cây, sai vài mét ở đồng bằng;
  - mốc độ cao của DEM và mốc mực nước trạm không trùng nhau;
  - mực nước một trạm không đại diện cho cả vùng.
- **Radar mưa bản miễn phí** chỉ có 2 giờ qua, không còn dự báo radar, và ảnh thô (zoom gốc tối đa 7). RainViewer chỉ
  cho dùng miễn phí cho mục đích cá nhân, giáo dục, và phải ghi nguồn.
- **Vùng ngập vệ tinh** cập nhật 1 lần/ngày, độ phân giải khoảng 375 m; mây che thì thiếu.
- **VNDMS không có API chính thức.** Worker đọc GeoJSON của bản đồ trên trang (cần header `Referer`) và tách mực nước
  từ HTML popup. Họ đổi trang thì lớp trạm ngừng: dashboard vẫn chạy và báo không lấy được số liệu. Muốn dùng lâu dài
  nên xin chia sẻ dữ liệu chính thức từ Cục Quản lý đê điều và Phòng, chống thiên tai.
- Open-Meteo miễn phí cho mục đích phi thương mại.
