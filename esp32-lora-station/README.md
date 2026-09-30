# Trạm bờ — ESP32-S3 + LoRa → Cloudflare

Trạm bờ nhận tọa độ từ phao qua LoRa 433 MHz rồi **đẩy lên Cloudflare** qua HTTPS.
Dashboard web chạy trên Cloudflare, có đăng nhập **Admin** (Google) hoặc **Khách**. ESP32 không còn phục vụ web.

```
Node phao ──LoRa──► ESP32-S3 ──HTTPS POST /api/ingest──► Cloudflare Worker + Durable Object ◄──WSS── Trình duyệt
                    (WiFi/hotspot 4G)                    (vị trí phao, lộ trình, phiên đăng nhập)   (web/ chạy như static assets)
```

## Phân quyền

| | Khách | Admin |
|---|---|---|
| Vào bằng | nút "Xem với tư cách khách" (không cần gì) | Đăng nhập Google, email phải nằm trong `ADMIN_EMAILS` |
| Xem vị trí phao, RSSI/SNR, bản đồ | có | có |
| Xem lộ trình và vùng quét (realtime) | có | có |
| Tạo/sửa/xóa điểm, vẽ vùng quét, nhập file | không | có |

Quyền được kiểm tra **ở server** (giao diện chỉ ẩn nút cho gọn): `PUT /api/mission` trả 403 với khách, 401 khi không có phiên.
Lộ trình là **một bản chung lưu trên server** (Durable Object). Admin lưu thì mọi người đang xem thấy ngay.
Nhiều admin sửa cùng lúc thì bản lưu sau cùng thắng.

## Cấu trúc thư mục

```
esp32-lora-station/
├── main/                    Firmware ESP32
│   ├── main.c               WiFi (tự nối lại), nhận LoRa, hàng đợi, gửi HTTPS lên cloud
│   ├── secrets.example.h    Mẫu cấu hình WiFi + URL + token (sao chép thành secrets.h)
│   └── CMakeLists.txt
├── components/sx127x/       Driver LoRa SX1276/78
│   ├── include/sx127x.h
│   └── src/sx127x.c
├── web/                     Giao diện (HTML + CSS + JS thuần, ES module), responsive
│   ├── index.html, img/logo.png
│   ├── css/                 tokens.css (biến), layout.css, components.css, login.css
│   ├── js/app.js            khởi động, bản đồ, phao, lộ trình, đăng nhập, WebSocket
│   ├── js/state.js          trạng thái dùng chung + bus sự kiện
│   ├── js/util/             geo.js (khoảng cách, phương vị, tuyến quét), format.js
│   ├── js/features/         statusbar, telemetry (đồ thị RSSI/SNR), eventlog, mapfx (thang tỷ lệ, tọa độ, thước đo, vệt)
│   └── dev/mock.js          Phao giả chuyển động cho ?mock, không được deploy (.assetsignore)
└── cloud/                   Cloudflare Worker
    ├── src/worker.js        Định tuyến, phân quyền, kiểm tra dữ liệu
    ├── src/auth.js          Xác minh ID token Google, phiên cookie ký HMAC
    ├── src/hub.js           Durable Object: phao, lộ trình, phát WebSocket
    ├── src/util.js
    ├── test/auth.test.mjs   50 kiểm thử đăng nhập/phân quyền/realtime/lịch sử
    ├── test/dev-idp.mjs     IdP giả để thử đăng nhập admin trên trình duyệt khi phát triển
    ├── wrangler.jsonc
    └── package.json
```

## API

| Method & path | Quyền | Việc |
|---|---|---|
| `POST /api/ingest` | token ESP32 | trạm đẩy bản tin phao (không cần đăng nhập) |
| `POST /api/station` | token ESP32 | trạm báo vị trí GPS của chính nó `{fix, lat, lon, sats, hdop}` mỗi 10 giây |
| `GET /api/config` | công khai | `googleClientId` cho nút đăng nhập |
| `GET /api/me` | công khai | `{role: "admin"\|"guest"\|null, email}` |
| `POST /api/auth/google` `{credential}` | công khai | đổi ID token Google lấy phiên admin (403 nếu email ngoài danh sách) |
| `POST /api/auth/guest` | công khai | phiên khách |
| `POST /api/auth/logout` | — | xóa phiên |
| `GET /ws`, `GET /api/state`, `GET /api/mission` | khách/admin | dữ liệu |
| `PUT /api/mission` | **admin** | lưu lộ trình `{waypoints, poly, spacing, angle, speed}` (≤ 300 điểm) |

Mọi POST/PUT từ trình duyệt phải có header `Origin` trùng chính trang (chống CSRF). Phiên là cookie
`HttpOnly; Secure; SameSite=Lax` ký HMAC; admin sống 7 ngày, khách 1 ngày. Mỗi request admin kiểm tra lại
email còn trong `ADMIN_EMAILS`, gỡ email khỏi danh sách là mất quyền ngay.

## 1. Tạo Google OAuth Client ID (làm một lần)

1. Vào <https://console.cloud.google.com/> → chọn/tạo project → **APIs & Services → OAuth consent screen**
   (User type: External; nếu để trạng thái Testing thì thêm email admin vào **Test users**).
2. **Credentials → Create credentials → OAuth client ID → Web application**.
3. **Authorized JavaScript origins**: thêm `https://mliot-redgescue.<tài-khoản>.workers.dev`
   (và `http://localhost:8787` nếu muốn thử cục bộ). Không cần Redirect URI.
4. Lấy **Client ID** (dạng `xxxx.apps.googleusercontent.com`, là giá trị công khai, không phải bí mật).

## 2. Deploy lên Cloudflare

Cần tài khoản Cloudflare (miễn phí) và Node.js.

```powershell
cd cloud
npm install
npx wrangler login
# đặt "GOOGLE_CLIENT_ID" trong wrangler.jsonc (mục vars) = Client ID ở bước 1
npx wrangler secret put INGEST_TOKEN     # chuỗi bí mật dài ngẫu nhiên, ESP32 dùng để gửi dữ liệu
npx wrangler secret put SESSION_SECRET   # chuỗi ngẫu nhiên >= 32 ký tự, dùng ký cookie phiên
npx wrangler secret put ADMIN_EMAILS     # email admin, nhiều email thì phân cách bằng dấu phẩy
npx wrangler deploy
```

Sửa web hoặc Worker thì chạy lại `npx wrangler deploy`, không phải nạp lại ESP32.
Đổi `SESSION_SECRET` sẽ đăng xuất mọi người. Đổi `ADMIN_EMAILS` có hiệu lực ngay.
Secret `DASH_PASSWORD` của phiên bản cũ không còn dùng, có thể xóa: `npx wrangler secret delete DASH_PASSWORD`.

## 3. Nạp firmware ESP32

```powershell
cd main
copy secrets.example.h secrets.h        # rồi sửa: WIFI_SSID, WIFI_PASSWORD, CLOUD_INGEST_URL, CLOUD_INGEST_TOKEN
cd ..
idf.py set-target esp32s3
idf.py -p COMx build flash monitor
```

`main/secrets.h` nằm trong `.gitignore`. `CLOUD_INGEST_TOKEN` phải trùng `INGEST_TOKEN` đã đặt ở bước 2.

Hành vi firmware:
- WiFi rớt thì tự nối lại mãi; LoRa vẫn nhận, tối đa 16 bản tin xếp hàng chờ gửi. Đầy thì bỏ bản tin cũ nhất.
- Gửi lỗi mạng thì thử lại 3 lần (cách nhau 2 giây). Sai token (HTTP 401) thì dừng ngay và báo log.
- Xác thực chứng chỉ HTTPS bằng bundle của ESP-IDF.

## Chạy thử trên máy

Giao diện với phao giả, không cần backend: `?mock` (admin giả) hoặc `?mock=guest` (khách giả).
```powershell
cd web
python -m http.server 8766        # mở http://localhost:8766/?mock
```

Cả Worker cục bộ (dữ liệu lưu qua các lần chạy). Tạo `cloud/.dev.vars` (đã gitignore):
```
INGEST_TOKEN=dev-token
SESSION_SECRET=<chuỗi ngẫu nhiên>
ADMIN_EMAILS=admin@example.com
GOOGLE_CLIENT_ID=test-client-id
GOOGLE_JWKS_URL=http://127.0.0.1:8799/jwks.json
```
```powershell
cd cloud
npx wrangler dev --port 8787      # mở http://localhost:8787
npm run test:auth                 # 50 kiểm thử (Worker phải đang chạy, khởi động lại wrangler dev trước mỗi lần chạy)
```
Thử đăng nhập admin trên trình duyệt mà không cần Google thật: chạy `npm run dev:idp`, rồi trong console của trang:
```js
const t = await fetch('http://127.0.0.1:8798/token?email=admin@example.com').then(r => r.text());
await fetch('/api/auth/google', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ credential: t }) });
location.reload();
```
Đổi `.dev.vars` xong phải khởi động lại `wrangler dev`. Gửi thử một gói như ESP32:
```powershell
curl -X POST http://localhost:8787/api/ingest -H "Authorization: Bearer dev-token" -d "{\"fix\":1,\"lat\":10.7626,\"lon\":106.6602,\"rssi\":-88,\"snr\":7.5,\"raw\":\"test\"}"
```

## Giao thức dữ liệu phao

ESP32 → `POST /api/ingest` (header `Authorization: Bearer <INGEST_TOKEN>`):
```json
{"id":"PHAO-01","fix":1,"lat":10.762622,"lon":106.660172,"rssi":-88,"snr":7.5,"raw":"..."}
{"fix":0,"rssi":-115,"snr":-8.0,"raw":"TRIGGER,1,NO_FIX"}
```
`id` có thể vắng (server gán `PHAO-01`). `fix:0` thì không có `lat`/`lon`.

Server → trình duyệt (`/ws`): cùng định dạng, thêm `age_s` (giây kể từ lúc server nhận). Khi mở trang,
server phát lại vị trí cuối của từng phao rồi bản tin lộ trình `{"type":"mission","rev":N,...}`.
Mỗi lần admin lưu, mọi trình duyệt nhận lại bản tin `mission` mới (bản của chính tab vừa lưu bị bỏ qua nhờ `cid`).

## Giao diện

Phong cách "trung tâm điều hành": nền tối, số liệu monospace, một màu nhấn cyan, icon Phosphor (tải từ CDN).

- **Thanh trạng thái:** trạng thái máy chủ, số phao trực tuyến / mất tín hiệu / chưa GPS, tuổi gói LoRa cuối, đồng hồ UTC và giờ máy.
- **Viễn trắc phao đang chọn:** tọa độ (thập phân hoặc độ phút giây), RSSI với thanh phân đoạn, SNR, tuổi gói,
  đồ thị RSSI/SNR 5 phút / 15 phút / tất cả với vạch ngưỡng -95 và -110 dBm, bản tin gốc.
- **Nhật ký sự kiện:** kết nối, phao mới, gói tin, mất/có lại tín hiệu (quá 30 giây), mất/có lại GPS, admin cập nhật lộ trình. Lọc "Cảnh báo".
- **Bản đồ:** thang tỷ lệ, tọa độ con trỏ, la bàn (Bắc luôn ở trên), vệt di chuyển của phao, thước đo nhiều điểm (Esc để thoát), nền sáng/tối.
- **Lịch sử ở server:** mỗi phao lưu 300 mẫu gần nhất `{t, rssi, snr, lat?, lon?}` (khóa `h:<id>` trong Durable Object).
  Khi mở WebSocket server gửi `{"type":"history","id":...,"points":[...]}` trước bản tin `mission`, nên khách mở trang vẫn thấy vệt và đồ thị.
- **Trạm bờ (màu đỏ):** trạm có GPS riêng (ATK-S1216 ở UART1, GPIO17 TX / GPIO18 RX, 38400 baud), báo vị trí lên `/api/station`.
  Dashboard vẽ marker đỏ "TRẠM BỜ", chip "Trạm bờ" trên thanh trạng thái (số vệ tinh, chưa có GPS, mất tín hiệu quá 60 giây),
  đường nét đứt tới phao đang chọn, và "Cách trạm bờ" (khoảng cách + phương vị) trong thẻ viễn trắc. Mất fix thì giữ vị trí cuối.
- Chỉ hiển thị dữ liệu thật của hệ thống (vị trí, RSSI, SNR, thời điểm nhận). Chưa có pin, tốc độ, hướng vì phao chưa gửi.

## Bố cục đáp ứng (responsive)

| Màn hình | Bố cục |
|---|---|
| Laptop / tablet ngang (> 820 px), điện thoại xoay ngang | Hai cột: bảng điều khiển bên trái, bản đồ bên phải |
| Điện thoại dọc, tablet dọc (≤ 820 px dọc, hoặc ≤ 600 px) | Bản đồ ở trên, bảng điều khiển là ngăn kéo phía dưới, có nút thu gọn |

Điều kiện chuyển bố cục nằm ở hai nơi phải khớp: media query trong `web/css/layout.css` và `narrow` trong `web/js/app.js`.

## Sơ đồ nối dây — ESP32-S3 + LoRa RA-02 (SX1278)

```
VCC(3.3V)->3V3   SCK->GPIO12   MISO->GPIO13
GND->GND         MOSI->GPIO11  NSS->GPIO10
RST->GPIO9       DIO0->GPIO8
```

## Lưu ý

- Cấu hình LoRa (433 MHz, SF9, BW125 kHz, CR 4/5, sync word `0xF3`) phải khớp với node.
- Bản đồ dùng Leaflet, tile OSM và nút đăng nhập Google tải từ internet, thiết bị mở web cần internet.
- Chế độ Khách mở tự do: ai có link đều xem được vị trí phao (không sửa được gì).
- Chưa có kênh gửi lộ trình xuống phao (nút "Gửi tới phao" đang khóa). Lộ trình đã nằm trên server nên bước tiếp theo
  có thể là `GET /api/mission` dành cho ESP32 (xác thực bằng `INGEST_TOKEN`).
- WiFi và mật khẩu cũ (`khanh tran`) đã nằm trong lịch sử git; nếu là mạng thật thì nên đổi mật khẩu.
