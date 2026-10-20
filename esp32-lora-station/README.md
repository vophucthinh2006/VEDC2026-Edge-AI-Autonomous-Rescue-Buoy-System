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
| Xem vị trí phao, trạng thái, người cần cứu, bản đồ | có | có |
| Tìm địa chỉ, xem phao nào ở gần | có | có |
| Xem lộ trình (realtime) | có | có |
| Đặt/dời/xoá điểm, gửi lộ trình, dừng phao, bật đèn, xoá chấm | không | có |

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
│   ├── js/util/             geo.js (khoảng cách, phương vị, hướng 8 chữ), format.js
│   ├── js/features/         telemetry (thẻ phao), victims (người cần cứu, chú giải), timeline (dòng sự kiện),
│   │                        search (tìm địa chỉ), hold, lights, mapfx (thang tỷ lệ, vệt),
│   │                        hud (HUD Pixhawk, giữ code nhưng đang ẩn)
│   └── dev/mock.js          Phao giả chuyển động cho ?mock, không được deploy (.assetsignore)
├── tools/mavlink_bridge.py  Cầu nối MAVLink -> /api/ingest (giả lập ArduPilot, sau này Pixhawk thật)
└── cloud/                   Cloudflare Worker
    ├── src/worker.js        Định tuyến, phân quyền, kiểm tra dữ liệu
    ├── src/auth.js          Xác minh ID token Google, phiên cookie ký HMAC
    ├── src/hub.js           Durable Object: phao, lộ trình, phát WebSocket
    ├── src/geocode.js       Tìm địa chỉ qua Nominatim: cache 1 ngày, 1 yêu cầu/giây
    ├── src/util.js
    ├── test/auth.test.mjs   50 kiểm thử đăng nhập/phân quyền/realtime/lịch sử
    ├── test/geocode.test.mjs  tìm địa chỉ với fetch và đồng hồ giả (chạy trong `npm test`)
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
| `PUT /api/mission` | **admin** | lưu lộ trình `{waypoints}` (≤ 300 điểm; `poly`, `spacing`, `angle`, `speed` vẫn nhận nhưng giao diện không còn dùng) |
| `GET /api/geocode?q=` | khách/admin | tìm địa chỉ: tối đa 5 `{name, lat, lon}` trong Việt Nam. 400 câu tìm sai, 429 `{retry_after_ms}` khi quá 1 lần/giây/phiên, 502 khi Nominatim lỗi |
| `POST /api/mission/send` `{id?, loop?, clear?}` | **admin** | nút "Gửi tới phao": chụp lộ trình đang lưu thành bản gửi xuống phao `id` (mặc định `PHAO-01`) |
| `GET /api/downlink` | token ESP32 | trạm lấy bản gửi `{rev, id, loop, pts: [[lat, lon]]}` để phát LoRa (`{rev: 0}`: chưa gửi gì). Có lệnh đèn thì thêm `lights: {"<id phao>": mask}` cho mọi phao có lệnh, kể cả khi `rev: 0` |
| `GET /api/victims` | khách/admin | `{victims: [...], alerts: [...]}`: người phát hiện và cảnh báo (mục "Người phát hiện, cảnh báo, đèn") |
| `POST /api/victims/clear` | **admin** hoặc `ADMIN_API_TOKEN` | nút "Xoá chấm": xóa mọi chấm người và cảnh báo trên bản đồ |
| `POST /api/lights` `{id, lights}` | **admin** hoặc `ADMIN_API_TOKEN` | lệnh hai đèn của phao `id`: `lights` 0..3, bit0 = đèn 1 (PC3), bit1 = đèn 2 (PC2) |

Mọi POST/PUT từ trình duyệt phải có header `Origin` trùng chính trang (chống CSRF). Phiên là cookie
`HttpOnly; Secure; SameSite=Lax` ký HMAC; admin sống 7 ngày, khách 1 ngày. Mỗi request admin kiểm tra lại
email còn trong `ADMIN_EMAILS`, gỡ email khỏi danh sách là mất quyền ngay.

`/api/lights` và `/api/victims/clear` còn nhận header `Authorization: Bearer <ADMIN_API_TOKEN>` thay cho phiên admin,
để script thử bật/tắt đèn không cần đăng nhập Google. Không đặt secret `ADMIN_API_TOKEN` thì chỉ phiên admin dùng được.
```powershell
curl -X POST http://localhost:8787/api/lights -H "Authorization: Bearer dev-admin-token" -H "Content-Type: application/json" -d "{\"id\":\"PHAO-01\",\"lights\":1}"
```

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
npx wrangler secret put ADMIN_API_TOKEN  # tùy chọn: chuỗi bí mật cho script gọi /api/lights, /api/victims/clear
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

Giao diện với phao giả, không cần backend: `?mock` (admin giả) hoặc `?mock=guest` (khách giả). Bản giả có đủ nhãn
trạng thái: PHAO-01 đang tuần tra (có hướng đi GPS, pin, đèn, gửi lộ trình thì thấy "đang nhận k/n" rồi "đã nhận"),
PHAO-02 quay về trạm từng đợt và bị lật sau khoảng 18 giây, PHAO-03 chờ lộ trình rồi mất tín hiệu sau khoảng 36 giây,
PHAO-04 đã dừng (tín hiệu yếu, pin 17 %), SIM-01 báo qua MAVLink. Trong 35 giây đầu có người đủ bốn mức nguy hiểm và
một lần bấm xác nhận. Tìm địa chỉ trả kết quả giả (thử "chợ lớn", "lê thị riêng", "ninh kiều" để thấy trường hợp không có phao).
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
ADMIN_API_TOKEN=dev-admin-token
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
{"id":"PHAO-01","fix":1,"lat":10.762622,"lon":106.660172,"rssi":-88,"snr":7.5,"roll":2.1,"pitch":-1.4,"yaw":278.5,"target_yaw":280.0,"mode":"A","imu_ok":1,"calib":3,"seq":12,"raw":"..."}
{"id":"PHAO-01","fix":0,"rssi":-115,"snr":-8.0,"roll":2.1,"pitch":-1.4,"yaw":278.5,"target_yaw":-1.0,"mode":"S","imu_ok":0,"calib":0,"seq":13,"raw":"..."}
```
`id` có thể vắng (server gán `PHAO-01`). `fix:0` thì không có `lat`/`lon`, nhưng vẫn có
dữ liệu tư thế. ESP32 và cloud kiểm tra miền Roll `[-180,180]`, Pitch `[-90,90]`, Yaw
`[0,360)`, calibration 0..3 và toàn bộ nhóm trường IMU. Nhóm IMU hỏng thì **chỉ bỏ nhóm đó**
(cloud gắn `attitude_bad:1`), vị trí và RSSI/SNR vẫn được lưu và phát.
Gói GPS-only cũ vẫn được chấp nhận; lịch sử server chỉ tiếp tục lưu RSSI/SNR và vị trí.

### Người phát hiện, cảnh báo, đèn (bản STM32)

Beacon có thể thêm (đều tùy chọn, trường nào sai miền thì chỉ bỏ trường đó):
- `course` 0..360 độ: hướng đi thực theo GPS (course over ground), `speed` m/s theo GPS,
- `battery_pct` 0..100, `lights` 0..3: trạng thái **thật** của hai relay (bit0 = đèn 1 PC3, bit1 = đèn 2 PC2),
- `mode` có thêm `R`: phao không nghe thấy trạm bờ quá 60 giây, tự chạy về trạm.

Bản tin sự kiện có `id/rssi/snr/raw` và nhóm `event`, không có nhóm tư thế; nó **không** đổi vị trí hay trạng thái phao:
```json
{"id":"PHAO-01","rssi":-90,"snr":6.5,"raw":"...","event":{"seq":12,"type":"V","lat":10.7651,"lon":106.6612,"risk":3,"conf":0.88,"count":2}}
```
- `seq` 1..65535. Phao gửi lặp tới khi trạm ACK, nên cùng `(id, seq, type)` trong 10 phút chỉ được ghi một lần
  (`/api/ingest` trả `"event":"new"` hoặc `"dup"`).
- `type` `V` = phát hiện người: `risk` 0 an toàn, 1 trung bình, 2 cao, 3 nguy kịch; `conf` 0..1; `count` tổng số người phao đã báo.
  Một `V` cách chấm cũ của cùng phao dưới 5 m thì cập nhật chấm đó (`risk`, `conf`, `last_at`), xa hơn thì thêm chấm.
  `V` có `lat/lon` null (phao chưa có GPS) chỉ thành cảnh báo.
- `type` `T` = lật/nghiêng lớn: `lat/lon` là GPS phao lúc đó, thêm `roll`, `pitch`. `type` `C` = người gặp nạn bấm nút
  xác nhận đã nhận đồ cứu hộ, thêm `presses`.
- Server giữ tối đa 200 chấm người `{vid, buoy, lat, lon, risk, conf, first_at, last_at}` và 50 cảnh báo gần nhất
  `{type, buoy, seq, at, lat, lon, ...}` (khóa `events` trong Durable Object).

Lệnh đèn (`POST /api/lights`) lưu riêng cho từng phao (khóa `lights`), không đụng tới bản gửi lộ trình: đổi đèn không làm
trạm gửi lại lộ trình đã giao. Trạm đọc lệnh trong `GET /api/downlink` (trường `lights`) và phát xuống phao; phao báo
trạng thái thật trong `lights` của beacon. Lệnh giữ nguyên tới khi admin đổi, nên phao khởi động lại vẫn được bật lại đèn.

### Nguồn MAVLink (cầu nối, giả lập)

`tools/mavlink_bridge.py` đọc MAVLink của ArduPilot và gửi cùng định dạng, với `"link":"mavlink"`,
**không có** `rssi`/`snr` (không đi qua LoRa) và thêm nhóm `nav` cho HUD:
```json
{"id":"SIM-01","link":"mavlink","fix":1,"lat":10.883438,"lon":106.796019,"roll":1.2,"pitch":-0.4,"yaw":91.3,
 "target_yaw":90.0,"mode":"A","imu_ok":1,"calib":3,"seq":42,"raw":"MAV GUIDED ARMED gs=1.4 hdg=91",
 "nav":{"armed":1,"mode_name":"GUIDED","gs":1.42,"thr":38,"wp_dist":18.4,"wp_seq":0,"batt_v":12.6,"batt_pct":100,"sats":10,"hdop":1.2}}
```
- `mode` A/M/S suy từ chế độ ArduPilot (AUTO/GUIDED/RTL/LOITER… là A, HOLD hoặc chưa arm là S, còn lại M).
- `calib` không có nghĩa BNO055 ở đây: 3 khi EKF có cả tư thế lẫn vị trí tuyệt đối, ngược lại 0.
- `wp_dist` = -1 khi không đi theo đích, `batt_pct` = -1 khi không biết.
- Cloud kiểm tra miền từng trường; nhóm `nav` hỏng thì chỉ bỏ nhóm đó (`nav_bad:1`).
- Gói MAVLink đến 1–2 lần/giây: vẫn phát realtime từng gói, nhưng chỉ ghi kho và lịch sử **mỗi 5 giây**
  một lần cho mỗi phao (giới hạn số lần ghi Durable Object). Gói LoRa vẫn ghi từng gói như cũ.

Chạy cầu nối (trong WSL, nơi có pymavlink), giả lập đang chạy bằng `sim/run_boat.sh` (MAVProxy phát
sẵn ra cổng 14551):
```bash
export VEDC_INGEST_TOKEN=...     # bằng INGEST_TOKEN của Worker
python3 tools/mavlink_bridge.py --url http://127.0.0.1:8787/api/ingest --id SIM-01
```
Đổi `--url` sang địa chỉ Worker trên Cloudflare để mọi người cùng xem.

Server → trình duyệt (`/ws`): cùng định dạng, thêm `age_s` (giây kể từ lúc server nhận). Khi mở trang,
server phát lại vị trí cuối của từng phao rồi bản tin lộ trình `{"type":"mission","rev":N,...}`.
Mỗi lần admin lưu, mọi trình duyệt nhận lại bản tin `mission` mới (bản của chính tab vừa lưu bị bỏ qua nhờ `cid`).
Thêm các bản tin: `{"type":"victims","victims":[...],"alerts":[...]}` (lúc mở trang và sau khi xóa chấm),
`{"type":"victim","victim":{...},"new":true|false}`, `{"type":"alert","alert":{...}}`, và
`{"type":"lights","id","want","have","by","at"}` (khi admin ra lệnh và khi đèn thật của phao đổi; `have` = `lights` của beacon cuối).

## Giao diện

Mục tiêu: giám khảo nhìn 5 giây là biết phao ở đâu, đang làm gì, có ai cần cứu không, ra lệnh gì được.
Nền tối cố định, chữ Be Vietnam Pro, một màu nhấn cyan, icon Phosphor (tải từ CDN).

- **Thanh trên:** logo, "Trạm bờ cứu hộ", chấm kết nối máy chủ (xanh/đỏ), vai trò và Đăng xuất.
- **Người cần cứu** (đầu cột trái): mỗi người một dòng, chấm màu theo mức rủi ro, tên mức, độ tin cậy, phao báo,
  "x phút trước". Sắp theo mức rủi ro rồi thời gian; bấm là bản đồ bay tới chấm. Admin có nút "Xoá hết".
- **Thẻ phao đang chọn:** tên và một nhãn trạng thái lớn suy ra từ dữ liệu, theo thứ tự ưu tiên:
  "Mất tín hiệu" (quá 30 giây không có gói), "Bị lật" (cảnh báo `T` trong 60 giây), "ĐÃ DỪNG" (`hold`),
  "Đang quay về trạm" (`mode` = `R`), "Đang tuần tra" (Pi đã nhận đủ lộ trình), "Chờ lộ trình".
  Dưới đó: tín hiệu bằng 4 vạch và một chữ Mạnh/Khá/Yếu/Mất (mạnh: RSSI ≥ -90 dBm và SNR ≥ 5; yếu: RSSI < -110
  hoặc SNR < 0; trễ quá 12 giây thì tụt một mức; không hiện số), pin (thanh + %, đỏ dưới 20 %), một câu tốc độ và
  khoảng cách tới trạm, nút lớn **DỪNG PHAO / CHẠY TIẾP**, hai công tắc đèn, một dòng tiến độ gửi lộ trình,
  nút "Sao chép toạ độ".
- **Danh sách phao:** tên, nhãn trạng thái, 4 vạch tín hiệu, pin. Phao cần chú ý (lật, dừng, mất tín hiệu) lên đầu.
- **Sự kiện:** dòng thời gian chỉ gồm việc quan trọng: phát hiện người, lật, xác nhận nhận đồ, dừng/chạy, gửi và nhận
  lộ trình, mất/có tín hiệu, tự quay về trạm. Không còn log kỹ thuật (gói tin, kết nối).
- **Bản đồ:** ghim phao màu theo trạng thái, mũi tên liền là mũi tàu (yaw IMU), mũi tên nét đứt là hướng đi GPS (ẩn khi
  dưới 0.2 m/s); vệt di chuyển luôn bật; trạm bờ màu đỏ, nhãn ghi trạng thái GPS của trạm, đường nét đứt tới phao đang chọn;
  chấm người 4 màu; dấu cảnh báo lật. Chú giải thu gọn được (thu sẵn trên điện thoại). Công cụ: Xem tất cả, Bám theo phao, zoom.
- **Lộ trình:** chỉ đặt waypoint. Chạm bản đồ để thêm, kéo để dời, danh sách điểm có nút xoá, Hoàn tác, Xoá hết,
  ô "Đi vòng", nút lớn **Gửi tới phao** (sửa sau khi gửi thì nút thành "Gửi bản mới tới phao"), dòng "n điểm · x m".
- **Tìm địa chỉ** (khách và admin): ô "Nhập địa chỉ hoặc nơi bạn đang ở…" trên bản đồ và nút "Vị trí của tôi" (định vị
  của trình duyệt, cần https hoặc localhost). Tìm qua `GET /api/geocode` (Worker gọi Nominatim, không gọi thẳng từ trình duyệt).
  Chọn một kết quả: ghim "Vị trí tìm kiếm", vòng bán kính 1/2/5 km (mặc định 2 km), danh sách phao trong vòng, gần nhất
  trước, kèm khoảng cách và hướng ("1.2 km về hướng Đông Bắc"), nhãn trạng thái, nút "Xem trên bản đồ", và số người cần
  cứu gần đó. Không có phao nào thì báo "Chưa có phao trong bán kính x km. Phao gần nhất: … (y km)". Khoảng cách tính
  trong trình duyệt từ dữ liệu phao mà khách vốn đã thấy.
- **Pixhawk/MAVLink:** HUD kiểu Mission Planner (`js/features/hud.js`) vẫn giữ trong code nhưng đang ẩn
  (`SHOW_MAV_HUD` trong `telemetry.js`). Thuyền Pixhawk vẫn có nhãn trạng thái theo chế độ autopilot (RTL, HOLD, armed).
- **Lịch sử ở server:** mỗi phao lưu 300 mẫu gần nhất `{t, rssi, snr, lat?, lon?}` (khóa `h:<id>` trong Durable Object).
  Khi mở WebSocket server gửi `{"type":"history","id":...,"points":[...]}` trước bản tin `mission`, nên khách mở trang vẫn thấy vệt.
- Chỉ hiển thị dữ liệu thật của hệ thống; không nội suy chuyển động khi mất gói.

**Tìm địa chỉ và chính sách Nominatim:** Worker gửi `User-Agent` riêng (`mliot-redgescue/1.0`, thêm secret/biến
`GEOCODE_CONTACT` là email hoặc URL liên hệ nếu muốn), `countrycodes=vn`, `accept-language=vi`, `limit=5`. Một Geocoder duy
nhất trong Durable Object giữ khoảng cách 1 giây giữa hai lần gọi Nominatim cho cả hệ thống, mỗi phiên tối đa 1 lần tìm
mới mỗi giây (lấy từ cache thì không tính), kết quả cache 1 ngày trong bộ nhớ Hub (mất khi Durable Object bị gỡ khỏi bộ nhớ).

## Bố cục đáp ứng (responsive)

| Màn hình | Bố cục |
|---|---|
| Laptop / tablet ngang (> 820 px), điện thoại xoay ngang | Hai cột: bảng điều khiển bên trái, bản đồ bên phải |
| Điện thoại dọc, tablet dọc (≤ 820 px dọc, hoặc ≤ 600 px) | Bản đồ ở trên, bảng điều khiển là ngăn kéo phía dưới, có nút thu gọn. Ô tìm địa chỉ là một hàng riêng trên bản đồ; mở kết quả tìm thì ngăn kéo tự thu gọn, bấm "Xem trên bản đồ" thì kết quả thu về một dòng và ngăn kéo mở thẻ phao |

Điều kiện chuyển bố cục nằm ở hai nơi phải khớp: media query trong `web/css/layout.css` và `narrow` trong `web/js/app.js`.

## Sơ đồ nối dây — ESP32-S3 + LoRa RA-02 (SX1278)

```
VCC(3.3V)->3V3   SCK->GPIO12   MISO->GPIO13
GND->GND         MOSI->GPIO11  NSS->GPIO10
RST->GPIO9       DIO0->GPIO8
```

## Lưu ý

- Cấu hình LoRa (433 MHz, SF9, BW125 kHz, CR 4/5, sync word `0xF3`) phải khớp với node.
- Bản đồ dùng Leaflet, tile OSM, font và nút đăng nhập Google tải từ internet, thiết bị mở web cần internet.
- Chế độ Khách mở tự do: ai có link đều xem được vị trí phao (không sửa được gì).
- Gửi lộ trình xuống phao STM32 qua LoRa: admin bấm "Gửi tới phao", trạm hỏi `GET /api/downlink` mỗi 5 giây và trả lời
  mỗi beacon của phao bằng một khung `RTE` (4 điểm). Phao báo tiến độ trong trường `w=` của beacon, dashboard hiện
  một dòng "đang chờ phao nhận", "đang nhận k/n", "đã nhận đủ, đang chuyển cho máy tính trên phao" rồi "Phao đã nhận lộ trình". Chi tiết: [docs/lora-route-downlink.md](../docs/lora-route-downlink.md).
  Phao Pixhawk không nhận qua đường này (lộ trình của nó nạp bằng Mission Planner).
- WiFi và mật khẩu cũ (`khanh tran`) đã nằm trong lịch sử git; nếu là mạng thật thì nên đổi mật khẩu.
