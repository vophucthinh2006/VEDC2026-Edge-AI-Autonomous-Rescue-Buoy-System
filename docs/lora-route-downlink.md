# Gửi lộ trình từ dashboard xuống phao STM32 qua LoRa

Admin vẽ lộ trình trên dashboard, bấm **Gửi tới phao**. Lộ trình đi:

```
dashboard ──PUT──▶ Worker ◀──GET /api/downlink── ESP32 trạm bờ ──LoRa RTE──▶ STM32F407 ──$RTE──▶ Pi (main.py)
                     ▲                              │      ▲                      │    ▲            │
                     └──── POST /api/ingest (route) ┘      └──── beacon w= ───────┘    └─ $HBT rev ─┘
```

Mỗi khâu xác nhận cho khâu trước, nên dashboard biết lộ trình đã tới phao và tới Pi chưa.

## 1. Cloud (`esp32-lora-station/cloud`)

| Đường dẫn | Ai | Việc |
|---|---|---|
| `POST /api/mission/send` | admin | chụp `waypoints` của lộ trình đang lưu thành bản gửi `downlink = {rev, id, loop, pts, by, at}`. `rev` tăng mỗi lần gửi (1..65535, quay vòng bỏ số 0). Body `{id?, loop?, clear?}`: `id` là phao nhận (mặc định `PHAO-01`), `clear: true` gửi lộ trình rỗng để phao bỏ lộ trình cũ. |
| `GET /api/downlink` | trạm (Bearer `INGEST_TOKEN`) | `{rev, id, loop, pts: [[lat, lon], ...]}`, hoặc `{rev: 0}` khi chưa gửi gì hay bản gửi đã giao xong |
| `POST /api/ingest` | trạm | thêm nhóm tùy chọn `route: {rev, have, pi}` lấy từ trường `w=` của beacon |

Hub phát `{"type": "downlink", rev, id, loop, n, by, at, delivered}` cho trình duyệt khi admin gửi và khi bản gửi
được giao. Trạng thái nhận của phao đi trong bản tin phao thường (`route`).

**Giao xong:** khi phao đích báo `route` có `rev` bằng bản gửi, `have` bằng số điểm và `pi = 1`, Hub ghi
`delivered` (thời điểm) vào bản gửi trong kho. Từ đó `GET /api/downlink` trả `{rev: 0}` tới lần gửi sau, nên trạm
thôi phát. Lần gửi mới (kể cả `clear: true`) bắt đầu lại ở trạng thái chưa giao.

Bản gửi là **bản chụp**: admin sửa lộ trình sau khi gửi không làm đổi thứ đang truyền.

## 2. LoRa (SF9, BW125, CR4/5, sync 0xF3, như beacon)

Khung tối đa **120 byte** (beacon cũ là 100; bộ đệm của trạm là 128).

**Phao → trạm (beacon):** khung cũ thêm `w=<rev>:<have>:<pi>` khi phao đang giữ hoặc đang nhận một lộ trình (`rev > 0`):

```
id=PHAO-01,10.883438,106.796019,r=0.4,p=-1.0,y=54.2,t=55.0,m=A,i=1,c=3,q=120,v=0,w=7:12:1
```

- `rev`: lộ trình phao đang giữ hoặc đang nhận, `have`: số điểm đã nhận **liên tiếp từ đầu**, `pi`: 1 khi Pi đã báo nhận đủ `rev` này.
- Đang nhận dở (`have < total`) phao phát beacon mỗi **1.5 s** thay cho 5 s.

**Trạm → phao (`RTE`), chỉ trả lời ngay sau một beacon của phao đích:**

```
RTE,<id>,<rev>,<first>,<total>,<loop>,<lat>,<lon>,<lat>,<lon>,...*CS
RTE,PHAO-01,7,12,40,0,10.883438,106.796110,10.883438,106.796248,10.883546,106.796248*6F
```

- `first`: chỉ số (từ 0) của điểm đầu trong khung, `total`: tổng số điểm (0..300), `loop`: 1 = đi hết thì quay lại điểm đầu.
- Tọa độ 6 chữ số thập phân. Thường 4 điểm một khung; trạm thêm điểm tới khi khung đạt 120 byte.
- `*CS`: XOR mọi ký tự trước `*`, như giao thức UART của Pi. Thêm vào CRC LoRa (trạm giờ bật CRC khi phát),
  vì một chữ số sai là một waypoint sai.
- Trạm gửi khi beacon cho thấy `rev` khác bản gửi, hoặc `have < total`. `first` = `have` nếu cùng `rev`, ngược lại 0.
- `total = 0`: xóa lộ trình. Phao xong ngay (`have = 0 = total`).

**Cửa sổ nhận:** phát beacon xong, phao nghe **1.5 s** (RX liên tục, đọc cờ RxDone qua SPI, không cần chân DIO0).
Trạm chờ 50 ms rồi phát; ở SF9/BW125 beacon 100 byte mất khoảng 0.7 s trên không, khung RTE 120 byte khoảng 0.8 s.

**Phao nhận một khung RTE** đúng `id` của mình:
- `rev` mới: chỉ nhận khi `first = 0`. Xóa lộ trình đang nhận, lấy `total` và `loop` mới.
- cùng `rev`: chỉ nhận khi `first = have`. Nối các điểm, `have` tăng.
- `total > 300`, tọa độ ngoài miền, `first` lệch: bỏ cả khung. Một khung mất thì lần sau trạm gửi lại từ `have`.

Mỗi lượt hỏi-đáp khoảng 1.6 s. Lộ trình 40 điểm: 10 lượt, khoảng 20 s (cộng tối đa 5 s chờ beacon đầu và 5 s trạm hỏi Worker). 300 điểm: khoảng 2 phút.
Đo trong giả lập: 23 điểm tới Pi sau 13 s; 3 điểm sau 5 s kể từ lúc trạm thấy bản gửi.

## 3. STM32F407 ↔ Pi (UART4, giao thức mục 11 của `STM32F4/README.md`)

**F407 → Pi**, khi lộ trình đã đủ và Pi chưa báo nhận `rev` đó:

```
$RTE,seq,<rev>,<first>,<total>,<loop>,<lat>;<lon>,<lat>;<lon>,...*CS
```

Tối đa 7 điểm một dòng. Mỗi chu kỳ telemetry (100 ms) một dòng; hết một lượt thì nghỉ 2 s rồi gửi lại, tới khi Pi báo nhận.

**Pi → F407:** `$HBT` thêm trường thứ 4, `rev` của lộ trình LoRa Pi đang dùng (0 = chưa có):

```
$HBT,seq,<state>,<route_rev>*CS
```

F407 cũ bỏ qua trường này (chỉ cần 3 trường). Pi cũ không gửi nó: F407 coi là 0 và cứ gửi lại.

## 4. Pi

Khi đủ mọi điểm của một `rev`, lộ trình LoRa **thay** lộ trình đang chạy (file `mission.file`), bắt đầu lại từ điểm
đầu. Geofence của file (thành hồ bơi) vẫn giữ: giao thức không gửi geofence. Pi không lưu lộ trình LoRa xuống đĩa:
khởi động lại thì F407 gửi lại, vì `$HBT` báo `route_rev = 0`.

## 5. Chạy và dừng

Thuyền STM32F407 không còn tay điều khiển RC: không có công tắc ARM, không có nút dừng cầm tay. Chỉ dashboard ra lệnh.

- **Gửi lộ trình = chạy.** Pi nhận đủ điểm thì thuyền chạy theo lộ trình đó ngay.
- **Gửi lộ trình rỗng = dừng.** Lộ trình đang trống thì nút **Gửi tới phao** gửi bản rỗng (`clear: true`). Lệnh tới
  thuyền trong khoảng 5–10 s qua LoRa (tối đa 5 s trạm hỏi Worker, tối đa 5 s chờ beacon, rồi một lượt hỏi-đáp).
- **Chưa có lộ trình = đứng yên.**
- **Bản đã giao không bị gửi lại sau khi khởi động lại.** Khi Pi báo nhận đủ, Worker thôi mời trạm phát bản đó
  (mục 1). F407 vừa bật báo `rev` 0, nhưng trạm không còn gì để gửi, nên thuyền không tự chạy trên bờ.
- **Pi khởi động lại giữa chừng:** F407 vẫn giữ lộ trình và gửi lại `$RTE` (`$HBT` của Pi báo `route_rev = 0`),
  thuyền chạy tiếp (Pi bắt đầu lại từ điểm đầu, mục 4).
- **F407 reset:** mất lộ trình, thuyền đứng yên tới khi admin bấm **Gửi tới phao** lần nữa. Dashboard báo
  "Pi đã nhận bản này ... nay phao không còn giữ nó".

## 6. Mã nguồn

| Khâu | File |
|---|---|
| Cloud | `cloud/src/worker.js`, `hub.js` (bản gửi, giao xong), `telemetry.js`; test `cloud/test/hub.test.mjs` (`npm test`) |
| Dashboard | `web/index.html`, `web/js/app.js` (nút Gửi tới phao, trạng thái nhận) |
| ESP32 | `main/route_downlink.c` (dựng khung, C thuần, có test), `main/main.c` (lấy bản gửi, trả lời beacon) |
| F407 | `App/route_link.c` (nhận khung, ráp lộ trình, dòng `$RTE`, C thuần, có test), `Modules/LoRa/sx127x.c` (RX), `App/lora_beacon.c` (cửa sổ nhận, `w=`), `Core/Src/main.c` (gửi `$RTE`, đọc `$HBT`) |
| Pi | `modules/lora_route.py` (ráp `$RTE`), `modules/uart_comm.py` (`lora_route`), `main.py` (thay `Route`, `$HBT` thêm `lora_rev`) |
| Giả lập | `sim/tools/stm32_board_sim.py` (phần phao, dùng chính `route_link.c`), `sim/tools/shore_station_sim.py` (phần trạm, dùng chính `route_downlink.c`) |
