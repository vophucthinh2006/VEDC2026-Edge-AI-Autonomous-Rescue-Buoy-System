# Điều khiển nhiều tàu từ trạm bờ

Câu hỏi của giám khảo: mô hình phát triển lên nhiều tàu thì trạm bờ điều khiển thế nào?

**Trả lời: điều khiển giám sát theo nhiệm vụ, không lái từ xa.** Một kênh LoRa chỉ được vài trăm bit/s, không đủ
để lái tay từng tàu. Mỗi tàu tự hành và tự giữ an toàn (tránh vật cản bằng LiDAR, mất liên lạc thì tự về). Trạm bờ
giao **nhiệm vụ mức cao** cho cả đội và giám sát.

Phạm vi hiện tại: kiến trúc Pi + STM32F407. Tàu thật vẫn là một chiếc (PHAO-01); đội nhiều tàu chạy trong mô phỏng
(`sim/tools/fleet_sim.sh`).

```
 Dashboard (admin)                 Worker + Hub: bộ quản lý đội tàu            ESP32: cổng LoRa          Tàu n (F407 + Pi)
 ─────────────────                 ───────────────────────────────            ────────────────          ─────────────────
 vẽ vùng, chọn phao ──plan──▶ chia vùng / đội hình (fleetplan.js)  ◀─GET /api/downlink─  routes[4], lệnh   ◀── beacon trong khe riêng
 Dừng / Về bờ cả đội ───────▶ lệnh từng phao: đèn, dừng, về bờ          (5 s một lần)    từng phao         ──▶ RTE / ACK riêng cho tàu đó
 bảng đội tàu ◀──WebSocket── nhiệm vụ, lộ trình đã giao, mất liên lạc,                      điểm đỗ, đội hình
                              điểm đỗ, bầu lại phao dẫn đầu
```

## Ba tầng

| Tầng | Ở đâu | Làm gì |
|---|---|---|
| Quản lý đội tàu | `esp32-lora-station/cloud/src/hub.js`, `fleetplan.js` | Mỗi tàu một bản gửi lộ trình, lệnh riêng, điểm đỗ riêng; sổ nhiệm vụ; chia vùng; đội hình; cảnh báo mất liên lạc; bầu lại phao dẫn đầu |
| Cổng LoRa | `esp32-lora-station/main/` | Giữ tối đa 4 lộ trình (`DOWNLINK_SLOTS`) và lệnh của 8 tàu; trả lời đúng tàu ngay sau beacon của tàu đó |
| Tàu | `STM32F4/App/`, `Rasp_Pi/modules/` | Beacon trong khe thời gian riêng; chấp hành lệnh; tự xử lý mọi thứ cục bộ |

## Worker (API cho admin, Bearer `ADMIN_API_TOKEN` cũng được cho script)

| Đường dẫn | Việc |
|---|---|
| `POST /api/mission/send {id, pts?, loop?, clear?}` | Gửi lộ trình cho một tàu. `pts` là lộ trình riêng của tàu đó; không có thì dùng lộ trình chung. Mỗi tàu một bản gửi, không ghi đè nhau. |
| `POST /api/fleet/plan {mode, poly, ids, spacing?, angle?, shape?, leader?, loop?, dry?}` | `mode: "sectors"`: chia vùng `poly` cho các tàu `ids`, mỗi tàu một dải liền, đi zíc-zắc, dải gán cho tàu gần nhất. `mode: "formation"`: lộ trình cho tàu dẫn đầu, chỗ cho các tàu theo. `dry: true`: chỉ trả kế hoạch để xem trước. |
| `POST /api/formation/end` | Kết thúc đội hình: các tàu đứng tại chỗ chờ lệnh. |
| `POST /api/return {id, ret}` | Gọi một tàu về điểm đỗ của nó / bỏ lệnh. Gửi lộ trình mới cũng bỏ lệnh về bờ. |
| `POST /api/fleet {action: hold\|resume\|return, ids?}` | Lệnh cả đội; không có `ids` là mọi tàu nghe được trong 5 phút. `resume` bỏ cả DỪNG lẫn VỀ BỜ. |

`GET /api/downlink` (cho trạm) thêm: `routes` (mọi bản gửi chưa giao, mỗi tàu một bản; bản mới nhất vẫn ở gốc cho
trạm cũ), `ret`, `home` (điểm đỗ từng tàu), `fol` (tàu theo → `[dẫn đầu, sang phải m, lùi sau m]`).

WebSocket thêm: `task` (nhiệm vụ của tàu: `idle`, `sector`, `return`, `formation`, `victim`: tàu đang giữ một người,
[victim-claim.md](victim-claim.md)),
`ret`, `link` (mất / có lại liên lạc), `formation`; cảnh báo `L` (mất liên lạc 30 s) và `E` (tàu khác lên dẫn đầu).

**Điểm đỗ:** PHAO-01 đỗ ngay tại trạm như khi chỉ có một tàu; PHAO-02 cách 8 m bên phải, PHAO-03 8 m bên trái,
PHAO-04 16 m bên phải... trên đường dọc bờ qua trạm (vuông góc với hướng từ trạm ra chỗ các tàu). Các tàu về bờ đỗ
cạnh nhau, không dồn vào một điểm.

## LoRa

**ACK** thêm bit 3 = VỀ BỜ và ba trường tùy chọn khi tàu ở đội hình:

```
ACK,<id>,<eseq>,<cmd>,<lat>,<lon>[,<dẫn đầu>,<sang phải m>,<lùi sau m>]*CS
ACK,PHAO-03,0,8,10.883366,106.795745*..            về bờ, tới điểm đỗ của PHAO-03
ACK,PHAO-02,0,0,10.883438,106.796019,PHAO-01,10,0*..   đi cách PHAO-01 10 m bên phải
```

Tàu ngoài đội hình nhận đúng khung 6 trường như cũ. ACK 11 trường thêm một người cần bỏ qua (tàu khác đã nhận):
[victim-claim.md](victim-claim.md). F407 đưa sang Pi: `$LNK` thêm trường `ret`; `$FOL,seq,dẫn đầu,phải,sau`;
`$LDR,seq,id,lat,lon,hướng,tốc độ` mỗi khi nghe beacon của tàu dẫn đầu (beacon đã có `g=` hướng đi và `s=` tốc độ GPS).

**Chia khe thời gian (TDMA, `STM32F4/App/lora_slot.c`):** một beacon (~0.7 s ở SF9/BW125) cộng câu trả lời của
trạm (khung RTE ~0.8 s) cần khoảng 1.6 s, nên mỗi tàu có một khe 2 s, 4 khe một chu kỳ 8 s. Khe của tàu là số cuối
tên trừ 1 (PHAO-03 → khe 2), đồng hồ chung là giờ GPS mà tàu nào cũng có, không cần trạm phát đồng bộ. Tàu thật
chạy một mình nên vẫn beacon tự do 5 s; mô phỏng đội tàu dùng khe (`stm32_board_sim.py --tdma`).

## Pi

- `$LNK ret 1` → `RETURN` tới điểm đỗ, rồi `HOME_HOLD`, ở đó tới khi admin cho chạy tiếp (đường truyền vẫn tốt).
- `$FOL` → trạng thái `FORMATION` (mã `O`): đích là chỗ của tàu so với tàu dẫn đầu, vị trí tàu dẫn đầu được ngoại suy
  từ beacon cuối theo hướng và tốc độ; tàu lái tới một điểm phía trước chỗ của mình (chạy song song, không đuổi vòng
  tròn), tốc độ = tốc độ tàu dẫn đầu + phần bị tụt lại (tối đa 1.4 lần tốc độ thường, để bắt kịp). Camera vẫn quét, gặp
  người thì `FOLLOW` như khi tuần tra rồi quay lại chỗ. Không nghe tàu dẫn đầu 15 s: đứng yên chờ.
- Tàu dẫn đầu chạy lộ trình bình thường, không biết mình đang dẫn.
- Tàu dẫn đầu mất liên lạc 30 s: Hub cho tàu theo có tên nhỏ nhất lên dẫn đầu, giao phần lộ trình tàu cũ chưa đi
  (từ waypoint nó đang tới trong khung chẩn đoán), xếp lại chỗ các tàu còn lại.

## Hai cách giao nhiệm vụ

| | Chia vùng | Đội hình (leader–follower) |
|---|---|---|
| Hợp với | vùng rộng, cần phủ nhanh | đi theo tuyến (sông, đường ngập), vào một khu cùng nhau |
| Trạm bờ gửi | mỗi tàu một lộ trình | lộ trình cho tàu dẫn đầu, chỗ cho tàu theo |
| Mất một tàu | phần của tàu đó bỏ trống (vấn đề 2 sẽ giao lại) | bầu tàu dẫn đầu mới, đội hình đi tiếp |
| Tàu nói chuyện với nhau | không | tàu theo nghe beacon tàu dẫn đầu |

## Mô phỏng và so sánh

```bash
VEDC_INGEST_TOKEN=... bash sim/tools/fleet_sim.sh --boats=3 --world=central --viewer
```

`sim/tools/fleet_compare.py run ...` chạy từng kịch bản (không màn hình), gửi kế hoạch như admin, rồi
`fleet_compare.py report` tính: số người tìm thấy và thời gian, % diện tích đã nhìn thấy, quãng đường, năng lượng,
số lần nhiều tàu báo cùng một người, số giây hai tàu quá gần nhau, khung LoRa mất do chồng nhau, sai số giữ đội hình.
