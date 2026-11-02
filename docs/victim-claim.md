# Nhận nạn nhân: một người, một tàu (vấn đề 2 của giám khảo)

Câu hỏi: nhiều tàu cùng phát hiện một nạn nhân thì xử lý thế nào, có tụm lại một chỗ làm tốn tài nguyên không?

Trước khi có cơ chế này, tàu nào thấy người cũng dừng lại đi theo người đó (`FOLLOW`) tới khi người bấm nút xác nhận,
mất dấu, hoặc hết thời gian giữ. Mô phỏng đội tàu ([fleet-compare](fleet-compare/report.html)) cho thấy đây là chỗ tốn
nhất: khi đi đội hình, 7/8 người ở vùng rộng bị nhiều tàu cùng báo, cả đội dừng cùng lúc bên một người.

## Quy tắc

- **Tàu báo một người đầu tiên là chủ** của người đó và ở lại cứu (giữ tới khi người bấm nút xác nhận hoặc hết
  `follow.max_hold_s`).
- **Tàu khác** thấy người cách một người đã có chủ dưới 6 m (`claim_radius_m`) thì **không dừng, không báo lại**, đi tiếp
  nhiệm vụ của mình (dải chia vùng, chỗ trong đội hình).
- Một nhóm người sát nhau (dưới 6 m, vd. ba người trên một mái nhà) tính là **một điểm cứu**, một tàu phục vụ.
- **Đã cứu:** nút xác nhận của tàu chủ được bấm cạnh người (sự kiện `C`), hoặc admin bấm "Đã cứu". Mọi tàu, kể cả chủ,
  bỏ qua người đó 30 phút.
- **Nhả:** tàu chủ mất liên lạc quá 30 s, bị gọi về bờ, hoặc admin bấm "Nhả". Tàu kế tiếp báo người đó thì thành chủ.
- **Giao:** admin chọn một tàu cho một người. Tàu đó thành chủ và nhận lộ trình một điểm tới chỗ người đó.

## Ai phân xử

Trạm bờ ESP32 nghe mọi tàu, nên **trạm phân xử ngay trong ACK** của chính báo cáo đó
(`esp32-lora-station/main/victim_claims.c`). Tàu thứ hai biết "đã có chủ" sau khoảng 1 s (cùng khe thời gian), không
cần internet. Cloud Hub ghi sổ chính (chủ, ai đã thấy, trạng thái), cho admin can thiệp, và gửi lời của nó xuống trạm
trong `GET /api/downlink` → `claims`.

```
Tàu A thấy người ──V──▶ Trạm: chưa ai giữ chỗ này → A là chủ ──ACK──▶ A ở lại
Tàu B thấy cùng người ─V─▶ Trạm: A đã giữ → ACK của B kèm điểm bỏ qua ──▶ F407 B ──$SKP──▶ Pi B bỏ FOLLOW, đi tiếp
                              └──▶ Cloud: một người, chủ A, thấy bởi A, B
```

Mỗi ACK chỉ mang **một** điểm bỏ qua: phán quyết cho báo cáo vừa nhận trước, không thì lần lượt từng người có chủ (hoặc
đã cứu) trong 60 m quanh tàu. Tàu đến gần một người đã có chủ thường đã biết trước khi thấy họ.

## Giao thức

**ACK 11 trường** (`RouteDownlink_AckSkip`, F407 `App/shore_link.c`): thêm điểm bỏ qua sau ba trường đội hình (để
trống khi tàu không ở đội hình). Không có điểm nào thì vẫn là khung 6 hoặc 9 trường như trước.

```
ACK,<id>,<eseq>,<cmd>,<lat>,<lon>,<dẫn đầu>,<phải m>,<sau m>,<lat bỏ qua>,<lon bỏ qua>*CS
ACK,PHAO-02,7,0,10.883438,106.796019,,,,10.883500,106.796100*..
```

Khoảng 80 byte, dưới giới hạn 120 byte của khung.

**F407 → Pi:** `$SKP,seq,lat,lon` một lần mỗi khi ACK mang điểm mới (`ShoreLink_SkipLine`). Pi
(`modules/mission.py`) giữ điểm đó `follow.claim_ttl_s` (120 s, tính lại mỗi lần trạm nhắc). Trong thời gian đó, người
cách điểm dưới `follow.claim_radius_m` không được chọn để theo và không được báo. Tàu đang theo đúng người đó thì rời
đi: "another boat looks after that person".

**Cloud:**

| Đường dẫn | Việc |
|---|---|
| `GET /api/victims` | Mỗi người có thêm `owner`, `status` (`claimed` / `rescued` / `released`), `seen_by` (các tàu đã thấy), `status_at`. |
| `POST /api/victims/:vid {action: rescued\|release\|assign, id?}` | Admin: đã cứu, nhả, giao cho tàu `id` (kèm lộ trình một điểm). |
| `GET /api/downlink` → `claims` | `[[lat, lon, chủ, 1 có chủ \| 2 đã cứu \| 0 nhả], ...]`, tối đa 16 người, mới nhất trước. |

Sổ nhiệm vụ: tàu chủ có `task` kind `victim` (`ref: {vid}`); xong việc (đã cứu, nhả, giao cho tàu khác) thì về lại nhiệm
vụ cũ (dải chia vùng hoặc chỗ trong đội hình).

**Dashboard:** chấm người có viền theo màu tàu chủ (nét đứt khi đang chờ tàu nhận, nhạt đi khi đã cứu). Danh sách ghi
"PHAO-02 +1" (một tàu khác cũng thấy). Popup có các nút **Đã cứu**, **Nhả** và **Giao** cho admin. Dòng thời gian ghi
"PHAO-03 cũng thấy người của PHAO-02: đi tiếp, không dừng".

## Mô phỏng

`sim/tools/shore_station_sim.py` dùng chính `victim_claims.c` qua ctypes (`librouteradio.so`), còn
`stm32_board_sim.py` gửi `$SKP` như F407. Cờ `--no-claims` (`fleet_sim.sh`, `fleet_compare.py run`) tắt cơ chế này để
chạy đối chứng. `fleet_compare.py report` có thêm **giây dừng trùng**: số giây một tàu ở `FOLLOW` cách dưới 10 m một
người mà tàu khác đã báo trước.

## Giới hạn

- Firmware ESP32 chưa được biên dịch ở máy này (không có ESP-IDF). Phần phân xử là C thuần, đã qua host test
  (`esp32-lora-station/test/victim_claims_test.c`) và chạy trong mô phỏng.
- Tàu thật chỉ có một chiếc, nên cơ chế này chỉ thấy được trong mô phỏng. F407 thật đã nhận ACK 11 trường và gửi
  `$SKP` (đã build), không đổi hành vi khi trạm không gửi điểm.
- Vị trí người là ước lượng (GPS tàu + hướng camera + khoảng cách từ khung hình), sai vài mét. Hai người cách nhau dưới
  6 m bị tính là một điểm cứu.
