# Giả lập 3D phao cứu hộ — ArduPilot + Gazebo + Mission Planner

Phao chạy trong **Gazebo Harmonic** (mặt nước có sóng, lực nổi, chân vịt), được điều khiển bởi
**ArduPilot Rover 4.7.1** chạy giả lập (SITL), và **Mission Planner** trên Windows nối vào qua MAVLink —
giống hệt khi nối với phao thật.

```
Windows                         WSL Ubuntu 24.04
┌────────────────┐  UDP 14550   ┌──────────┐  TCP 5760  ┌────────────┐  UDP 9002   ┌──────────────────┐
│ Mission Planner│◄────────────►│ MAVProxy │◄──────────►│ ardurover  │◄───────────►│ Gazebo           │
└────────────────┘  (MAVLink)   └──────────┘            │ (SITL)     │ (lock-step) │ sóng + vedc_buoy │
                                                        └────────────┘             │ camera + LiDAR   │
                                                                                   └──────────────────┘
```

Mã nguồn bên thứ ba nằm ở `~/vedc_sim` trong WSL (không đưa vào repo): `ardupilot`, `ardupilot_gazebo`,
`SITL_Models` (mô hình BlueBoat), `asv_wave_sim` (sóng và thủy động lực học).

## Cài đặt (một lần)

```bash
wsl -d Ubuntu-24.04 -- bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/install_deps.sh
wsl -d Ubuntu-24.04 -- bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/build.sh
```

- `install_deps.sh` cần sudo: CGAL, FFTW, GStreamer, colcon và script cài đặt chính thức của ArduPilot
  (tạo `~/venv-ardupilot` có MAVProxy, pymavlink).
- `build.sh` không cần sudo, khoảng 10 phút: ArduPilot SITL, plugin Gazebo, plugin sóng.
  Build lại một phần: `build.sh ardupilot | gazebo | waves`.
- Gazebo Harmonic có sẵn từ ROS 2 Jazzy, không cài riêng.
- `.wslconfig` phải có `networkingMode=mirrored` để Windows thấy `127.0.0.1` của WSL.

## Chạy

```bash
wsl -d Ubuntu-24.04 -- bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/run_boat.sh
```

Mở cửa sổ Gazebo 3D, một cửa sổ xterm của ArduPilot, và dấu nhắc `MAV>` của MAVProxy trong terminal.
`Ctrl+C` ở terminal để tắt tất cả.

| Tùy chọn | Tác dụng |
|---|---|
| `--headless` | Gazebo không mở cửa sổ (máy yếu, hoặc kiểm thử tự động) |
| `--wipe` | Xóa tham số đã lưu, nạp lại mặc định + `params/vedc_buoy.parm` |
| `HOME_LOCATION=lat,lon,alt,hướng` | Đổi điểm xuất phát. Mặc định giữa **Hồ đá 01, Làng Đại học ĐHQG-HCM** `10.883438,106.796019,10,90` (cách bờ 217 m, phía đông trống khoảng 265 m) |

**Mission Planner:** góc trên phải chọn **UDP** → **Connect** → cổng **14550**.

## Thử nhanh trong Mission Planner

1. Tab **DATA**, đợi HUD hết báo lỗi EKF (khoảng 20–30 giây sau khi khởi động)
2. **Actions** → **Arm/Disarm**
3. Chuột phải lên bản đồ → **Fly to Here** (chế độ GUIDED) — phao chạy tới đó, xem trong Gazebo
4. Tab **PLAN**: click để thêm waypoint → **Write WPs** → về DATA, đổi chế độ sang **Auto**
5. Đổi sang **RTL** — phao quay về điểm xuất phát
6. Thử failsafe mất liên lạc: khi phao đang chạy, ngắt Mission Planner (**Disconnect**) và gõ
   `set heartbeat 0` ở dấu nhắc `MAV>` (MAVProxy cũng phát heartbeat GCS). Sau 5 giây phao tự RTL.
   Gõ `set heartbeat 1` để bật lại.

## Kịch bản cứu hộ với code của Pi

```bash
wsl -d Ubuntu-24.04 -- bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/run_boat.sh --pi
```

`--pi` chạy thêm `Rasp_Pi/rescue_main.py` (đúng code sẽ chạy trên Pi thật) và nạp `params/avoidance.parm`:

- ảnh camera Gazebo đi vào model TFLite của đội (`Rasp_Pi/models/detect.tflite`);
- Pi nối vào **TELEM2 giả lập** của ArduPilot (TCP 5763), như trên phao thật;
- LiDAR Gazebo đi vào Pi, Pi gửi cho ArduPilot (`OBSTACLE_DISTANCE`, 72 ô 5°); BendyRuler bẻ lộ trình
  vòng qua vật cản rồi quay lại tuyến;
- khi phao đang chạy **AUTO** và thấy người đủ 1 giây: Pi chuyển **GUIDED**, đưa phao tới cách người 2 m,
  chuyển **LOITER** giữ vị trí 20 giây, báo `VICTIM SEEN / REACHED` (hiện ở tab Messages của Mission Planner),
  rồi trả lại **AUTO**; người đã tiếp cận thành vùng cấm để lộ trình quét tiếp không chạy qua;
- gạt sang chế độ khác (MANUAL, HOLD, RTL...) lúc nào Pi cũng nhả quyền ngay.

Cách thử: vẽ lộ trình đi ngang gần nạn nhân (cách điểm xuất phát 32 m về đông, 8 m về bắc), Write WPs,
Arm, AUTO. Log của Pi: `~/vedc_sim/sitl_run/pi.log`. `--pi=simulated` dùng bộ phát hiện hình học thay model.

**Xem Pi đang thấy gì.** Với `--pi`, mở `http://127.0.0.1:8090/` trên trình duyệt:

- bên trái là ảnh camera, có khung quanh người mà model phát hiện, độ tin cậy, khoảng cách ước tính, và góc
  servo đang nhìn (đang quét, đang bám, hay nhìn thẳng);
- bên phải là LiDAR nhìn từ trên xuống, mũi phao hướng lên: chấm trắng là điểm LiDAR quét được, vạch đỏ là
  72 ô 5° đúng như Pi gửi cho bộ lái, vòng cam là vùng cấm quanh nạn nhân đã tiếp cận, hình quạt là góc nhìn
  60° của camera (vàng khi quét, xanh khi bám), dấu × là người đang được bám;
- dòng dưới cùng: chế độ bay, pha cứu hộ, waypoint hiện tại, vật cản gần nhất.

Trang này do chính `rescue_main.py --viewer [cổng]` phát ra (`Rasp_Pi/modules/viewer.py`), khoảng 5 hình/giây,
chỉ đọc dữ liệu chứ không tác động tới phao. Trên Pi thật dùng được y như vậy qua wifi:
`http://<địa chỉ Pi>:8090/`. Với `--pi=simulated` không có ảnh camera, chỉ có bản đồ LiDAR và hình quạt.

Không dùng `--pi` thì giả lập chạy như cũ với riêng Mission Planner. (Khi đã nạp `avoidance.parm` mà không
có Pi gửi dữ liệu vật cản, ArduPilot từ chối arm.)

Kiểm thử tự động, chạy song song được với phiên đang mở:

```bash
bash sim/tools/rescue_test.sh camera      # model thật trên camera Gazebo
bash sim/tools/rescue_test.sh simulated   # bộ phát hiện hình học
```

**Camera trong giả lập**: 640 × 480, 15 khung/giây, góc nhìn ngang 60° (dọc 46.8°), cao 0.295 m trên mặt
nước. Bộ phát hiện bị giữ ở 8 khung hình/giây (`camera.max_fps` trong `sim.yaml`), tốc độ đội báo cho Pi 4.
Các số đo trong tài liệu này được lấy khi còn giả định 5 khung hình/giây.

**Servo camera quét khi đang tìm** (`camera.scan` trong `settings.yaml`). Khi phao đã arm, đang ở AUTO và
chưa thấy ai, camera quay qua lại giữa −60° và +60°, phủ 180° phía trước. Thấy người là dừng quét, servo
chuyển sang bám; mất dấu 1.5 s thì quét tiếp từ đúng góc đang nhìn. Ở chế độ khác (lái tay, GUIDED, LOITER,
RTL) camera nhìn thẳng mũi. Có hai kiểu quét (`camera.scan.mode`):

- `sweep` (mặc định): quay mượt với tốc độ `sweep_rate_deg_s` (30°/giây). Lệnh servo gửi theo từng khung
  hình camera, mỗi bước khoảng 2°. Một người nằm trong khung hình 60 / tốc độ giây (2 giây ở 30°/giây); một
  vòng trái–phải–trái mất 240 / tốc độ giây (8 giây), trong lúc đó phao vẫn chạy. Quay chậm hơn thì nhìn
  mỗi hướng lâu hơn nhưng lâu hơn mới quay lại.
- `step`: dừng ở **0° → −60° → 0° → +60°**; ở mỗi hướng chờ 0.25 s cho servo đứng yên (bỏ khung hình chụp
  lúc đang xoay), xét 3 khung hình rồi sang hướng kế. Hướng mũi được nhìn gấp đôi.

Trong giả lập hai kiểu cho kết quả như nhau (bảng dưới). Camera của Gazebo không có nhòe chuyển động, nên
ảnh hưởng của việc quay lên chất lượng ảnh phải thử trên camera thật.

Pi không tự dừng phao để quét. Muốn nhìn kỹ một chỗ, đặt thời gian chờ ở waypoint đó (cột Delay trong
Mission Planner): phao đứng yên, vẫn ở AUTO, camera vẫn quét.

Xác nhận một người: thấy trong ít nhất 3 khung hình, trải trên 1 giây, không có khoảng trống nào dài quá
1 giây (`rescue.confirm_*`). Điểm của model sát ngưỡng nên hay rớt một hai khung hình; trước đây rớt một
khung là đếm lại từ đầu.

Dải tìm kiếm đo trong cảnh hồ, phao chạy 1 m/s trên lộ trình thẳng đi ngang cách nạn nhân một khoảng:

| Lộ trình cách nạn nhân | Camera cố định (`noscan`) | Dừng–nhảy (`scan`) | Quay mượt 30°/s (`sweep`) |
|---|---|---|---|
| 2 m | thấy | thấy | — |
| 3.5 m | thấy | thấy, 2 lần trên 2 | thấy |
| 5 m | thấy | thấy | thấy |
| 6.5 m | **không thấy** | thấy, 2 lần trên 2 | thấy, 2 lần trên 2 |
| 6.5 m, chờ 12 s ở waypoint ngang nạn nhân | **không thấy** | thấy | — |
| 8 m | — | **không thấy** | **không thấy** |

Quay mượt ở 20°/giây và 45°/giây cũng thấy người ở 6.5 m (mỗi tốc độ một lần chạy). Tọa độ báo về lệch
0.2–1.7 m. Mỗi ô là 1–2 lần chạy. Tức với camera cố định các tuyến quét nên cách nhau không quá khoảng
10 m, có quét thì khoảng 13 m. Đo lại:

```bash
bash sim/tools/scan_test.sh camera 6.5 sweep       # lệch 6.5 m, quay mượt
bash sim/tools/scan_test.sh camera 6.5 sweep=20    # quay mượt 20 độ/giây
bash sim/tools/scan_test.sh camera 6.5 scan        # dừng–nhảy
bash sim/tools/scan_test.sh camera 6.5 noscan      # camera cố định
bash sim/tools/scan_test.sh camera 6.5 sweep 12    # dừng 12 s ở waypoint ngang nạn nhân
```

Khi nạn nhân lệch hẳn sang bên, lúc phao quay mũi để tiến lại camera có khi mất dấu vài giây
(`VICTIM LOST`) rồi bắt lại; phao vẫn tới nơi nhưng chậm hơn.

**Quét trong cảnh có nhiều nhà thì rủi ro hơn.** Camera nhìn ngang vào nhà cửa nên model có nhiều thứ để
nhầm hơn. Trong cảnh lũ, một lần chạy có quét đã cho hai báo nhầm (model chấm 0.64 cho một góc mái nhà, ngang
điểm của người thật 0.61–0.77 nên nâng ngưỡng không lọc được), phao tự tiến tới đó và áp sát nhà. Sau hai
sửa đổi (khoảng cách tới người dưới nước chỉ lấy từ khung hình; dừng tiếp cận khi có vật cách LiDAR dưới
0.9 m), ba lần chạy tiếp theo không có báo nhầm nào, nhưng ba lần là quá ít để nói đã hết.
`flood_test.sh` đếm báo nhầm và coi đó là không đạt. Tắt quét: `rescue_main.py --no-scan`, hoặc
`camera.scan.enabled: false`; so sánh bằng `bash sim/tools/flood_test.sh camera noscan`.

**Tầm phát hiện đo được** (model SSD MobileNet v1, ngưỡng 0.60, người quay mặt về phao, camera đứng yên
trên nước lặng của cảnh lũ, 30 khung hình mỗi cự ly):

| Cự ly | Người ngập tới ngực | Người đứng trên mái |
|---|---|---|
| 2–3 m | 0.74 | — |
| 4 m | 0.62 | 0.62 |
| 5 m | 0.67 | 0.62 |
| 6 m | 0.72 | 0.61 (87 % khung hình đạt ngưỡng) |
| 7 m | 0.70 | — |
| 8 m | 0.66 | 0.64 |
| 10 m | 0.54, không đạt | 0.48, không đạt |
| 12–20 m | — | 0.40–0.52, không đạt |

Không có người trong khung hình: điểm cao nhất 0.17. Tức **tầm tin cậy là 8 m**, nhưng điểm chỉ cách
ngưỡng 0.01–0.14: đây là điều kiện đẹp nhất. Trên mặt hồ có sóng, phao đang chạy, phép đo trước đó chỉ chắc
tới 4 m, chập chờn ở 5–6 m, và phao tiêu màu cam từng bị chấm 0.33–0.56. Vì vậy tốc độ quét để 1.0 m/s và
các tuyến quét phải đủ sát nhau.

## Cảnh lũ lụt

```bash
wsl -d Ubuntu-24.04 -- bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/run_boat.sh --pi --world=flood
```

Một con phố ngập tới mái (`worlds/vedc_flood.sdf`). Phao xuất phát ở đầu phố, mũi hướng đông:

```
 bắc  [nhà]  [nhà tôn]  [nhà]  [nhà tôn]              mép mái cách tim đường 6.2 m
      P>    (cây)    (xe)    A    (cột)     R[nhà cuối phố]
 nam  [nhà tôn]  [nhà]  [nhà tôn]  [nhà]
      0      10      18     27    31      43.5   m về phía đông
```

| Vật | Vị trí (đông, bắc) | |
|---|---|---|
| ngọn cây | 10, -1.0 | chắn tim đường |
| ô tô ngập tới cửa kính | 18, 1.0 | |
| **A** người dưới nước | 27, -0.5 | |
| cột điện nghiêng | 31, 0.8 | |
| **R** người đứng ở mép mái nhà cuối phố | 43.5, 0.5 | |

Cách thử trong Mission Planner: vẽ **một waypoint ở cuối phố, cách điểm xuất phát khoảng 38 m về đông**
(đừng đặt sát nhà cuối phố hơn 4.5 m), thêm lệnh RTL, Write WPs, Arm, AUTO. Phao lượn qua cây, xe;
dừng cách người dưới nước 2 m, giữ 20 giây, đi tiếp; vòng qua cột điện; dừng trước người trên mái; rồi về,
lại vòng qua các vật cản và qua chỗ người dưới nước.

Người trên mái: phao tới sát nhất mà nhà cho phép. Trên những mét cuối, hễ LiDAR thấy vật ở phía trước gần
hơn 1.5 m (bức tường dưới chân người đó) thì phao dừng tại đó và báo `VICTIM REACHED`; điểm dừng 2 m tính
từ người, không tính từ tường, nên khi phao tiến vào theo góc chéo nó có thể nằm sát tường. Nếu trong 10 giây không lại gần thêm được
(`rescue.stall_s`), Pi báo `VICTIM UNREACHABLE <vĩ độ> <kinh độ>` thay cho `VICTIM REACHED`, vẫn giữ vị trí
rồi đi tiếp. Khoảng cách tới người lấy từ LiDAR khi có tia trúng đúng hướng đó (bức tường dưới chân người
trên mái, trong vòng ±3°), không thì từ chiều cao khung hình (`camera.distance_source: fused`). LiDAR chỉ
được dùng cho người đứng khỏi mặt nước: với người dưới nước, tia LiDAR đi qua đầu họ và trúng thứ đứng sau lưng. Khung hình có
đáy nằm trên đường chân trời là người đứng khỏi mặt nước: tính theo chiều cao đứng 1.7 m thay vì 0.7 m của
người ngập tới ngực, nếu không khoảng cách bị đọc ngắn đi 2.5 lần.

**Waypoint rơi trúng vật cản.** Người vẽ lộ trình không thấy cây, xe hay mái nhà trên bản đồ, nên một
waypoint có thể nằm ngay trên chúng. BendyRuler không cho phao vào đó, và phao sẽ loanh quanh mãi trước
vật cản. Pi xử lý bằng ba luật (`rescue.blocked_*` trong `Rasp_Pi/config/settings.yaml`), rồi cho mission
nhảy sang điểm kế và báo `WP n BLOCKED, SKIPPED` (tab Messages):

- LiDAR thấy vật trong vòng 1.5 m quanh chính waypoint, liên tục 3 giây: bỏ qua ngay, từ xa;
- hoặc phao đã ở trong 6 m quanh waypoint, 20 giây không lại gần thêm, và có vật cản trong 4 m;
- hoặc, dù waypoint xa bao nhiêu (giữa một tòa nhà lớn), 120 giây liền phao không lại gần hơn mức gần
  nhất đã đạt. (Từng đặt 45 giây: bộ lái có lúc chần chừ cả phút trước một vật cản rồi mới vòng qua, và luật
  này đã bỏ nhầm một waypoint không bị chắn.)

Phao đã vào trong 2.5 m quanh waypoint thì coi là đã tới (có thể đang chờ theo Delay), không tính là bị chắn.

Nếu điểm bị chắn là **điểm cuối cùng** của mission thì không còn điểm nào để nhảy tới: Pi chuyển phao sang
HOLD (`rescue.blocked_last_mode`) và báo `WP n BLOCKED, HOLD`, thay vì để phao lượn vòng mãi.

Đã thử với waypoint đặt giữa mái một căn nhà 7 × 5 m: phao lượn quanh nhà 55–106 giây rồi bỏ qua điểm đó;
khi đó là điểm cuối, phao dừng sau 75 giây.

Kiểm thử tự động (trên instance 1, chạy song song được với phiên đang mở):

```bash
bash sim/tools/flood_test.sh simulated
bash sim/tools/flood_test.sh camera
```

Đạt khi: không lúc nào tâm phao cách vật cản, nhà hay người dưới 0.8 m; có lượn khỏi tim đường để qua
cây; báo đủ hai người, tọa độ báo về lệch không quá 3 m; không bỏ qua waypoint nào; về tới điểm xuất phát.

Giới hạn đã biết của cảnh này:

- **Vật thấp hơn mặt quét LiDAR (0.37 m) thì phao không thấy**: khúc gỗ trôi, dây, vật chìm. Mọi vật cản
  trong cảnh đều nhô ít nhất 0.5 m.
- **Khe hẹp**: BendyRuler giữ lề 1 m (`OA_MARGIN_MAX`) cộng sai số ô 5°. Khe dưới khoảng 4 m phao không
  chui, nó tìm đường khác hoặc đứng lại.
- **Điểm đích sát vật cản**: BendyRuler dò thêm 2 m phía sau điểm đích; waypoint cách tường dưới ~3.5 m
  thì phao loanh quanh trước nó cho tới khi luật "waypoint bị chắn" ở trên bỏ qua điểm đó.
- **Quay tại chỗ**: `avoidance.parm` bật `WP_PIVOT_ANGLE 60`, cần hai ESC sau đảo chiều được. Trên phao
  thật nếu ESC sau chỉ quay một chiều thì phải thử lại.
- Nước vẫn màu xanh: lớp sóng của asv_wave_sim tự tô màu, chưa đổi được sang màu nước lũ.
- Mọi thứ trừ phao đều đứng yên (không có vật trôi).

## Nhiều người trong khung hình

Camera giữ lại **mọi** người model phát hiện trong một khung hình, và Pi ghi từng người lên bản đồ
(`Rasp_Pi/modules/rescue.py`):

- một phát hiện là người đã biết khi nó nằm đúng hướng của họ (lệch ngang không quá 1 m, hoặc 6°) và ở
  khoảng cách xấp xỉ (trong 3 m: khoảng cách đọc từ chiều cao khung kém chắc hơn hướng nhiều); không thì là
  người mới. `rescue.revisit_radius_m`, `rescue.same_person_range_m`;
- mỗi người được xác nhận riêng (3 khung hình, 1 giây) và có trạng thái riêng;
- phao tới người chưa xử lý gần nhất, xong thì tới người kế, không cần thấy lại từ đầu;
- trong lúc tiếp cận, chỉ phát hiện rơi đúng vào người đang nhắm mới làm vị trí của họ thay đổi, và camera
  bám người đó (kể cả quay về phía họ khi họ chưa ở trong khung hình);
- người đã xử lý, người chưa tới lượt và người vừa bị mất dấu đều là vùng cấm với bộ né vật cản;
- khi có từ hai người, Pi báo `PEOPLE 2 FOUND 1 ATTENDED`.

```bash
bash sim/tools/two_person_test.sh side      # hai người đứng ngang nhau, cách nhau 3 m
bash sim/tools/two_person_test.sh behind    # một người gần, một người xa hơn 4.5 m và lệch 1.5 m
```

Đạt khi cả hai người được xử lý, mọi tọa độ báo về lệch dưới 3 m, và phao không tới gần ai dưới 0.8 m.

| | Trước (chỉ giữ một người mỗi khung hình, bán kính "người cũ" 8 m) | Sau |
|---|---|---|
| Xử lý đủ 2 người | 0 trên 6 lần chạy | 4 trên 4 |
| Tọa độ báo về lệch | 0.3–0.6 m | 0.1–0.7 m |

Còn vụng ở cảnh `side`: cả hai lần phao mất dấu người thứ hai hai lần rồi mới tới nơi (5 lần chuyển quyền lái
thay vì 2), và có lần báo cùng một người hai lần. Người thứ hai đứng cách người vừa xử lý 3 m, mà người vừa
xử lý lại là vùng cấm. Chưa thử: ba người trở lên, hai người sát nhau dưới 1 m, một người dưới nước cạnh một
người trên mái.

**Cắt ảnh theo dải chân trời** (đưa vào model hai ô cắt quanh đường chân trời thay cho cả khung hình, để mỗi
người chiếm nhiều điểm ảnh hơn) đã được đo và **không được đưa vào code**. Điểm tin cậy của model, camera đứng
yên, ngưỡng 0.60:

| Cự ly | Người bơi: cả khung / hai ô | Người trên mái: cả khung / hai ô |
|---|---|---|
| 6 m | 0.72 / 0.71 | 0.61 / 0.43 |
| 8 m | 0.66 / 0.48 | 0.64 / 0.51 |
| 10 m | 0.54 / 0.62 | 0.48 / 0.64 |
| 12 m | 0.59 / 0.57 | 0.52 / 0.54 |
| 14–16 m | 0.51–0.55 / 0.48–0.56 | 0.40–0.45 / 0.63 |

Hai ô giúp ở 10 m và với người trên mái ở 14–16 m, nhưng làm hỏng ở 8 m và với người trên mái ở gần (dải cắt
mất một phần người). Điểm của model không tăng đều theo kích thước người trong ảnh mà dao động quanh ngưỡng,
nên phóng to ảnh không phải lối ra; muốn nhìn xa hơn một cách chắc chắn thì phải huấn luyện lại model trên ảnh
thật.

## So sánh thuật toán né vật cản

Bài báo trong `papers/` (Jo, Kim, Kim, Park, *J. Mar. Sci. Eng.* 2022, 10, 2036) so sánh hai cách tránh va
chạm cho tàu mặt nước: trường thế thiên lệch (B-APF) và chướng ngại vận tốc (VO), cùng cổng rủi ro
TCPA/DCPA. Bài báo xét các tàu trong một đội hình, biết chính xác vị trí và vận tốc của nhau, và chỉ mô
phỏng. Ở đây hai cách đó được thử cho việc khác: một phao né vật cản đứng yên mà LiDAR thấy.

`Rasp_Pi/pi_steer.py` là chương trình thí nghiệm: bộ lái ở GUIDED, Pi tính hướng và tốc độ 5 lần mỗi giây
(`Rasp_Pi/modules/navigation.py`) rồi ra lệnh. Không có phát hiện người. Phao bình thường **không** chạy
kiểu này; nó dùng BendyRuler của bộ lái.

```bash
bash sim/tools/avoid_compare.sh bendyruler street   # cách hiện tại
bash sim/tools/avoid_compare.sh paper street        # B-APF của bài báo
bash sim/tools/avoid_compare.sh vo gap              # VO của bài báo, lộ trình khe hẹp
bash sim/tools/avoid_compare.sh bapf street         # trường thế của nhánh STM32 (main.py)
```

Hai lộ trình trong cảnh lũ: **dọc phố** (tới 38.5 m rồi quay về, qua cây, xe, cột điện, khoảng 75 m) và
**khe hẹp** (ba waypoint buộc phao đi giữa chiếc xe và dãy nhà bắc, khe rộng 4.3 m). Ba lần chạy mỗi ô, ghi
khoảng nhỏ nhất – lớn nhất. "Dưới 1 L" là tổng thời gian tâm phao cách vật cản dưới một chiều dài phao
(1.1 m), thước đo an toàn của bài báo.

| Lộ trình | Cách | Xong | Thời gian | Gần vật cản nhất | Dưới 1 L | Tổng góc quay mũi |
|---|---|---|---|---|---|---|
| Dọc phố | BendyRuler | 3/3 | 115–133 s | 0.63–1.09 m | 0.5–8.1 s | 1128–1532° |
| Dọc phố | B-APF bài báo | 3/3 | 144–176 s | 0.08–0.40 m | 34–56 s | 5092–6825° |
| Dọc phố | VO bài báo | 2/3 | 97–127 s | 0.48–0.95 m | 0.5–17 s | 1595–3543° |
| Dọc phố | Trường thế STM32 | 0/1 | kẹt sau 8 m | | | |
| Khe hẹp | BendyRuler | 3/3, nhưng bỏ 2 trong 3 waypoint | 56–83 s | 0.40–0.70 m | 2.3–3.3 s | 689–1503° |
| Khe hẹp | B-APF bài báo | 3/3 | 37–39 s | 1.15–1.22 m | 0 s | 608–694° |
| Khe hẹp | VO bài báo | 1/3 | 71 s | 1.24 m | 0 s | 2193° |
| Khe hẹp | Trường thế STM32 | 0/1 | kẹt | | | |

Với VO, các số là của những lần chạy xong; ba lần không xong (hết 240 s) đều lượn vòng, tổng góc quay
16 500–18 200°, và có lúc "cách vật cản" 0.02–0.08 m. Vùng tính khoảng cách tới hai dãy nhà gồm cả các hẻm
giữa nhà, nên số sát 0 có thể là phao chui vào hẻm chứ chưa chắc chạm tường; bài test không phân biệt được.

Đọc bảng này thế nào:

- **Không cách nào hơn hẳn.** BendyRuler là cách duy nhất đi hết lộ trình dọc phố cả ba lần mà không lần nào
  sát vật cản dưới 0.6 m. B-APF đi khe hẹp gọn và đều nhất, nhưng trên phố nó ở sát vật cản gần một phút và
  lắc gấp bốn. VO có lần chạy nhanh nhất trên phố, nhưng ba trong sáu lần không tới đích.
- **Giữ BendyRuler làm mặc định.** Điểm yếu của nó lộ ra ở khe hẹp: nó bỏ waypoint đặt gần vật cản thay vì
  luồn tới.
- **Trường thế của nhánh STM32 (`BapfNavigator`) kẹt vĩnh viễn** ở cả hai lộ trình: quy tắc "vật cản gần
  hơn `stop_distance_m` thì dừng" không có lối ra, đã dừng thì không rời được vật cản.
  Sau lượt đo này quy tắc đó đã được sửa: chỉ dừng khi hướng đi được lệnh dẫn vào vật cản, còn lại thì bò ra.
  Phao hết đứng im (đi 78–104 m trong 240 s) nhưng vẫn lượn vòng không tới đích, 4 trên 4 lần chạy.
- So sánh không hoàn toàn công bằng: BendyRuler chạy trong bộ lái, ba cách kia chạy trên Pi ở 5 Hz qua
  MAVLink. Hệ số của bài báo (cho tàu 4.88 m, ô 25 m) phải thu nhỏ cho phao 1.1 m trong phố rộng 12 m, qua
  hai vòng chỉnh; hệ số và lý do từng lần đổi ghi ở đầu `pi_steer.py`. VO với vật cản đứng yên cũng mất lợi
  thế chính của nó là dùng vận tốc của vật cản.
- Ba lần chạy mỗi ô là ít, và kết quả dao động mạnh giữa các lần (VO trên phố: 97 s lần này, không tới đích
  lần sau).

## Xem trên dashboard web

`run_boat.sh` cho MAVProxy phát thêm ra cổng **14551**. Cầu nối
`esp32-lora-station/tools/mavlink_bridge.py` đọc cổng đó và đẩy lên Worker như một phao `SIM-01`, có HUD
kiểu Mission Planner. Chạy Worker cục bộ (`npx wrangler dev` trong `esp32-lora-station/cloud`), rồi trong WSL:

```bash
export VEDC_INGEST_TOKEN=...   # INGEST_TOKEN trong esp32-lora-station/cloud/.dev.vars
python3 /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/esp32-lora-station/tools/mavlink_bridge.py --url http://127.0.0.1:8787/api/ingest
```

Mở `http://127.0.0.1:8787`, vào bằng chế độ khách, chọn `SIM-01`. Xem `esp32-lora-station/README.md`.

Kiểm thử tự động (không cần Mission Planner): chạy `--headless`, rồi trong WSL
`python3 sim/tools/smoke_test.py` — arm, GUIDED 20 m về phía đông, báo PASS/FAIL.

## Thư mục

| File | Nội dung |
|---|---|
| `env.sh` | Biến môi trường: ROS/Gazebo, GPU qua d3d12, đường dẫn mô hình và plugin |
| `models/vedc_buoy/` | Catamaran 1.10 × 0.60 m, hình dáng lấy từ file CAD của đội (`meshes/*.stl`): 2 motor sau trên pod lái, 1 motor trước ở giữa đảo chiều được, pod trước quay ngược pod sau. Camera 60° trên hộp mũi (`/vedc_buoy/camera`), LiDAR 360° trên cột ngắn trên nóc (`/vedc_buoy/lidar`) |
| `Thuyen.ETL.STL` | file CAD gốc của đội (mm) |
| `tools/cad_to_mesh.py` | đổi file CAD sang các mesh của mô hình (thân, 3 pod, camera): mét, x hướng mũi, y sang trái, z lên |
| `worlds/vedc_lake.sdf` | Mặt nước sóng FFT, ba phao tiêu làm vật cản, một nạn nhân cách điểm xuất phát 33 m về phía đông bắc, bước vật lý 2 ms |
| `models/victim/` | Người nổi mặc áo phao cam, một tay giơ lên; tĩnh, gốc tọa độ ở mặt nước. LiDAR của phao (mặt quét cao khoảng 0.37 m) chỉ bắt được cánh tay, nên khoảng cách tới người phải lấy từ camera |
| `models/vedc_waves/` | Sóng của asv_wave_sim trên lưới 128 ô (gốc 256) để chạy kịp thời gian thực; texture lấy từ `model://waves` |
| `params/vedc_buoy.parm` | `FRAME_CLASS 2` (tàu), gán kênh motor/pod, tốc độ, bộ lái đã chỉnh, pin, giữ vị trí, failsafe mất GCS → RTL |
| `params/avoidance.parm` | né vật cản BendyRuler bằng dữ liệu từ Pi, quay tại chỗ; chỉ nạp với `--pi` |
| `worlds/vedc_flood.sdf` | cảnh lũ: phố ngập, hai dãy nhà, cây, xe, cột điện, một người dưới nước, một người trên mái |
| `models/flooded_house/`, `flooded_house_tin/` | nhà ngập tới mái (ngói đỏ 7 × 5 m, tôn xanh 6 × 4.5 m), ghép từ khối hộp, tĩnh |
| `models/victim_standing/` | người đứng (cùng mesh Fuel), gốc tọa độ ở bàn chân, để đặt lên mái |
| `models/vedc_flood_waves/` | mặt nước của cảnh lũ: gió 1.5 m/s (sóng lăn tăn) |
| `tools/flood_test.sh` | kiểm thử trọn cảnh lũ với code Pi, trên instance 1 |
| `tools/scan_test.sh` | đo dải tìm kiếm của camera: lộ trình đi lệch bên cạnh nạn nhân, có quét hoặc camera cố định |
| `tools/two_person_test.sh` | hai người trong cùng khung hình: phải xử lý đủ cả hai |
| `tools/avoid_compare.sh` | so sánh BendyRuler với các thuật toán né vật cản chạy trên Pi (thí nghiệm) |
| `models/victim_person/` | người giống thật (mesh "Standing person" của Gazebo Fuel, CC0, tự tải lần đầu), ngập nước tới ngực |
| `tools/rescue_test.sh` | kiểm thử trọn kịch bản cứu hộ với code Pi, trên instance 1 |
| `tools/smoke_test.py` | Kiểm thử nhanh qua pymavlink: arm, GUIDED 20 m |
| `tools/integration_test.sh` | Kiểm thử đầy đủ trên instance 1 (cổng 9012/5770/14561), chạy song song được với phiên đang mở: tham số, nhiễu IMU, arm, GUIDED, chiều kênh khi rẽ |

### Ánh xạ kênh ArduPilot → phao

| SERVO | Chức năng | Trên phao | Firmware STM32 tương ứng |
|---|---|---|---|
| 1 | ThrottleLeft (73) | motor sau trái | `ESC_LEFT`, DShot PA5 |
| 2 | GroundSteering (26) | hai pod sau | `SERVO_LEFT` / `SERVO_RIGHT` |
| 3 | ThrottleRight (74) | motor sau phải | `ESC_RIGHT`, DShot PA3 |
| 4 | Throttle (70) | motor trước, 1500 µs = dừng | `ESC_FRONT`, PWM PA1 |
| 5 | GroundSteering, đảo chiều | pod trước | `SERVO_FRONT`, `RUDDER_DIR_FRONT = -1` |

Rẽ phải: motor trái mạnh hơn phải, pod sau > 1500 µs, pod trước < 1500 µs.

### Mô hình CAD của phao

Hình dáng phao trong Gazebo là file CAD của đội. Khi có bản CAD mới, xuất lại STL rồi chạy:

```bash
python3 sim/tools/cad_to_mesh.py sim/Thuyen.ETL.STL sim/models/vedc_buoy/meshes
```

File CAD để nguyên như phần mềm xuất ra: đơn vị mm, chiều dài theo trục Z (mũi ở Z nhỏ), bề ngang theo X,
chiều cao theo Y. Script xoay về hệ trục Gazebo, đổi sang mét, đặt đáy hai thân thấp hơn gốc tọa độ 5 cm,
rồi cắt khối CAD liền thành các phần chuyển động được:

| Mesh | Gắn vào | |
|---|---|---|
| `hull.stl` | thân | màu cam |
| `pod_rear_left.stl`, `pod_rear_right.stl` | hai pod sau | xoay theo servo lái |
| `pod_front.stl` | pod trước | xoay ngược pod sau |
| `camera.stl` | camera | xoay theo servo camera |

Những chỗ giả lập **khác file CAD**:

- **Motor trước hạ thấp 9 cm.** Trong CAD, tâm chân vịt trước (rộng 12 cm) chỉ thấp hơn mặt nước khoảng
  2 cm ở tải 15 kg, nửa trên nằm trên mặt nước. Giả lập đặt tâm thấp hơn gốc tọa độ 11 cm, ngang hai
  chân vịt sau, và vẽ thêm một thanh nối lên trần hầm giữa hai thân.
- **Thêm LiDAR.** CAD chưa có. Giả lập đặt nó trên cột ngắn trên nóc, phía mũi, mặt quét cao 0.37 m: vừa
  đủ vượt camera (0.305 m) và đầu anten (0.355 m). Thấp hơn thì nó quét trúng hai thứ này và coi là vật cản
  dính sát phao.

Các mesh chỉ quyết định **hình dáng nhìn thấy**. Còn lại vẫn khai trong `model.sdf`:

- **lực nổi và lực cản**: hai khối hộp 1.10 × 0.14 × 0.20 m đặt đúng chỗ hai thân. Lực nổi tính trên
  từng tam giác mỗi bước 2 ms, nên mesh hơn 9 000 tam giác của thân sẽ làm giả lập chậm hẳn;
- **khối lượng 15 kg** và quán tính;
- **vị trí motor và camera**, đo trên CAD: chân vịt sau ở (-0.40, ±0.234, -0.115) m, chân vịt trước ở
  (0.19, 0, -0.11) m sau khi hạ, camera ở (0.41, 0, 0.295) m. Vị trí các link trong `model.sdf` phải khớp
  với các hằng số ở đầu `cad_to_mesh.py`.

Chân vịt không quay trên hình (lực đẩy vẫn đúng). STL không mang màu: màu đặt trong `model.sdf`.

### Thông số module trong giả lập

| Module thật | Trong giả lập | Nguồn |
|---|---|---|
| Thân 110 × 60 cm, **15 kg** | khối lượng và quán tính của `base_link` | đội cung cấp |
| Pin 12 V, 5600 mAh | `BATT_CAPACITY`, `SIM_BATT_*`, failsafe pin yếu → RTL (giả định pin lithium 3S, đầy 12.6 V) | đội cung cấp |
| Servo pod **DS51150-12V** | tốc độ khớp pod 5 rad/s (0.21 s/60° ở 12 V) | thông số nhà sản xuất |
| Servo camera **SG90** | khớp `camera_pan_joint`, ±90°, 10 rad/s; lệnh góc (rad, dương là quay phải) trên `/model/vedc_buoy/joint/camera_pan_joint/cmd_pos` | thông số phổ biến của SG90 |
| Camera **OV9726**, ống kính **60°** | 640 × 480, góc nhìn ngang 60° (dọc 46.8° ở tỉ lệ 4:3); `settings.yaml` của Pi cũng đặt 60°. Phát 15 khung/giây: số tự chọn cho giả lập, chưa đối chiếu với camera thật | mã camera và góc: đội cung cấp |
| LiDAR **LDS-008** (loại của robot hút bụi) | 360 điểm/vòng, 5 vòng/giây theo model cùng họ LDS-006; tầm 0.15–8 m **chưa xác nhận** | ảnh nhãn + tài liệu LDS-006 |
| GPS Holybro M10 | GPS mặc định của ArduPilot SITL | — |

### Những số còn giả định

- **LiDAR LDS-008**: không có datasheet. Model cùng họ LDS-006 dùng 115200 baud, gói 22 byte bắt đầu
  bằng `0xFA` (kiểu Neato XV-11), phải gửi `startlds$` mới quay. Giao thức này khác LD14, nên
  `Rasp_Pi/modules/lidar_ld14.py` không đọc được; cần xác nhận trên module thật rồi viết bộ giải mã mới.
  Tầm đo và khả năng làm việc ngoài nắng chưa biết.
- **DS51150** có bản 180° và 270°; mô hình đang để pod ±45° trên 1000–2000 µs.
- Thân 0.14 × 0.20 m; lực đẩy ±15 N mỗi motor sau và ±10 N motor trước (motor, chân vịt chưa có thông số).
- Tốc độ chạy model trên Pi 4 (giả lập chưa giới hạn theo tốc độ Pi).

Hai chỗ khác phao thật: motor sau thật (BLHeli_S DShot) chỉ quay một chiều, ở đây đảo chiều được;
quán tính chân vịt để 0.1 kg·m² (lớn hơn thật nhiều) như mô hình BlueBoat, vì link quay nhẹ làm
bộ giải vật lý rung tới ±50 g và ArduPilot từ chối arm ("Accels inconsistent").

## Lưu ý

- **Tốc độ giả lập:** góc dưới phải cửa sổ Gazebo là real-time factor (RTF). ArduPilot chạy khóa nhịp
  với Gazebo nên RTF thấp thì phao phản ứng chậm theo. Đo trên laptop dev (headless): 0.45 với bước
  1 ms + lưới sóng 256 + barrage; khoảng 1.0 với cấu hình hiện tại. Chi phí chính là lực nổi tính trên
  từng tam giác thân tàu mỗi bước — mỗi vật nổi thêm vào thế giới đều làm chậm. Camera/LiDAR gần như
  không ảnh hưởng.
- **GPU:** WSLg mặc định render bằng CPU (llvmpipe), sóng sẽ rất giật. `env.sh` đặt
  `GALLIUM_DRIVER=d3d12` để dùng card NVIDIA. Kiểm tra: `glxinfo -B` phải ra `D3D12 (NVIDIA ...)`.
- **Sóng:** gió 2.5 m/s (sóng khoảng 0.13 m), hợp với hồ kín như Hồ Đá. Gió 5 m/s của bản gốc là sóng
  biển khoảng 0.5 m, quá lớn cho phao 1.1 m.
- **Test song song:** đừng chạy thêm một Gazebo có `ArduPilotPlugin` cổng 9002 khi `run_boat.sh` đang
  mở — ArduPilot sẽ nói chuyện nhầm sang đó. Dùng `tools/integration_test.sh` (instance 1, cổng riêng).
- Khi tắt Gazebo có thể thấy `OGRE EXCEPTION ... mHlmsGlobalIndex` — lỗi lúc hủy cảnh của Gazebo,
  vô hại.
