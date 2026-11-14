# Mô phỏng đội 3 phao: Chia vùng và Đội hình (2026-11-12)

Gazebo, mặt nước 60 × 36 m (`sim/worlds/vedc_wide_*.sdf`). Mỗi phao có Pi (`Rasp_Pi/main.py`), STM32F407 mô phỏng
và LoRa chung một kênh. Kế hoạch được gửi từ Worker như admin bấm trên web (`sim/tools/fleet_compare.py`).
Khoảng cách làn 6 m. Mỗi lần chạy 600 s sau khi gửi kế hoạch. Phao giữ người 20 s rồi đi tiếp.

## Ảnh (`img/`)
| File | Nội dung |
|---|---|
| `01a_…TRUOC_sua`, `01b_…SAU_sua` | Chia vùng, nước trống: trước và sau khi sửa bám làn và gán dải |
| `02a_…TRUOC_sua` | Đội hình, nước trống, code cũ: quay đầu thì phao theo đổi bên, cắt ngang phao dẫn đầu |
| `02b_…SAU_sua_lan`, `02c_…SAU_sua_NED` | Hai cách sửa: giữ bên theo làn, và độ lệch Bắc/Đông kiểu ArduPilot AP_Follow |
| `03_vatcan_chia_vung`, `04_vatcan_doi_hinh` | Có vật cản, không có người |
| `05_hoanchinh_chia_vung`, `06_hoanchinh_doi_hinh` | Mô phỏng hoàn chỉnh: vật cản + 8 người |
| `tren_xuong_*` | Camera Gazebo nhìn thẳng xuống, đầu, giữa và cuối lần chạy hoàn chỉnh |
| `web_admin_*`, `gazebo_gui_*` | Ảnh tĩnh của web admin và cửa sổ Gazebo |

Trên hình đường đi: mỗi màu là một PHAO, chấm tròn là chỗ xuất phát, mũi tên chỉ hướng đi (30 s một mũi). Nét chấm
là dải hoặc lộ trình được giao. Vòng nâu là vật cản, × là người gặp nạn, vòng tròn rỗng là chỗ phao báo VICTIM_FOUND.

## Video (`video/`), world hoàn chỉnh
| File | Nội dung | Tốc độ |
|---|---|---|
| `gazebo_tren_xuong_{chia_vung,doi_hinh}.mp4` | Camera Gazebo từ trên xuống, cả vùng | ×8 |
| `gazebo_gui_{chia_vung,doi_hinh}.mp4` | Cửa sổ Gazebo 3D, nhìn chéo từ phía Nam | ×4 |
| `web_admin_{chia_vung,doi_hinh}.mp4` | Trang admin `/admin`, tab Đội tàu | ×8 |
| `web_admin_*_phong_to.mp4` | Như trên, cắt và phóng to quanh vùng tìm kiếm | ×8 |

`so_lieu_cac_lan_chay.json`: số liệu của từng lần chạy (`fleet_compare.py report`).

## Vòng 2 (2026-11-13): chỉnh ga, né vật cản sớm, tàu dẫn đầu chậm lại

Ảnh `img/v2_*`: đường đi vẽ trên ảnh camera Gazebo nhìn từ trên xuống (`fleet_compare.py plot --bg`). Mỗi tình huống có
cặp `_a_truoc` (lần chạy ngày 12) và `_b_sau`. Dải dưới ảnh ghi số liệu chính. Video vòng 2 có hậu tố `_v2`; bản vòng 1
giữ nguyên tên.

| File | Nội dung |
|---|---|
| `v2_01_trong_chia_vung_*` | Chia vùng, nước trống |
| `v2_02_trong_doi_hinh_*` | Đội hình, nước trống; `_beacon_2s4_b_sau`: beacon 2,4 s thay vì 8 s (chỉ trong mô phỏng) |
| `v2_03_vatcan_chia_vung_*`, `v2_04_vatcan_doi_hinh_*` | Có vật cản, không có người |
| `v2_05_hoanchinh_chia_vung_*`, `v2_06_hoanchinh_doi_hinh_*` | Vật cản + 8 người |

Lần chạy vòng 2 không mở cửa sổ Gazebo. Khi có cửa sổ, mô phỏng chỉ chạy bằng khoảng 0,35 lần thời gian thực, còn Pi
vẫn tính theo đồng hồ thật, nên số liệu lệch. Video cửa sổ Gazebo `gazebo_gui_*_v2.mp4` vẫn được quay, chỉ dùng để xem
hình. Số liệu so sánh: `bao_cao.html`.

## Vòng 3 (2026-11-13): PID hướng mới

Bộ chạy của vòng 2 chạy lại với PID hướng của phiên PID (`media/simulation/pid/`), không mở cửa sổ Gazebo:
`fleet_compare.py run ... --heading-gains tuned`. Ảnh `img/v3_*.jpg` (một ảnh cho mỗi tình huống; ảnh vòng 2 là
`v2_*_b_sau.jpg`), video `*_v3.mp4` (từ trên xuống và web admin; vòng 3 không quay cửa sổ Gazebo).
`so_lieu_cac_lan_chay.json`: trường `round` là 1, 2 hoặc 3.

Chạy lại: `python3 sim/tools/fleet_compare.py run --name X --mode sectors|formation --world wide_empty|wide_obstacles|wide_full --spacing 6 [--offsets ned] [--gui]`,
rồi `fleet_compare.py plot --run ~/vedc_sim/compare/X --png X.png`.
