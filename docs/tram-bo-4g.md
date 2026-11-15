# Trạm bờ: đường 4G Cat.1 dự phòng (Ai-Thinker Ca-01)

Trạm bờ lên cloud bằng **WiFi trước**. Mất WiFi thì tự chuyển sang **4G** (module Ca-01). Cả hai cùng mất thì
bản tin phao vẫn xếp hàng chờ như trước (16 bản). Liên lạc giữa phao và trạm vẫn hoàn toàn bằng LoRa: 4G chỉ là
đường từ trạm lên Cloudflare, nên trạm đặt ở rìa vùng lũ (còn sóng di động), LoRa phủ qua vùng mất sóng.

```
Phao ──LoRa──► ESP32-S3 ──┬── WiFi (route_prio 100) ──┐
                          └── UART2 ─ Ca-01 ─ PPP 4G ─┴──► HTTPS ──► Cloudflare Worker
                                     (route_prio 20)
```

ESP32 nói chuyện với Ca-01 bằng PPP (PPPoS, thư viện `esp_modem`). 4G hiện ra như một card mạng bình thường,
nên phần HTTPS, kiểm chứng chỉ, hàng đợi và thử lại trong `main.c` không đổi.

Mã: [`esp32-lora-station/main/cellular.c`](../esp32-lora-station/main/cellular.c), thay đổi trong `main.c`
(`khoi_dong_mang`, `cho_co_mang`, `duong_mang`).

## 1. Nối dây

| ESP32-S3 | Ca-01 (cổng UART 5 V) |
|---|---|
| GPIO4 (UART2 TX) | RXD |
| GPIO5 (UART2 RX) | TXD, **qua mạch chuyển mức** |
| GND | GND |

Nguồn Ca-01 cấp riêng theo board (dòng đỉnh khi phát sóng khoảng 2 A), không lấy từ chân 3V3 của ESP32.
Bật module bằng nút PWR trên board.

> **Cảnh báo mức điện áp.** Chân GPIO của ESP32-S3 chỉ chịu tối đa 3,6 V. TXD của cổng UART 5 V đưa thẳng 5 V vào
> GPIO5 thì có thể hỏng ESP32. Chọn một trong hai cách:
> - **Module chuyển mức 2 chiều** (BSS138, loại 4 kênh): phía LV nối 3V3 của ESP32, phía HV nối 5 V của board
>   Ca-01. Đi cả TX và RX qua nó. Cách này chắc chắn nhất.
> - **Cầu chia áp** cho chiều Ca-01 → ESP32: TXD ─ 1 kΩ ─┬─ GPIO5, nhánh ┴ 2 kΩ xuống GND (5 V × 2/3 ≈ 3,3 V).
>   Chiều ESP32 → Ca-01 thì nối thẳng GPIO4 → RXD. 3,3 V thường đủ là mức cao với đầu vào 5 V có transistor
>   chuyển mức trên board. Nếu module không trả lời AT (bước 4) thì chuyển sang module chuyển mức.
>
> Nếu board Ca-01 có chân hoặc jumper chọn mức UART 3,3 V thì dùng 3,3 V và nối thẳng cả hai dây.

Các chân này không đụng tới phần đang dùng: LoRa GPIO8–13, GPS UART1 GPIO17/18, console UART0 GPIO43/44,
USB GPIO19/20.

## 2. Thử module riêng bằng máy tính (trước khi nối ESP32)

Nối Ca-01 với USB-UART (CP2102/CH340, mức 5 V hoặc qua chuyển mức), mở terminal 115200 8N1, xuống dòng **CR+LF**.
Lắp SIM có data, bấm nút PWR, chờ đèn mạng nháy.

| Gõ | Phải thấy | Nếu không |
|---|---|---|
| `AT` | `OK` | đảo TX/RX, kiểm tra baud, GND |
| `AT+CPIN?` | `+CPIN: READY` | SIM lắp sai chiều, SIM có mã PIN |
| `AT+CSQ` | `+CSQ: 10..31,99` | 99 = chưa có sóng; dưới 10 là yếu, kiểm tra anten |
| `AT+CEREG?` | `+CEREG: 0,1` hoặc `0,5` | `0,2` đang tìm mạng (chờ); `0,3` bị từ chối (SIM hết hạn, chưa đăng ký) |
| `AT+CGDCONT=1,"IP","m3-world"` | `OK` | đổi APN theo nhà mạng (bảng dưới) |
| `ATD*99#` | `CONNECT` rồi ký tự lạ (khung PPP) | `ERROR`/`NO CARRIER`: APN sai hoặc SIM không có data |

Sau `CONNECT`, chờ 1 giây, gõ `+++` (không Enter), chờ 1 giây: phải về `OK` hoặc `NO CARRIER`. Firmware
dùng đúng lệnh `ATD*99#` này, nên **bước này qua thì firmware chạy được**.

| Nhà mạng | APN |
|---|---|
| VinaPhone | `m3-world` |
| Viettel | `v-internet` |
| MobiFone | `m-wap` |

## 3. Cấu hình và nạp firmware

Trong `main/secrets.h` (chép từ `secrets.example.h` nếu chưa có), thêm:

```c
#define NET_USE_WIFI  1
#define NET_USE_CELL  1
#define CELL_APN      "m3-world"
```

Thiếu các dòng này thì mặc định là bật cả hai và APN `m3-world`. Chân, UART và baud đổi được bằng
`CELL_TX_PIN`, `CELL_RX_PIN`, `CELL_UART_NUM`, `CELL_BAUDRATE` trong `secrets.h`.

```powershell
cd esp32-lora-station
idf.py set-target esp32s3      # chỉ cần nếu build lần đầu
idf.py reconfigure             # tải esp_modem 1.4.x, áp PPP + LCP echo vào sdkconfig
idf.py -p COMx build flash monitor
```

Lần build đầu cần Internet để Component Manager tải `espressif/esp_modem` (ghi vào `managed_components/` và
`dependencies.lock`). Kiểm tra `idf.py menuconfig` → Component config → LWIP → **Enable PPP support** đang bật.

## 4. Kiểm thử

### 4.1. Chỉ 4G

Đặt `NET_USE_WIFI 0`, build, nạp. Log đúng theo thứ tự:

```
I tram_bo: WiFi dang TAT (NET_USE_WIFI 0): chi dung 4G
I tram_4g: 4G du phong: UART2 TX=GPIO4 RX=GPIO5 115200 baud, APN "m3-world"
I tram_4g: Song 4G: CSQ 21 (khoang -71 dBm)
I tram_4g: Da bam mang 4G (mang nha)
I tram_4g: Da quay so (APN "m3-world"), cho PPP cap IP...
I tram_4g: 4G da co IP: 10.x.x.x, gateway ...
I tram_bo: [CLOUD] da gui qua 4G: {"fix":0}
```

Đạt khi: có dòng `4G da co IP`, sau đó có `[CLOUD] da gui qua 4G`, và dashboard thấy trạm (vị trí GPS trạm hoặc
bản tin phao) cập nhật. Thời gian từ lúc bật tới lúc có IP thường dưới 30 giây.

| Log | Nguyên nhân |
|---|---|
| `Module 4G khong tra loi lenh AT` | dây TX/RX, GND, chưa bấm PWR, mức điện áp (mục 1) |
| `SIM chua san sang` | SIM |
| `Dang cho bam mang 4G: "+CEREG: 0,3"` | SIM bị từ chối |
| `Khong vao duoc che do data` | APN sai, SIM hết data |
| `PPP khong lay duoc IP sau 30 s` | thường là APN sai; thử lại bước 2 bằng máy tính |
| `Gui cloud qua 4G loi: ESP_ERR_HTTP_CONNECT` | có IP nhưng không ra được Internet hoặc DNS: thử SIM khác |

### 4.2. Tự chuyển WiFi ↔ 4G

Đặt lại `NET_USE_WIFI 1`, nạp. Đầu tiên log cho thấy cả WiFi và 4G cùng có IP, bản tin đi `qua WiFi`.

1. **Tắt hotspot/router WiFi.** Phải thấy `Mat ket noi WiFi ... Chuyen sang 4G.`, các bản tin tiếp theo
   `[CLOUD] da gui qua 4G` (bản đầu có thể lỗi 1 lần rồi thử lại thành công). Dashboard không bị gián đoạn quá
   khoảng 10 giây.
2. **Bật lại WiFi.** Phải thấy `Da ket noi WiFi`, các bản tin quay về `qua WiFi`.
3. **Rút anten 4G hoặc bọc module bằng giấy bạc** trong lúc WiFi tắt. Trong khoảng 30 giây (LCP echo 10 s × 3)
   phải thấy `4G mat IP` hoặc `PPP bao loi`, rồi `Mat ket noi 4G, quay so lai`. Cắm anten lại thì tự có IP lại,
   không cần reset.
4. **Reset ESP32 (nút EN) trong lúc 4G đang chạy**, không tắt module. Phải thấy
   `Module con phien 4G cu ... huy phien`, `Khoi dong lai module 4G (AT+CFUN=1,1): OK`, rồi `4G da co IP`
   sau khoảng 25 giây (đo ngày 14/11/2026: hai lần liên tiếp, 24 s).

   Vì sao phải khởi động lại module: ESP32 reset ngang thì không kịp gửi LCP Terminate, PPP phía Ca-01 vẫn coi
   phiên cũ đang mở. Sau `+++`, `ATH`, `AT+CGACT=0,1`, kể cả `AT+CFUN=0/1`, lệnh `ATD*99#` vẫn trả `CONNECT`
   nhưng PPP không lên. Chỉ `AT+CFUN=1,1` (khởi động lại module) mới quay số được ngay.

### 4.3. Lệnh từ dashboard xuống phao qua 4G

Với WiFi tắt: trên dashboard admin bật đèn PHAO-01 hoặc gửi lộ trình. Log trạm phải có `[LENH] PHAO-01: ...` hoặc
`[LO TRINH] ban gui #...` trong khoảng 5 giây (chu kỳ hỏi `/api/downlink`), rồi ACK/RTE phát LoRa như khi chạy
WiFi.

### 4.4. Chạy dài và dung lượng

Để trạm chạy bằng 4G ít nhất 2 giờ, có phao phát beacon. Ghi lại: số lần `Mat ket noi 4G`, số bản tin
`bo ban tin sau 3 lan that bai` (phải bằng 0) và dung lượng SIM đã dùng (hỏi tổng đài hoặc app nhà mạng).

Ước tính: hỏi `/api/downlink` mỗi 5 giây nay **giữ kết nối HTTPS** (không bắt tay TLS lại mỗi lần), khoảng
0,5–1 KB mỗi lần khi chưa có lộ trình, tức cỡ 10–20 MB/ngày, cộng bản tin phao. Khi đang gửi lộ trình
300 điểm, mỗi lần hỏi trả khoảng 8 KB nên dùng nhiều hơn hẳn trong lúc đó. Gói 4G vài GB/tháng là dư.

## 5. Ghi chú

- Đã thử trên phần cứng (14/11/2026, ESP-IDF v5.4.3, esp_modem 1.4.3, SIM VinaPhone `m3-world`): CSQ 31,
  bật nguồn sạch thì có IP sau khoảng 6 giây kể từ lúc quay số; chạy 3 phút với hàng đợi cloud đầy liên tục và
  53 gói LoRa thật thì không crash. Chưa thử gửi lên Worker thật (cần URL và token thật trong `secrets.h`).
- Task `nhan_lora` cần stack 6 KB: với 4 KB nó tràn stack khi hàng đợi cloud đầy (mất mạng lâu) và ESP32 tự
  khởi động lại. Đã sửa trong `main.c` (bộ đệm bỏ bản tin cũ là `static`).

- Còn WiFi thì lwIP đi WiFi vì route_prio 100 > 20 của PPP; 4G vẫn giữ kết nối ở chế độ chờ (chỉ tốn LCP echo)
  để chuyển ngay khi WiFi mất.
- DNS cố định `8.8.8.8` và `1.1.1.1` cho cả hai đường: DNS của hotspot (`192.168.x.1`) không dùng được qua 4G.
- `esp_modem` ghim bản 1.x (`main/idf_component.yml`): bản 2.x đổi API C.
