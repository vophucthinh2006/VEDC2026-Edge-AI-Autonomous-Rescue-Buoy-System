// Đường mạng 4G Cat.1 dự phòng cho trạm bờ: Ai-Thinker Ca-01 nối UART2, chạy PPP (PPPoS) qua esp_modem.
// 4G hiện ra như một card mạng esp_netif bình thường, nên mã HTTPS trong main.c không đổi. WiFi có độ ưu tiên
// route cao hơn (100 so với 20 của PPP): còn WiFi thì đi WiFi, mất WiFi thì lwIP tự chuyển sang 4G.
//
// Trình tự mỗi lần quay số (cùng các lệnh AT trong bảng của Ca-01):
//   AT -> AT+CPIN? -> AT+CSQ -> chờ AT+CEREG? = 1/5 -> AT+CGDCONT=1,"IP",APN -> ATD*99# (CONNECT) -> PPP cấp IP
// Mất kết nối (LCP echo hỏng, nhà mạng ngắt, mất sóng) thì +++ về chế độ lệnh rồi làm lại từ đầu.
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "driver/uart.h"
#include "esp_modem_api.h"

#include "cellular.h"
#include "secrets.h"

// ------------------------------------------------------------------ Cấu hình (ghi đè được trong secrets.h)
#ifndef CELL_APN
#define CELL_APN            "m3-world"     // VinaPhone; Viettel "v-internet", MobiFone "m-wap"
#endif
#ifndef CELL_UART_NUM
#define CELL_UART_NUM       UART_NUM_2     // UART0: console, UART1: GPS trạm
#endif
#ifndef CELL_TX_PIN
#define CELL_TX_PIN         4              // ESP32 TX -> RXD của Ca-01
#endif
#ifndef CELL_RX_PIN
#define CELL_RX_PIN         5              // ESP32 RX <- TXD của Ca-01 (QUA MẠCH CHUYỂN MỨC nếu UART module là 5 V)
#endif
#ifndef CELL_BAUDRATE
#define CELL_BAUDRATE       115200
#endif

#define CELL_REG_TIMEOUT_MS  120000   // chờ bám mạng tối đa (AT+CEREG?)
#define CELL_PPP_TIMEOUT_MS  30000    // sau CONNECT, chờ PPP cấp IP tối đa
#define CELL_RETRY_MS        10000    // nghỉ giữa hai lần thử khi lỗi
#define CELL_AT_OUT_MAX      128      // >= CONFIG_ESP_MODEM_C_API_STR_MAX

#define PPP_GOT_IP_BIT  BIT0
#define PPP_LOST_BIT    BIT1

static const char *TAG = "tram_4g";

static EventGroupHandle_t s_net_eg;     // event group của main.c
static EventBits_t s_up_bit;
static EventGroupHandle_t s_ppp_eg;     // sự kiện PPP cho task quay số
static esp_modem_dce_t *s_dce;

void net_use_public_dns(esp_netif_t *netif) {
    if (netif == NULL) return;
    esp_netif_dns_info_t dns = { 0 };
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = ESP_IP4TOADDR(8, 8, 8, 8);
    esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns);
    dns.ip.u_addr.ip4.addr = ESP_IP4TOADDR(1, 1, 1, 1);
    esp_netif_set_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns);
}

static void on_ppp_ip(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (id == IP_EVENT_PPP_GOT_IP) {
        const ip_event_got_ip_t *e = (const ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "4G da co IP: " IPSTR ", gateway " IPSTR, IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw));
        net_use_public_dns(e->esp_netif);
        xEventGroupSetBits(s_net_eg, s_up_bit);
        xEventGroupSetBits(s_ppp_eg, PPP_GOT_IP_BIT);
    } else if (id == IP_EVENT_PPP_LOST_IP) {
        ESP_LOGW(TAG, "4G mat IP");
        xEventGroupClearBits(s_net_eg, s_up_bit);
        xEventGroupSetBits(s_ppp_eg, PPP_LOST_BIT);
    }
}

// Lỗi PPP (mã 1..0xFF): nhà mạng từ chối, LCP echo hỏng, mất sóng...; các mã từ 0x100 là pha PPP, chỉ để log
static void on_ppp_status(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (id > NETIF_PPP_ERRORNONE && id < NETIF_PP_PHASE_OFFSET) {
        ESP_LOGW(TAG, "PPP bao loi %ld", (long)id);
        xEventGroupClearBits(s_net_eg, s_up_bit);
        xEventGroupSetBits(s_ppp_eg, PPP_LOST_BIT);
    }
}

// AT+CEREG? -> "+CEREG: <n>,<stat>[,...]". stat 1 = mạng nhà, 5 = roaming
static bool cho_bam_mang(void) {
    char out[CELL_AT_OUT_MAX];
    TickType_t bat_dau = xTaskGetTickCount();
    int lan = 0;
    while ((xTaskGetTickCount() - bat_dau) < pdMS_TO_TICKS(CELL_REG_TIMEOUT_MS)) {
        out[0] = '\0';
        if (esp_modem_at(s_dce, "AT+CEREG?", out, 2000) == ESP_OK) {
            const char *phay = strchr(out, ',');
            int stat = phay ? atoi(phay + 1) : -1;
            if (stat == 1 || stat == 5) {
                ESP_LOGI(TAG, "Da bam mang 4G (%s)", stat == 1 ? "mang nha" : "roaming");
                return true;
            }
            if (lan++ % 5 == 0) ESP_LOGI(TAG, "Dang cho bam mang 4G: \"%s\" (2 = dang tim, 3 = bi tu choi)", out);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    return false;
}

// ESP32 reset giữa phiên 4G: +++ đưa module về chế độ lệnh nhưng PDP context 1 vẫn mở, nên AT+CGDCONT / ATD*99#
// trả ERROR tới khi module tự hủy phiên (khoảng 1 phút, đo trên Ca-01). Gác máy và hủy context để quay số lại ngay.
// manh: phiên bị bỏ ngang (ESP32 reset, không kịp gửi LCP Terminate) thì PPP phía module vẫn coi là đang mở,
// ATH, CGACT, kể cả tắt/bật sóng AT+CFUN=0/1 đều chưa đủ (đo trên Ca-01): khởi động lại module (AT+CFUN=1,1).
static void huy_phien_cu(bool manh) {
    char out[CELL_AT_OUT_MAX];
    esp_modem_at(s_dce, "ATH", out, 3000);
    esp_modem_at(s_dce, "AT+CGACT=0,1", out, 10000);
    if (manh) {
        esp_err_t r = esp_modem_at(s_dce, "AT+CFUN=1,1", out, 5000);
        ESP_LOGI(TAG, "Khoi dong lai module 4G (AT+CFUN=1,1): %s", r == ESP_OK ? "OK" : "khong tra loi OK");
        vTaskDelay(pdMS_TO_TICKS(8000));
        for (int i = 0; i < 30 && esp_modem_sync(s_dce) != ESP_OK; i++) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // Quay số ngay sau khi hủy (1,6 s) thì Ca-01 trả CONNECT nhưng PPP không lên; chờ khoảng 10 s thì được
    vTaskDelay(pdMS_TO_TICKS(6000));
}

static void task_4g(void *arg) {
    int lan_quay_loi = 0;
    int lan_khong_tra_loi = 0;
    while (true) {
        xEventGroupClearBits(s_ppp_eg, PPP_GOT_IP_BIT | PPP_LOST_BIT);

        // 1. Module có trả lời AT không. ESP32 vừa reset mà module vẫn ở chế độ data của lần trước thì AT
        //    không được trả lời: +++ đưa module về chế độ lệnh rồi thử lại.
        if (esp_modem_sync(s_dce) != ESP_OK) {
            // Gửi +++ thô (cần 1 s im lặng trước và sau). Không dùng esp_modem_set_mode(DETECT/COMMAND): nó bật
            // rồi tắt netif PPP, sau đó lần quay số đầu được CONNECT mà PPP không lên (đo trên Ca-01).
            char out[CELL_AT_OUT_MAX];
            vTaskDelay(pdMS_TO_TICKS(1100));
            esp_modem_at_raw(s_dce, "+++", out, "OK", "ERROR", 2500);
            vTaskDelay(pdMS_TO_TICKS(1100));
            if (esp_modem_sync(s_dce) == ESP_OK) {
                ESP_LOGI(TAG, "Module con phien 4G cu (ESP32 vua reset): huy phien roi quay so lai");
                huy_phien_cu(true);
            }
            if (esp_modem_sync(s_dce) != ESP_OK) {
                if (lan_khong_tra_loi++ % 6 == 0)
                    ESP_LOGW(TAG, "Module 4G khong tra loi lenh AT (UART%d, TX=GPIO%d, RX=GPIO%d, %d baud). "
                                  "Kiem tra: TX/RX dao cheo, GND chung, module da bat (nut PWR), muc dien ap UART.",
                             CELL_UART_NUM, CELL_TX_PIN, CELL_RX_PIN, CELL_BAUDRATE);
                vTaskDelay(pdMS_TO_TICKS(CELL_RETRY_MS));
                continue;
            }
        }
        if (lan_khong_tra_loi) ESP_LOGI(TAG, "Module 4G da tra loi AT");
        lan_khong_tra_loi = 0;

        // 2. SIM
        bool sim_ok = false;
        if (esp_modem_read_pin(s_dce, &sim_ok) != ESP_OK || !sim_ok) {
            ESP_LOGE(TAG, "SIM chua san sang (AT+CPIN? khong ra READY): kiem tra the SIM, ma PIN");
            vTaskDelay(pdMS_TO_TICKS(CELL_RETRY_MS));
            continue;
        }

        // 3. Sóng: CSQ 0..31 -> -113 + 2*CSQ dBm, 99 = chưa đo được
        int rssi = 99, ber = 99;
        if (esp_modem_get_signal_quality(s_dce, &rssi, &ber) == ESP_OK) {
            if (rssi >= 0 && rssi <= 31)
                ESP_LOGI(TAG, "Song 4G: CSQ %d (khoang %d dBm)%s", rssi, -113 + 2 * rssi, rssi < 10 ? ", YEU" : "");
            else
                ESP_LOGW(TAG, "Song 4G: chua do duoc (CSQ %d)", rssi);
        }

        // 4. Bám mạng
        if (!cho_bam_mang()) {
            ESP_LOGW(TAG, "Chua bam duoc mang 4G sau %d s, thu lai", CELL_REG_TIMEOUT_MS / 1000);
            vTaskDelay(pdMS_TO_TICKS(CELL_RETRY_MS));
            continue;
        }

        // 5. Quay số: esp_modem gửi AT+CGDCONT=1,"IP",APN rồi ATD*99#, module trả CONNECT và PPP bắt đầu.
        //    Xóa cờ ngay trước: dừng phiên cũ ở trên sinh sự kiện "User interrupt" (PPP_LOST_BIT).
        xEventGroupClearBits(s_ppp_eg, PPP_GOT_IP_BIT | PPP_LOST_BIT);
        if (esp_modem_set_mode(s_dce, ESP_MODEM_MODE_DATA) != ESP_OK) {
            ESP_LOGW(TAG, "Khong vao duoc che do data (AT+CGDCONT / ATD*99#), APN \"%s\": huy phien cu roi thu lai",
                     CELL_APN);
            huy_phien_cu(lan_quay_loi > 0);   // lỗi lần thứ hai liên tiếp thì tắt/bật sóng
            // Lần đầu thử lại ngay sau khi hủy phiên; lỗi tiếp thì có thể APN sai, nghỉ như thường
            vTaskDelay(pdMS_TO_TICKS(lan_quay_loi++ == 0 ? 2000 : CELL_RETRY_MS));
            continue;
        }
        lan_quay_loi = 0;
        ESP_LOGI(TAG, "Da quay so (APN \"%s\"), cho PPP cap IP...", CELL_APN);

        EventBits_t b = xEventGroupWaitBits(s_ppp_eg, PPP_GOT_IP_BIT | PPP_LOST_BIT, pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(CELL_PPP_TIMEOUT_MS));
        if ((b & PPP_GOT_IP_BIT) && !(b & PPP_LOST_BIT)) {
            // Chạy tới khi mất kết nối
            xEventGroupWaitBits(s_ppp_eg, PPP_LOST_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
            ESP_LOGW(TAG, "Mat ket noi 4G, quay so lai");
        } else {
            ESP_LOGW(TAG, "PPP khong lay duoc IP sau %d s", CELL_PPP_TIMEOUT_MS / 1000);
        }
        xEventGroupClearBits(s_net_eg, s_up_bit);
        esp_modem_set_mode(s_dce, ESP_MODEM_MODE_COMMAND);   // dừng PPP, +++ về chế độ lệnh
        huy_phien_cu(false);   // +++ không đóng PDP context: không hủy thì lần quay sau CONNECT mà PPP không lên
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void cellular_start(EventGroupHandle_t eg, EventBits_t up_bit) {
    s_net_eg = eg;
    s_up_bit = up_bit;
    s_ppp_eg = xEventGroupCreate();

    esp_modem_dte_config_t dte_cfg = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_cfg.uart_config.port_num = CELL_UART_NUM;
    dte_cfg.uart_config.tx_io_num = CELL_TX_PIN;
    dte_cfg.uart_config.rx_io_num = CELL_RX_PIN;
    dte_cfg.uart_config.rts_io_num = UART_PIN_NO_CHANGE;
    dte_cfg.uart_config.cts_io_num = UART_PIN_NO_CHANGE;
    dte_cfg.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_NONE;
    dte_cfg.uart_config.baud_rate = CELL_BAUDRATE;
    dte_cfg.uart_config.rx_buffer_size = 4096;
    dte_cfg.uart_config.tx_buffer_size = 2048;
    dte_cfg.uart_config.event_queue_size = 30;
    dte_cfg.task_stack_size = 4096;
    dte_cfg.task_priority = 5;
    dte_cfg.dte_buffer_size = 2048;

    esp_modem_dce_config_t dce_cfg = ESP_MODEM_DCE_DEFAULT_CONFIG(CELL_APN);
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_PPP();   // route_prio 20 < WiFi STA 100
    esp_netif_t *netif = esp_netif_new(&netif_cfg);
    if (netif == NULL) { ESP_LOGE(TAG, "Khong tao duoc netif PPP"); return; }

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, on_ppp_ip, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_LOST_IP, on_ppp_ip, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID, on_ppp_status, NULL));

    s_dce = esp_modem_new_dev(ESP_MODEM_DCE_GENERIC, &dte_cfg, &dce_cfg, netif);
    if (s_dce == NULL) { ESP_LOGE(TAG, "Khong khoi tao duoc esp_modem tren UART%d", CELL_UART_NUM); return; }

    xTaskCreate(task_4g, "tram_4g", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "4G du phong: UART%d TX=GPIO%d RX=GPIO%d %d baud, APN \"%s\"",
             CELL_UART_NUM, CELL_TX_PIN, CELL_RX_PIN, CELL_BAUDRATE, CELL_APN);
}
