#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "nvs_flash.h"
#include "cJSON.h"

#include "sx127x.h"
#include "nmea_gps.h"
#include "lora_payload.h"
#include "route_downlink.h"
#include "secrets.h"   // WIFI_SSID, WIFI_PASSWORD, CLOUD_INGEST_URL, CLOUD_INGEST_TOKEN (xem secrets.example.h)

static const char *TAG = "tram_bo";

// CẤU HÌNH CHÂN LoRa - ESP32-S3
#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  11
#define LORA_CS    10
#define LORA_RST   9
#define LORA_DIO0  8

// CẤU HÌNH THÔNG SỐ LoRa (phải khớp với node phao)
#define LORA_FREQUENCY_HZ     433000000
#define LORA_SYNC_WORD         0xF3
#define LORA_SPREADING_FACTOR  9
#define LORA_BANDWIDTH_HZ      125000
#define LORA_CODING_RATE       5

// CẤU HÌNH GPS CỦA TRẠM BỜ (ATK-S1216, UART1, cùng cách nối với node cũ)
#define GPS_UART_NUM          UART_NUM_1
#define GPS_RX_PIN            18      // nối vào chân TX của GPS
#define GPS_TX_PIN            17      // nối vào chân RX của GPS
#define GPS_BAUDRATE          38400
#define STATION_REPORT_MS     10000   // chu kỳ báo vị trí trạm lên cloud

// CẤU HÌNH ĐẨY LÊN CLOUD
#define CLOUD_QUEUE_LEN        16    // số bản tin chờ gửi khi mất mạng tạm thời
#define CLOUD_HTTP_TIMEOUT_MS  8000
#define CLOUD_MAX_TRIES        3

#define JSON_MAX 512

// LỘ TRÌNH GỬI XUỐNG PHAO (docs/lora-route-downlink.md)
#define DOWNLINK_POLL_MS       5000   // chu kỳ hỏi Worker bản admin đã bấm "Gửi tới phao"
#define DOWNLINK_BODY_MAX      16384  // 300 điểm khoảng 8 KB JSON
#define LORA_TURNAROUND_MS     50     // phao chuyển sang nghe ngay khi phát xong beacon
#define LORA_TX_TIMEOUT_MS     2000   // khung 120 byte ở SF9 mất khoảng 0.5 s

typedef struct { char json[JSON_MAX]; bool station; } cloud_msg_t;   // station: vị trí trạm (/api/station), ngược lại là gói phao (/api/ingest)

static sx127x_t lora_dev;
static EventGroupHandle_t wifi_event_group;
static QueueHandle_t cloud_queue;
static route_downlink_t downlink;          // bản đang gửi, rev = 0: không có gì để gửi
static SemaphoreHandle_t downlink_lock;

// LỆNH ĐÈN admin bật trên dashboard, theo từng phao (GET /api/downlink, trường "lights").
// Phao bật relay đúng theo giá trị này mỗi lần nhận ACK; phao chưa có lệnh: tắt (0).
#define MAX_LIGHT_BUOYS 8
// mask = trường lệnh trong ACK: bit 0 đèn 1, bit 1 đèn 2, bit 2 DỪNG (F407 khóa motor tới khi admin cho chạy tiếp)
typedef struct { char id[32]; uint8_t mask; } light_cmd_t;
static light_cmd_t lights[MAX_LIGHT_BUOYS];   // dưới downlink_lock
static int light_count;

// VỊ TRÍ TRẠM cho ACK: phao mất liên lạc quá 60 s thì tự chạy về đây.
static gps_fix_t tram_fix;
static portMUX_TYPE tram_fix_mux = portMUX_INITIALIZER_UNLOCKED;
#define WIFI_CONNECTED_BIT BIT0

// ---------------------------------------------------------------- WiFi Station
// Không chặn app_main: LoRa vẫn chạy khi chưa có WiFi, gói nhận được xếp hàng chờ gửi.
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "Mat ket noi WiFi, dang thu lai...");
        esp_wifi_connect();   // thử lại mãi mãi
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Da ket noi WiFi, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void khoi_dong_wifi_sta(void) {
    wifi_event_group = xEventGroupCreate();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Dang ket noi WiFi \"%s\"...", WIFI_SSID);
}

// Làm sạch chuỗi để nhúng an toàn vào JSON (thay dấu " \ và ký tự điều khiển bằng khoảng trắng)
static void lam_sach_cho_json(char *dst, const char *src, size_t max_len) {
    size_t i = 0;
    for (; src[i] != '\0' && i < max_len - 1; i++) {
        char c = src[i];
        if (c == '"' || c == '\\' || (unsigned char)c < 0x20) {
            dst[i] = ' ';
        } else {
            dst[i] = c;
        }
    }
    dst[i] = '\0';
}

static bool json_append(char *dst, size_t capacity, size_t *used, const char *format, ...) {
    if (*used >= capacity) return false;
    va_list args;
    va_start(args, format);
    int written = vsnprintf(dst + *used, capacity - *used, format, args);
    va_end(args);
    if (written < 0 || (size_t)written >= capacity - *used) return false;
    *used += (size_t)written;
    return true;
}

// Hàng đợi đầy (mất mạng lâu): bỏ bản tin CŨ NHẤT để giữ dữ liệu mới nhất
static void xep_hang_cloud(const cloud_msg_t *m) {
    if (xQueueSend(cloud_queue, m, 0) != pdTRUE) {
        cloud_msg_t bo;
        xQueueReceive(cloud_queue, &bo, 0);
        xQueueSend(cloud_queue, m, 0);
        ESP_LOGW(TAG, "Hang doi cloud day, da bo ban tin cu nhat");
    }
}

// ------------------------------------------------------------- Sự kiện của phao (ev=...)
// Người phát hiện (V), phao lật (T), nút xác nhận lấy đồ (C). Phao phát lại tới khi trạm ACK, nên
// Worker có thể nhận trùng (id, seq): Worker bỏ bản trùng.
static void xu_ly_su_kien(const lora_event_t *ev, const char *raw, int rssi, float snr) {
    char id_sach[32], raw_sach[100];
    lam_sach_cho_json(id_sach, ev->id, sizeof(id_sach));
    lam_sach_cho_json(raw_sach, raw, sizeof(raw_sach));
    cloud_msg_t m;
    m.station = false;
    size_t used = 0U;
    char lat[24] = "null", lon[24] = "null";
    if (ev->has_position) {
        snprintf(lat, sizeof(lat), "%.6f", ev->lat);
        snprintf(lon, sizeof(lon), "%.6f", ev->lon);
    }
    bool ok = json_append(m.json, sizeof(m.json), &used,
                          "{\"id\":\"%s\",\"rssi\":%d,\"snr\":%.1f,\"raw\":\"%s\",\"event\":{\"seq\":%u,\"type\":\"%c\",\"lat\":%s,\"lon\":%s",
                          id_sach, rssi, snr, raw_sach, ev->seq, ev->type, lat, lon);
    if (ev->type == 'V') {
        ok = ok && json_append(m.json, sizeof(m.json), &used, ",\"risk\":%u,\"conf\":%.2f,\"count\":%u",
                               ev->risk, ev->confidence, ev->count);
        ESP_LOGW(TAG, "[SU KIEN] %s phat hien nguoi, muc %u, tin cay %.2f, tai %s %s", id_sach, ev->risk, ev->confidence, lat, lon);
    } else if (ev->type == 'T') {
        ok = ok && json_append(m.json, sizeof(m.json), &used, ",\"roll\":%.1f,\"pitch\":%.1f", ev->roll, ev->pitch);
        ESP_LOGE(TAG, "[SU KIEN] %s BI LAT: roll %.1f pitch %.1f, tai %s %s", id_sach, ev->roll, ev->pitch, lat, lon);
    } else {
        ok = ok && json_append(m.json, sizeof(m.json), &used, ",\"presses\":%u", ev->presses);
        ESP_LOGW(TAG, "[SU KIEN] %s: nguoi da lay do tiep te (lan nhan %u)", id_sach, ev->presses);
    }
    ok = ok && json_append(m.json, sizeof(m.json), &used, "}}");
    if (!ok) {
        ESP_LOGE(TAG, "JSON su kien vuot qua %u byte", (unsigned int)sizeof(m.json));
        return;
    }
    xep_hang_cloud(&m);
}

// ------------------------------------------------------------- Xử lý gói LoRa nhận được
// Dựng JSON đúng định dạng dashboard/Worker mong đợi rồi xếp vào hàng đợi gửi cloud.
static void xu_ly_goi_lora(const char *raw, int rssi, float snr) {
    lora_event_t ev;
    if (lora_event_parse(raw, &ev)) {
        xu_ly_su_kien(&ev, raw, rssi, snr);
        return;
    }
    lora_payload_t payload;
    if (!lora_payload_parse(raw, &payload)) {
        ESP_LOGW(TAG, "Bo goi LoRa khong hop le");
        return;
    }
    if (payload.attitude_rejected) {
        ESP_LOGW(TAG, "Bo nhom tu the (IMU) khong hop le, van gui vi tri: %s", raw);
    }

    char id_sach[32];
    lam_sach_cho_json(id_sach, payload.id, sizeof(id_sach));
    bool co_id = (id_sach[0] != '\0');

    char raw_sach[100];
    lam_sach_cho_json(raw_sach, raw, sizeof(raw_sach));

    cloud_msg_t m;
    m.station = false;
    size_t used = 0U;
    bool json_ok = json_append(m.json, sizeof(m.json), &used, "{");
    if (co_id) json_ok = json_ok && json_append(m.json, sizeof(m.json), &used, "\"id\":\"%s\",", id_sach);
    if (payload.has_position) {
        json_ok = json_ok && json_append(m.json, sizeof(m.json), &used,
                 "\"fix\":1,\"lat\":%.6f,\"lon\":%.6f,\"rssi\":%d,\"snr\":%.1f,\"raw\":\"%s\"",
                 payload.lat, payload.lon, rssi, snr, raw_sach);
        ESP_LOGI(TAG, "[NHAN] id=%s lat=%.6f lon=%.6f | RSSI=%d dBm | SNR=%.1f dB",
                 co_id ? id_sach : "(khong co)", payload.lat, payload.lon, rssi, snr);
    } else {
        json_ok = json_ok && json_append(m.json, sizeof(m.json), &used,
                 "\"fix\":0,\"rssi\":%d,\"snr\":%.1f,\"raw\":\"%s\"",
                 rssi, snr, raw_sach);
        ESP_LOGI(TAG, "[NHAN] id=%s khong co toa do hop le: %s",
                 co_id ? id_sach : "(khong co)", raw_sach);
    }
    if (payload.has_attitude) {
        json_ok = json_ok && json_append(m.json, sizeof(m.json), &used,
                 ",\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"target_yaw\":%.1f,"
                 "\"mode\":\"%c\",\"imu_ok\":%u,\"calib\":%u,\"seq\":%lu",
                 payload.roll, payload.pitch, payload.yaw, payload.target_yaw,
                 payload.mode, payload.imu_ok, payload.calibration,
                 (unsigned long)payload.sequence);
    }
    if (payload.has_route) {
        json_ok = json_ok && json_append(m.json, sizeof(m.json), &used, ",\"route\":{\"rev\":%u,\"have\":%u,\"pi\":%u}",
                 payload.route_rev, payload.route_have, payload.route_pi);
    }
    if (payload.has_victims) {
        json_ok = json_ok && json_append(m.json, sizeof(m.json), &used, ",\"victims\":%u", payload.victims);
        if (payload.victims > 0U) ESP_LOGW(TAG, "[NHAN] id=%s da bao phat hien nguoi: %u", co_id ? id_sach : "(khong co)", payload.victims);
    }
    // Hướng đi thật (GPS), tốc độ, pin, đèn: phao bỏ những trường không vừa khung hoặc chưa biết.
    if (payload.has_course) json_ok = json_ok && json_append(m.json, sizeof(m.json), &used, ",\"course\":%.1f", payload.course);
    if (payload.has_speed) json_ok = json_ok && json_append(m.json, sizeof(m.json), &used, ",\"speed\":%.2f", payload.speed);
    if (payload.has_battery) json_ok = json_ok && json_append(m.json, sizeof(m.json), &used, ",\"battery_pct\":%u", payload.battery_pct);
    if (payload.has_lights) json_ok = json_ok && json_append(m.json, sizeof(m.json), &used, ",\"lights\":%u", payload.lights);
    if (payload.has_hold) json_ok = json_ok && json_append(m.json, sizeof(m.json), &used, ",\"hold\":%u", payload.hold);
    json_ok = json_ok && json_append(m.json, sizeof(m.json), &used, "}");
    if (!json_ok) {
        ESP_LOGE(TAG, "JSON telemetry vuot qua %u byte", (unsigned int)sizeof(m.json));
        return;
    }
    xep_hang_cloud(&m);
}

// Lệnh đèn của phao id (gọi khi đang giữ downlink_lock); chưa có lệnh: tắt.
static uint8_t lenh_den(const char *id) {
    for (int i = 0; i < light_count; i++)
        if (strcmp(lights[i].id, id) == 0) return lights[i].mask;
    return 0U;
}

// ------------------------------------------------- Trả lời MỌI khung của phao
// Phao nghe 1.5 s sau mỗi khung, nên trạm phát ngay sau khi nghe phao:
// - khung RTE khi phao còn thiếu lộ trình, khung đó là beacon và đèn của phao đã đúng lệnh;
// - ngược lại ACK: số sự kiện được nhận (0 với beacon), lệnh đèn, vị trí trạm (đường về của phao).
// Phao coi mọi khung của trạm là còn liên lạc; im quá 60 s thì phao tự quay về trạm.
static void tra_loi_phao(const char *raw) {
    lora_event_t ev;
    lora_payload_t payload = { 0 };
    bool la_su_kien = lora_event_parse(raw, &ev);
    const char *id;
    if (la_su_kien) {
        id = ev.id;
    } else {
        if (!lora_payload_parse(raw, &payload) || payload.id[0] == '\0') return;
        id = payload.id;
    }
    char frame[DOWNLINK_FRAME_MAX + 1];
    size_t len = 0;
    xSemaphoreTake(downlink_lock, portMAX_DELAY);
    uint8_t den = lenh_den(id);
    uint16_t count = downlink.count;
    route_ack_t ack = { 0 };
    // RTE chỉ khi đèn và trạng thái dừng của phao đã đúng lệnh: lệnh dừng không phải chờ hết lộ trình
    bool den_dung = !payload.has_lights || payload.lights == (den & 3U);
    bool dung_dung = !payload.has_hold || payload.hold == ((den >> 2) & 1U);
    if (!la_su_kien && den_dung && dung_dung) {
        ack = (route_ack_t){ .has = payload.has_route, .rev = payload.route_rev, .have = payload.route_have, .pi = payload.route_pi };
        len = RouteDownlink_Frame(&downlink, id, &ack, frame, sizeof(frame));
    }
    xSemaphoreGive(downlink_lock);
    bool la_rte = len > 0;
    if (!la_rte) {
        portENTER_CRITICAL(&tram_fix_mux);
        gps_fix_t tram = tram_fix;
        portEXIT_CRITICAL(&tram_fix_mux);
        len = RouteDownlink_Ack(id, la_su_kien ? ev.seq : 0U, den, tram.fix_valid, tram.lat, tram.lon, frame, sizeof(frame));
        if (len == 0) return;
    }
    vTaskDelay(pdMS_TO_TICKS(LORA_TURNAROUND_MS));
    bool ok = sx127x_send_packet(&lora_dev, (const uint8_t *)frame, (uint8_t)len, LORA_TX_TIMEOUT_MS);
    sx127x_start_receive(&lora_dev);
    if (la_rte)
        ESP_LOGI(TAG, "[LO TRINH] %s -> %s (phao co %u/%u diem): %s", ok ? "da phat" : "LOI phat", id,
                 ack.has ? ack.have : 0U, count, frame);
    else
        ESP_LOGI(TAG, "[ACK] %s -> %s: %s", ok ? "da phat" : "LOI phat", id, frame);
}

// ---------------------------------------------------------------- Task nhận LoRa
static void task_nhan_lora(void *arg) {
    uint8_t buf[256];
    sx127x_start_receive(&lora_dev);

    while (true) {
        int rssi;
        float snr;
        int len = sx127x_receive_packet(&lora_dev, buf, sizeof(buf) - 1, &rssi, &snr);
        if (len > 0) {
            buf[len] = '\0';
            xu_ly_goi_lora((const char *)buf, rssi, snr);
            tra_loi_phao((const char *)buf);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// URL báo vị trí trạm suy ra từ URL gửi gói phao: .../api/ingest -> .../api/station
static void tao_url_tram(char *out, size_t n) {
    const char *u = CLOUD_INGEST_URL;
    const char *p = strstr(u, "/api/ingest");
    if (p) snprintf(out, n, "%.*s/api/station", (int)(p - u), u);
    else   snprintf(out, n, "%s", u);
}

// URL lấy lộ trình gửi xuống phao: .../api/ingest -> .../api/downlink
static void tao_url_downlink(char *out, size_t n) {
    const char *u = CLOUD_INGEST_URL;
    const char *p = strstr(u, "/api/ingest");
    if (p) snprintf(out, n, "%.*s/api/downlink", (int)(p - u), u);
    else   snprintf(out, n, "%s", u);
}

// {"rev":7,"id":"PHAO-01","loop":false,"pts":[[lat,lon],...],"lights":{"PHAO-01":1},"hold":{"PHAO-01":1}}
// -> downlink và lệnh của từng phao (đèn ở bit 0-1, dừng ở bit 2). False nếu lộ trình sai dạng; lệnh sai
// dạng thì bỏ qua từng mục.
static bool doc_downlink(const char *body, route_downlink_t *out, light_cmd_t *den, int *so_den) {
    cJSON *root = cJSON_Parse(body);
    if (root == NULL) return false;
    bool ok = false;
    memset(out, 0, sizeof(*out));
    *so_den = 0;
    const cJSON *lights_json = cJSON_GetObjectItem(root, "lights");
    const cJSON *muc;
    cJSON_ArrayForEach(muc, lights_json) {
        if (*so_den >= MAX_LIGHT_BUOYS || muc->string == NULL || strlen(muc->string) >= sizeof(den->id)) continue;
        if (!cJSON_IsNumber(muc) || muc->valueint < 0 || muc->valueint > 3) continue;
        strcpy(den[*so_den].id, muc->string);
        den[*so_den].mask = (uint8_t)muc->valueint;
        (*so_den)++;
    }
    cJSON_ArrayForEach(muc, cJSON_GetObjectItem(root, "hold")) {
        if (muc->string == NULL || strlen(muc->string) >= sizeof(den->id)) continue;
        if (!cJSON_IsNumber(muc) || (muc->valueint != 0 && muc->valueint != 1)) continue;
        int i = 0;
        while (i < *so_den && strcmp(den[i].id, muc->string) != 0) i++;
        if (i == *so_den) {
            if (*so_den >= MAX_LIGHT_BUOYS) continue;
            strcpy(den[i].id, muc->string);
            den[i].mask = 0U;
            (*so_den)++;
        }
        if (muc->valueint) den[i].mask |= 4U; else den[i].mask &= 3U;
    }
    const cJSON *rev = cJSON_GetObjectItem(root, "rev");
    if (cJSON_IsNumber(rev) && rev->valueint == 0) {
        ok = true;                                    // chưa gửi gì
    } else if (cJSON_IsNumber(rev) && rev->valueint > 0 && rev->valueint <= 0xFFFF) {
        const cJSON *id = cJSON_GetObjectItem(root, "id");
        const cJSON *loop = cJSON_GetObjectItem(root, "loop");
        const cJSON *pts = cJSON_GetObjectItem(root, "pts");
        int n = cJSON_IsArray(pts) ? cJSON_GetArraySize(pts) : -1;
        if (cJSON_IsString(id) && strlen(id->valuestring) < sizeof(out->id) && n >= 0 && n <= (int)DOWNLINK_MAX_POINTS) {
            out->rev = (uint16_t)rev->valueint;
            strcpy(out->id, id->valuestring);
            out->loop = cJSON_IsTrue(loop);
            ok = true;
            for (int i = 0; i < n && ok; i++) {
                const cJSON *pt = cJSON_GetArrayItem(pts, i);
                const cJSON *lat = cJSON_GetArrayItem(pt, 0), *lon = cJSON_GetArrayItem(pt, 1);
                ok = cJSON_IsNumber(lat) && cJSON_IsNumber(lon) && fabs(lat->valuedouble) <= 90.0 && fabs(lon->valuedouble) <= 180.0;
                if (ok) { out->lat[i] = lat->valuedouble; out->lon[i] = lon->valuedouble; }
            }
            out->count = (uint16_t)n;
        }
    }
    cJSON_Delete(root);
    if (!ok) memset(out, 0, sizeof(*out));
    return ok;
}

// ------------------------------------------------- Task lấy lộ trình admin đã bấm gửi
static void task_lay_lo_trinh(void *arg) {
    static char url[160];
    static route_downlink_t moi;                      // 5 KB: không để trên stack của task
    static light_cmd_t den_moi[MAX_LIGHT_BUOYS];
    tao_url_downlink(url, sizeof(url));
    char *body = malloc(DOWNLINK_BODY_MAX);
    if (body == NULL) { ESP_LOGE(TAG, "Khong du bo nho cho lo trinh"); vTaskDelete(NULL); return; }
    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = CLOUD_HTTP_TIMEOUT_MS,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    while (true) {
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        esp_http_client_handle_t client = esp_http_client_init(&cfg);
        esp_http_client_set_header(client, "Authorization", "Bearer " CLOUD_INGEST_TOKEN);
        int len = -1;
        if (esp_http_client_open(client, 0) == ESP_OK && esp_http_client_fetch_headers(client) >= 0 &&
            esp_http_client_get_status_code(client) == 200) {
            len = 0;
            int r;
            while (len < DOWNLINK_BODY_MAX - 1 && (r = esp_http_client_read(client, body + len, DOWNLINK_BODY_MAX - 1 - len)) > 0) len += r;
        }
        esp_http_client_cleanup(client);
        if (len >= 0) {
            body[len] = '\0';
            int so_den = 0;
            if (doc_downlink(body, &moi, den_moi, &so_den)) {
                xSemaphoreTake(downlink_lock, portMAX_DELAY);
                bool doi = moi.rev != downlink.rev;
                downlink = moi;
                for (int i = 0; i < so_den; i++) {
                    if (lenh_den(den_moi[i].id) != den_moi[i].mask)
                        ESP_LOGW(TAG, "[LENH] %s: %u (den 1 %s, den 2 %s, %s)", den_moi[i].id, den_moi[i].mask,
                                 den_moi[i].mask & 1U ? "bat" : "tat", den_moi[i].mask & 2U ? "bat" : "tat",
                                 den_moi[i].mask & 4U ? "DUNG" : "duoc chay");
                }
                memcpy(lights, den_moi, sizeof(den_moi));
                light_count = so_den;
                xSemaphoreGive(downlink_lock);
                if (doi && moi.rev) ESP_LOGW(TAG, "[LO TRINH] ban gui #%u: %u diem cho %s%s", moi.rev, moi.count, moi.id, moi.loop ? ", di vong" : "");
            } else {
                ESP_LOGW(TAG, "[LO TRINH] Worker tra ve lo trinh sai dang");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(DOWNLINK_POLL_MS));
    }
}

// ------------------------------------------------------------ Task đẩy dữ liệu lên cloud
static void task_gui_cloud(void *arg) {
    esp_http_client_config_t cfg = {
        .url = CLOUD_INGEST_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = CLOUD_HTTP_TIMEOUT_MS,
        .crt_bundle_attach = esp_crt_bundle_attach,   // xác thực chứng chỉ HTTPS của Cloudflare
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    char station_url[160];
    tao_url_tram(station_url, sizeof(station_url));
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Authorization", "Bearer " CLOUD_INGEST_TOKEN);

    cloud_msg_t m;
    while (true) {
        xQueueReceive(cloud_queue, &m, portMAX_DELAY);

        bool ok = false;
        for (int lan = 1; lan <= CLOUD_MAX_TRIES && !ok; lan++) {
            xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

            esp_http_client_set_url(client, m.station ? station_url : CLOUD_INGEST_URL);
            esp_http_client_set_post_field(client, m.json, strlen(m.json));
            esp_err_t err = esp_http_client_perform(client);
            if (err == ESP_OK) {
                int status = esp_http_client_get_status_code(client);
                if (status == 200) {
                    ok = true;
                } else if (status == 401 || status == 400) {
                    // Sai token hoặc bản tin bị từ chối: gửi lại vô ích
                    ESP_LOGE(TAG, "Cloud tu choi (HTTP %d). Kiem tra CLOUD_INGEST_TOKEN.", status);
                    break;
                } else {
                    ESP_LOGW(TAG, "Cloud tra ve HTTP %d (lan %d/%d)", status, lan, CLOUD_MAX_TRIES);
                }
            } else {
                ESP_LOGW(TAG, "Gui cloud loi: %s (lan %d/%d)", esp_err_to_name(err), lan, CLOUD_MAX_TRIES);
            }
            if (!ok) vTaskDelay(pdMS_TO_TICKS(2000));
        }
        if (ok) ESP_LOGI(TAG, "[CLOUD] da gui: %s", m.json);
        else    ESP_LOGE(TAG, "[CLOUD] bo ban tin sau %d lan that bai", CLOUD_MAX_TRIES);
    }
}

// ------------------------------------------------------------------- GPS của trạm bờ
// Đọc NMEA liên tục; mỗi STATION_REPORT_MS (và ngay khi trạng thái fix đổi) đẩy vị trí trạm lên cloud
// để dashboard vẽ trạm bờ (màu đỏ) và tính khoảng cách tới phao.
static void gui_vi_tri_tram(const gps_fix_t *fix) {
    cloud_msg_t m;
    m.station = true;
    if (fix->fix_valid) {
        snprintf(m.json, sizeof(m.json), "{\"fix\":1,\"lat\":%.6f,\"lon\":%.6f,\"sats\":%d,\"hdop\":%.1f}",
                 fix->lat, fix->lon, fix->satellites, fix->hdop);
    } else {
        snprintf(m.json, sizeof(m.json), "{\"fix\":0}");
    }
    // Hàng đợi đầy thì bỏ bản này: gói phao quan trọng hơn, bản sau 10 giây sẽ thay thế
    if (xQueueSend(cloud_queue, &m, 0) != pdTRUE) ESP_LOGW(TAG, "Hang doi cloud day, bo ban tin vi tri tram");
}

static void task_gps_tram(void *arg) {
    uart_config_t cfg = {
        .baud_rate = GPS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    gps_fix_t fix = { .fix_valid = false };
    bool da_bao_fix = false;
    TickType_t lan_bao_cuoi = 0;
    uint8_t buf[64];

    while (true) {
        int len = uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < len; i++) nmea_feed_char((char)buf[i], &fix);
        portENTER_CRITICAL(&tram_fix_mux);
        tram_fix = fix;   // ACK gửi vị trí này cho phao làm đường về
        portEXIT_CRITICAL(&tram_fix_mux);

        bool doi_trang_thai = (fix.fix_valid != da_bao_fix);
        TickType_t now = xTaskGetTickCount();
        if (doi_trang_thai || (now - lan_bao_cuoi) >= pdMS_TO_TICKS(STATION_REPORT_MS)) {
            if (doi_trang_thai) {
                if (fix.fix_valid) ESP_LOGI(TAG, "GPS tram: da co fix (%d ve tinh)", fix.satellites);
                else               ESP_LOGW(TAG, "GPS tram: mat fix");
            }
            da_bao_fix = fix.fix_valid;
            lan_bao_cuoi = now;
            if (nmea_debug_so_byte_da_nhan() < 10) {
                ESP_LOGW(TAG, "GPS tram: chua nhan duoc byte nao, kiem tra day GPS (RX=GPIO%d, TX=GPIO%d, %d baud)",
                         GPS_RX_PIN, GPS_TX_PIN, GPS_BAUDRATE);
            }
            gui_vi_tri_tram(&fix);
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Khoi dong tram bo (ESP32-S3) -> Cloudflare...");

    cloud_queue = xQueueCreate(CLOUD_QUEUE_LEN, sizeof(cloud_msg_t));
    downlink_lock = xSemaphoreCreateMutex();

    khoi_dong_wifi_sta();   // không chặn: WiFi kết nối ở nền

    // GPS trạm và cloud chạy dù LoRa chưa lên: vị trí trạm vẫn lên dashboard trong lúc sửa dây.
    xTaskCreate(task_gps_tram, "gps_tram", 3072, NULL, 3, NULL);
    xTaskCreate(task_gui_cloud, "gui_cloud", 8192, NULL, 4, NULL);
    xTaskCreate(task_lay_lo_trinh, "lay_lo_trinh", 6144, NULL, 3, NULL);

    // Không tìm thấy module LoRa thì thử lại mỗi 3 s thay vì dừng hẳn: module cấp nguồn muộn,
    // hay dây cắm lại khi trạm đang chạy, vẫn được nhận.
    for (int lan = 1; !sx127x_init(&lora_dev, SPI2_HOST, LORA_SCK, LORA_MISO, LORA_MOSI,
                                   LORA_CS, LORA_RST, LORA_DIO0, LORA_FREQUENCY_HZ); lan++) {
        ESP_LOGE(TAG, "Khong khoi tao duoc LoRa (lan %d), thu lai sau 3 s", lan);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    sx127x_set_sync_word(&lora_dev, LORA_SYNC_WORD);
    sx127x_set_spreading_factor(&lora_dev, LORA_SPREADING_FACTOR);
    sx127x_set_bandwidth(&lora_dev, LORA_BANDWIDTH_HZ);
    sx127x_set_coding_rate(&lora_dev, LORA_CODING_RATE);
    sx127x_set_crc(&lora_dev, true);   // khung lộ trình gửi xuống phao có CRC như beacon của phao

    xTaskCreate(task_nhan_lora, "nhan_lora", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "San sang. Dang cho tin hieu LoRa...");
}
