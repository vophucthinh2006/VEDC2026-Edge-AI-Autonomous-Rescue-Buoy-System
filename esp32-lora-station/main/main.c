#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "nvs_flash.h"

#include "sx127x.h"
#include "nmea_gps.h"
#include "lora_payload.h"
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

#define JSON_MAX 384

typedef struct { char json[JSON_MAX]; bool station; } cloud_msg_t;   // station: vị trí trạm (/api/station), ngược lại là gói phao (/api/ingest)

static sx127x_t lora_dev;
static EventGroupHandle_t wifi_event_group;
static QueueHandle_t cloud_queue;
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

// ------------------------------------------------------------- Xử lý gói LoRa nhận được
// Dựng JSON đúng định dạng dashboard/Worker mong đợi rồi xếp vào hàng đợi gửi cloud.
static void xu_ly_goi_lora(const char *raw, int rssi, float snr) {
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
    if (payload.has_victims) {
        json_ok = json_ok && json_append(m.json, sizeof(m.json), &used, ",\"victims\":%u", payload.victims);
        if (payload.victims > 0U) ESP_LOGW(TAG, "[NHAN] id=%s da bao phat hien nguoi: %u", co_id ? id_sach : "(khong co)", payload.victims);
    }
    json_ok = json_ok && json_append(m.json, sizeof(m.json), &used, "}");
    if (!json_ok) {
        ESP_LOGE(TAG, "JSON telemetry vuot qua %u byte", (unsigned int)sizeof(m.json));
        return;
    }

    // Hàng đợi đầy (mất mạng lâu): bỏ bản tin CŨ NHẤT để giữ dữ liệu mới nhất
    if (xQueueSend(cloud_queue, &m, 0) != pdTRUE) {
        cloud_msg_t bo;
        xQueueReceive(cloud_queue, &bo, 0);
        xQueueSend(cloud_queue, &m, 0);
        ESP_LOGW(TAG, "Hang doi cloud day, da bo ban tin cu nhat");
    }
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

    khoi_dong_wifi_sta();   // không chặn: WiFi kết nối ở nền

    if (!sx127x_init(&lora_dev, SPI2_HOST, LORA_SCK, LORA_MISO, LORA_MOSI,
                      LORA_CS, LORA_RST, LORA_DIO0, LORA_FREQUENCY_HZ)) {
        ESP_LOGE(TAG, "Khong khoi tao duoc LoRa! Dung chuong trinh.");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    sx127x_set_sync_word(&lora_dev, LORA_SYNC_WORD);
    sx127x_set_spreading_factor(&lora_dev, LORA_SPREADING_FACTOR);
    sx127x_set_bandwidth(&lora_dev, LORA_BANDWIDTH_HZ);
    sx127x_set_coding_rate(&lora_dev, LORA_CODING_RATE);

    xTaskCreate(task_gps_tram, "gps_tram", 3072, NULL, 3, NULL);
    xTaskCreate(task_gui_cloud, "gui_cloud", 8192, NULL, 4, NULL);
    xTaskCreate(task_nhan_lora, "nhan_lora", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "San sang. Dang cho tin hieu LoRa...");
}
