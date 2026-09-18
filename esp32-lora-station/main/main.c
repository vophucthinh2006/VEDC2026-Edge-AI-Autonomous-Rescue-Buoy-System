#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"

#include "sx127x.h"

static const char *TAG = "web_dashboard";

// CONNECT WIFI
#define WIFI_SSID     "khanh tran"
#define WIFI_PASSWORD "1234567890"
#define WIFI_MAX_RETRY 10

// CẤU HÌNH CHÂN LoRa - ESP32-S3
#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  11
#define LORA_CS    10
#define LORA_RST   9
#define LORA_DIO0  8

// CẤU HÌNH THÔNG SỐ LoRa
#define LORA_FREQUENCY_HZ     433000000
#define LORA_SYNC_WORD         0xF3
#define LORA_SPREADING_FACTOR  9
#define LORA_BANDWIDTH_HZ      125000
#define LORA_CODING_RATE       5

static sx127x_t lora_dev;
static httpd_handle_t http_server = NULL;
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// File dashboard.html được nhúng vào firmware qua EMBED_TXTFILES
extern const uint8_t dashboard_html_start[] asm("_binary_dashboard_html_start");
extern const uint8_t dashboard_html_end[]   asm("_binary_dashboard_html_end");

//  WiFi Station 
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {
    static int retry_count = 0;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGW(TAG, "Mat ket noi WiFi, dang thu lai (%d/%d)...", retry_count, WIFI_MAX_RETRY);
        } else {
            ESP_LOGE(TAG, "Khong the ket noi WiFi sau %d lan thu. Kiem tra lai SSID/mat khau.", WIFI_MAX_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Da ket noi WiFi! Dia chi IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Mo trinh duyet vao: http://" IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "========================================");
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void khoi_dong_wifi_sta(void) {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(nvs_flash_init());
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
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

//  Nhận diện lat/lon linh hoạt trong bản tin text 
// Quét các trường phân cách bởi dấu phẩy, bỏ qua trường chứa '=' (dạng key=value),
// lấy 2 số thập phân (chứa dấu '.') đầu tiên tìm được làm lat va lon.
static bool tach_lat_lon(const char *msg, double *lat, double *lon) {
    char buf[128];
    strncpy(buf, msg, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    double gia_tri[2];
    int so_gia_tri_tim_duoc = 0;

    char *token = strtok(buf, ",");
    while (token != NULL && so_gia_tri_tim_duoc < 2) {
        bool co_dau_bang = (strchr(token, '=') != NULL);
        bool co_dau_cham = (strchr(token, '.') != NULL);

        if (!co_dau_bang && co_dau_cham) {
            char *end_ptr;
            double val = strtod(token, &end_ptr);
            // Chấp nhận nếu strtod đọc được số hợp lệ (end_ptr khác vị trí bắt đầu)
            if (end_ptr != token) {
                gia_tri[so_gia_tri_tim_duoc++] = val;
            }
        }
        token = strtok(NULL, ",");
    }

    if (so_gia_tri_tim_duoc == 2) {
        *lat = gia_tri[0];
        *lon = gia_tri[1];
        return true;
    }
    return false;
}

// Nhận diện ID phao trong bản tin (nếu máy phát có gửi)
static void tach_id(const char *msg, char *id_out, size_t max_len) {
    char buf[128];
    strncpy(buf, msg, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    id_out[0] = '\0';

    char *token = strtok(buf, ",");
    while (token != NULL) {
        while (*token == ' ') token++; // bỏ khoảng trắng đầu token

        if (strncasecmp(token, "id=", 3) == 0) {
            const char *gia_tri = token + 3;
            size_t i = 0;
            for (; gia_tri[i] != '\0' && i < max_len - 1; i++) {
                id_out[i] = gia_tri[i];
            }
            id_out[i] = '\0';
            return;
        }
        token = strtok(NULL, ",");
    }
}

// Làm sạch chuỗi để nhúng an toàn vào JSON (thay dấu " và ký tự điều khiển bằng khoảng trắng)
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

// WebSocket broadcast
static void ws_gui_toi_tat_ca_client(const char *json_msg) {
    if (!http_server) return;
    size_t max_clients = 4;
    int client_fds[4];
    if (httpd_get_client_list(http_server, &max_clients, client_fds) != ESP_OK) return;

    httpd_ws_frame_t frame = {
        .final = true, .fragmented = false, .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)json_msg, .len = strlen(json_msg),
    };

    for (size_t i = 0; i < max_clients; i++) {
        int sock = client_fds[i];
        if (httpd_ws_get_fd_info(http_server, sock) == HTTPD_WS_CLIENT_WEBSOCKET) {
            httpd_ws_send_frame_async(http_server, sock, &frame);
        }
    }
}

// HTTP handlers
static esp_err_t handler_trang_chu(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    size_t len = dashboard_html_end - dashboard_html_start;
    return httpd_resp_send(req, (const char *)dashboard_html_start, len);
}

static esp_err_t handler_websocket(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Client WebSocket moi ket noi (fd=%d)", httpd_req_to_sockfd(req));
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    httpd_ws_recv_frame(req, &ws_pkt, 0);
    return ESP_OK;
}

static void khoi_dong_http_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 4; // LWIP mac dinh chi cho toi da 7 sau khi tru 3 socket noi bo cua httpd
    config.lru_purge_enable = true;

    ESP_ERROR_CHECK(httpd_start(&http_server, &config));

    httpd_uri_t uri_trang_chu = { .uri = "/", .method = HTTP_GET, .handler = handler_trang_chu };
    httpd_register_uri_handler(http_server, &uri_trang_chu);

    httpd_uri_t uri_ws = {
        .uri = "/ws", .method = HTTP_GET, .handler = handler_websocket, .is_websocket = true,
    };
    httpd_register_uri_handler(http_server, &uri_ws);

    ESP_LOGI(TAG, "HTTP server + WebSocket da san sang tren cong 80");
}

// Xử lý gói LoRa nhận được 
static void xu_ly_goi_lora(const char *raw, int rssi, float snr) {
    double lat, lon;
    bool co_toa_do = tach_lat_lon(raw, &lat, &lon);

    char id_tho[32];
    tach_id(raw, id_tho, sizeof(id_tho));
    char id_sach[32];
    lam_sach_cho_json(id_sach, id_tho, sizeof(id_sach));
    bool co_id = (id_sach[0] != '\0');

    char raw_sach[100];
    lam_sach_cho_json(raw_sach, raw, sizeof(raw_sach));

    char id_json[48];
    if (co_id) {
        snprintf(id_json, sizeof(id_json), "\"id\":\"%s\",", id_sach);
    } else {
        id_json[0] = '\0'; // Khong co id -> dashboard tu gan "PHAO-01"
    }

    char json_msg[280];
    if (co_toa_do) {
        snprintf(json_msg, sizeof(json_msg),
                 "{%s\"fix\":1,\"lat\":%.6f,\"lon\":%.6f,\"rssi\":%d,\"snr\":%.1f,\"raw\":\"%s\"}",
                 id_json, lat, lon, rssi, snr, raw_sach);
        ESP_LOGI(TAG, "[NHAN] id=%s lat=%.6f lon=%.6f | RSSI=%d dBm | SNR=%.1f dB",
                 co_id ? id_sach : "(khong co)", lat, lon, rssi, snr);
    } else {
        snprintf(json_msg, sizeof(json_msg),
                 "{%s\"fix\":0,\"rssi\":%d,\"snr\":%.1f,\"raw\":\"%s\"}",
                 id_json, rssi, snr, raw_sach);
        ESP_LOGI(TAG, "[NHAN] id=%s khong tim thay toa do hop le trong ban tin: %s",
                 co_id ? id_sach : "(khong co)", raw_sach);
    }

    ws_gui_toi_tat_ca_client(json_msg);
}

//  Task nhận LoRa 
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

void app_main(void) {
    ESP_LOGI(TAG, "Khoi dong web dashboard (ESP32-S3)...");

    khoi_dong_wifi_sta();

    if (!sx127x_init(&lora_dev, SPI2_HOST, LORA_SCK, LORA_MISO, LORA_MOSI,
                      LORA_CS, LORA_RST, LORA_DIO0, LORA_FREQUENCY_HZ)) {
        ESP_LOGE(TAG, "Khong khoi tao duoc LoRa! Dung chuong trinh.");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    sx127x_set_sync_word(&lora_dev, LORA_SYNC_WORD);
    sx127x_set_spreading_factor(&lora_dev, LORA_SPREADING_FACTOR);
    sx127x_set_bandwidth(&lora_dev, LORA_BANDWIDTH_HZ);
    sx127x_set_coding_rate(&lora_dev, LORA_CODING_RATE);

    khoi_dong_http_server();

    xTaskCreate(task_nhan_lora, "nhan_lora", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "San sang. Dang cho tin hieu LoRa...");
}
