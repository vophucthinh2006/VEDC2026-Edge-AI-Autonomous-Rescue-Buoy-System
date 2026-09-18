#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "sx127x.h"
#include "nmea_gps.h"

static const char *TAG = "gpio_gps_lora";

// ================= CẤU HÌNH CHÂN LoRa SX1278 (RA-02) =================
#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  11
#define LORA_CS    10
#define LORA_RST   9
#define LORA_DIO0  8
#define LORA_FREQUENCY_HZ 433000000
#define LORA_SYNC_WORD     0xF3
#define LORA_SPREADING_FACTOR 9
#define LORA_BANDWIDTH_HZ  125000
#define LORA_CODING_RATE   5

// ================= CẤU HÌNH CHÂN GPS ATK-S1216 (UART1) =================
#define GPS_UART_NUM  UART_NUM_1
#define GPS_RX_PIN    18     // nối vào chân TX của ATK-S1216
#define GPS_TX_PIN    17     // nối vào chân RX của ATK-S1216
#define GPS_BAUDRATE  38400  // baud thuc te cua ATK-S1216 (da xac nhan hoat dong)

// ================= CẤU HÌNH CHÂN TRIGGER (tín hiệu kích hoạt) =================
// Đổi số chân này thành BẤT KỲ GPIO nào bạn muốn dùng làm ngõ vào kích hoạt
#define TRIGGER_GPIO       GPIO_NUM_4
#define TRIGGER_DEBOUNCE_MS 300   // bo qua cac ngat lien tiep trong khoang thoi gian nay (chong rung phim)

static sx127x_t lora_dev;
static gps_fix_t gps_fix = { .fix_valid = false };
static QueueHandle_t gpio_evt_queue = NULL;
static uint32_t so_lan_kich_hoat = 0;

// ================= Ngắt GPIO =================
// Hàm chạy trong ngữ cảnh ngắt (ISR): CHỈ đẩy số chân vào queue, không làm gì
// nặng/tốn thời gian ở đây (không log, không gọi hàm blocking) theo đúng
// khuyến nghị của ESP-IDF cho GPIO ISR.
static void IRAM_ATTR trigger_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void khoi_tao_chan_trigger(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIGGER_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,     // doi thanh GPIO_PULLDOWN_ENABLE + intr_type
                                               // GPIO_INTR_POSEDGE neu tin hieu kich hoat la muc CAO
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,        // ngat khi tin hieu chuyen tu CAO -> THAP (vd nut nhan keo GND)
    };
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_install_isr_service(0);
    gpio_isr_handler_add(TRIGGER_GPIO, trigger_isr_handler, (void *)TRIGGER_GPIO);

    ESP_LOGI(TAG, "Da cau hinh ngat GPIO%d lam chan trigger (canh xuong, chong rung %dms)",
             TRIGGER_GPIO, TRIGGER_DEBOUNCE_MS);
}

// ================= UART GPS =================
static void khoi_tao_gps_uart(void) {
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
}

// Task nền: đọc GPS liên tục, luôn cập nhật gps_fix mới nhất
static void task_doc_gps(void *arg) {
    uint8_t byte;
    while (true) {
        int len = uart_read_bytes(GPS_UART_NUM, &byte, 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            nmea_feed_char((char)byte, &gps_fix);
        }
    }
}

// Task nền: in log chan doan GPS moi 5 giay - giup phan biet "UART khong
// nhan duoc gi" voi "GPS co du lieu nhung chua fix" (2 nguyen nhan khac nhau)
static void task_debug_gps(void *arg) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        unsigned long so_byte = nmea_debug_so_byte_da_nhan();
        unsigned long so_gga = nmea_debug_so_cau_gga_da_parse();

        ESP_LOGI(TAG, "---- [DEBUG GPS] ----");
        ESP_LOGI(TAG, "  So byte da nhan tu UART   : %lu", so_byte);
        ESP_LOGI(TAG, "  So cau $GPGGA/$GNGGA da parse : %lu", so_gga);
        ESP_LOGI(TAG, "  Dang co fix ngay bay gio? : %s", gps_fix.fix_valid ? "CO" : "CHUA");
        if (gps_fix.fix_valid) {
            ESP_LOGI(TAG, "  So ve tinh dang dung      : %d", gps_fix.satellites);
        }

        if (so_byte < 10) {
            ESP_LOGW(TAG, "  => UART KHONG NHAN DUOC GI CA! Kiem tra day RX/TX,");
            ESP_LOGW(TAG, "     nguon cap cho module GPS, hoac sai baud rate (dang dat %d).", GPS_BAUDRATE);
        } else if (so_gga == 0) {
            ESP_LOGW(TAG, "  => Co nhan duoc du lieu qua UART nhung KHONG THAY cau GGA hop le.");
            ESP_LOGW(TAG, "     Rat co the SAI BAUD RATE, hoac dau RX/TX bi dao nguoc.");
        } else if (!gps_fix.fix_valid) {
            ESP_LOGI(TAG, "  => GPS dang hoat dong binh thuong, chi la CHUA BAT DUOC VE TINH.");
            ESP_LOGI(TAG, "     Dem module ra ngoai troi thoang, cho vai phut roi thu lai.");
        } else {
            ESP_LOGI(TAG, "  => GPS da/dang co fix, moi thu binh thuong.");
        }
        ESP_LOGI(TAG, "----------------------");
    }
}

// ================= Xử lý khi có tín hiệu trigger =================
static void gui_toa_do_qua_lora(void) {
    char msg[80];

    if (gps_fix.fix_valid) {
        snprintf(msg, sizeof(msg), "TRIGGER,%lu,%.6f,%.6f,sats=%d,hdop=%.1f",
                 (unsigned long)so_lan_kich_hoat, gps_fix.lat, gps_fix.lon,
                 gps_fix.satellites, gps_fix.hdop);
    } else {
        snprintf(msg, sizeof(msg), "TRIGGER,%lu,NO_FIX", (unsigned long)so_lan_kich_hoat);
    }

    bool ok = sx127x_send_packet(&lora_dev, (const uint8_t *)msg, (uint8_t)strlen(msg), 2000);
    ESP_LOGI(TAG, "Da gui qua LoRa (%s): %s", ok ? "OK" : "THAT BAI", msg);
}

// Task lắng nghe queue từ ISR, xử lý debounce rồi gửi LoRa
static void task_xu_ly_trigger(void *arg) {
    uint32_t gpio_num;
    int64_t lan_cuoi_kich_hoat_ms = 0;

    while (true) {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
            int64_t now_ms = esp_timer_get_time() / 1000;
            if ((now_ms - lan_cuoi_kich_hoat_ms) < TRIGGER_DEBOUNCE_MS) {
                continue; // bo qua, coi la rung phim / nhieu tin hieu lien tiep
            }
            lan_cuoi_kich_hoat_ms = now_ms;
            so_lan_kich_hoat++;

            ESP_LOGI(TAG, "Phat hien tin hieu tren GPIO%lu (lan thu %lu)",
                     (unsigned long)gpio_num, (unsigned long)so_lan_kich_hoat);

            gui_toa_do_qua_lora();
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Khoi dong...");

    khoi_tao_gps_uart();
    ESP_LOGI(TAG, "Da khoi tao GPS UART (ATK-S1216, baud %d)", GPS_BAUDRATE);

    if (!sx127x_init(&lora_dev, SPI2_HOST, LORA_SCK, LORA_MISO, LORA_MOSI,
                      LORA_CS, LORA_RST, LORA_DIO0, LORA_FREQUENCY_HZ)) {
        ESP_LOGE(TAG, "Khong khoi tao duoc LoRa! Kiem tra day noi SPI. Dung chuong trinh.");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // Chan doan: kiem tra do tin cay SPI truoc khi cau hinh cac thong so LoRa that su
    // (tu test se ghi tam thoi len thanh ghi SYNC_WORD, nen phai chay TRUOC khi dat gia
    // tri sync word that, tranh bi ghi de)
    sx127x_self_test(&lora_dev);

    sx127x_set_sync_word(&lora_dev, LORA_SYNC_WORD);
    sx127x_set_spreading_factor(&lora_dev, LORA_SPREADING_FACTOR);
    sx127x_set_bandwidth(&lora_dev, LORA_BANDWIDTH_HZ);
    sx127x_set_coding_rate(&lora_dev, LORA_CODING_RATE);
    sx127x_set_tx_power(&lora_dev, 17); // TAM HA TU 20 XUONG 17dBm de giam dong dinh tieu thu khi phat,
                                          // test xem loi TX timeout co phai do sut ap nguon khong
    ESP_LOGI(TAG, "LoRa khoi tao thanh cong!");

    khoi_tao_chan_trigger();

    xTaskCreate(task_doc_gps, "doc_gps", 3072, NULL, 5, NULL);
    xTaskCreate(task_debug_gps, "debug_gps", 3072, NULL, 3, NULL);
    xTaskCreate(task_xu_ly_trigger, "xu_ly_trigger", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "San sang. Dang cho tin hieu tren GPIO%d...", TRIGGER_GPIO);
}
