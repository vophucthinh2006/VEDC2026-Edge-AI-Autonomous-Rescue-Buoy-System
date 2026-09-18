/*
 * ============================================================
 *  DRIVER LoRa SX127x (module RA-02) CHO ESP-IDF
 * ============================================================
 * Driver tối giản dùng SPI Master của ESP-IDF để giao tiếp với
 * module RA-02 (chip SX1278). Dùng chung cho cả node phao (gửi)
 * và ESP32 trạm bờ (nhận).
 *
 * Không phụ thuộc Arduino - chỉ dùng driver/spi_master.h và
 * driver/gpio.h chuẩn của ESP-IDF.
 * ============================================================
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

typedef struct {
    spi_device_handle_t spi;
    gpio_num_t pin_cs;
    gpio_num_t pin_reset;
    gpio_num_t pin_dio0;
    int32_t frequency_hz;
} sx127x_t;

/*
 * Khởi tạo module LoRa.
 *   host        : SPI host (thường dùng SPI2_HOST trên ESP32)
 *   sck/miso/mosi/cs/reset/dio0 : chân GPIO tương ứng
 *   frequency_hz: tần số hoạt động, vd 433000000 cho 433MHz
 * Trả về true nếu khởi tạo thành công (đọc được thanh ghi VERSION = 0x12).
 */
bool sx127x_init(sx127x_t *dev, spi_host_device_t host,
                  gpio_num_t sck, gpio_num_t miso, gpio_num_t mosi,
                  gpio_num_t cs, gpio_num_t reset, gpio_num_t dio0,
                  int32_t frequency_hz);

void sx127x_set_sync_word(sx127x_t *dev, uint8_t sync_word);
void sx127x_set_spreading_factor(sx127x_t *dev, uint8_t sf);   // 6-12
void sx127x_set_bandwidth(sx127x_t *dev, long bandwidth_hz);   // vd 125000
void sx127x_set_coding_rate(sx127x_t *dev, uint8_t denominator); // 5-8 (4/5 .. 4/8)
void sx127x_set_tx_power(sx127x_t *dev, uint8_t level_dbm);    // 2-20

/*
 * Gửi 1 gói tin (chặn/blocking cho tới khi gửi xong hoặc timeout).
 * Trả về true nếu gửi thành công.
 */
bool sx127x_send_packet(sx127x_t *dev, const uint8_t *data, uint8_t len, uint32_t timeout_ms);

/*
 * Kiểm tra và đọc gói tin nếu có (không chặn - gọi liên tục trong vòng lặp/task).
 * Trả về số byte nhận được (>0 nếu có gói mới), 0 nếu chưa có gói nào.
 * rssi (dBm) và snr (dB) chỉ hợp lệ khi trả về > 0.
 */
int sx127x_receive_packet(sx127x_t *dev, uint8_t *buf, uint8_t max_len, int *rssi_out, float *snr_out);

/* Bật chế độ nhận liên tục (gọi 1 lần trước khi loop gọi sx127x_receive_packet) */
void sx127x_start_receive(sx127x_t *dev);
