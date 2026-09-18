/*
 * ============================================================
 *  DRIVER LoRa SX127x (module RA-02) - CÀI ĐẶT
 * ============================================================
 * Tham chiếu thanh ghi theo datasheet Semtech SX1276/77/78/79.
 * Giao tiếp SPI mode 0, mỗi thanh ghi 8-bit, ghi = (addr|0x80),
 * đọc = (addr&0x7F).
 * ============================================================
 */
#include "sx127x.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "sx127x";

// Địa chỉ thanh ghi
#define REG_FIFO                 0x00
#define REG_OP_MODE               0x01
#define REG_FRF_MSB               0x06
#define REG_FRF_MID               0x07
#define REG_FRF_LSB               0x08
#define REG_PA_CONFIG             0x09
#define REG_LNA                   0x0c
#define REG_FIFO_ADDR_PTR         0x0d
#define REG_FIFO_TX_BASE_ADDR     0x0e
#define REG_FIFO_RX_BASE_ADDR     0x0f
#define REG_FIFO_RX_CURRENT_ADDR  0x10
#define REG_IRQ_FLAGS             0x12
#define REG_RX_NB_BYTES           0x13
#define REG_PKT_SNR_VALUE         0x19
#define REG_PKT_RSSI_VALUE        0x1a
#define REG_MODEM_CONFIG_1        0x1d
#define REG_MODEM_CONFIG_2        0x1e
#define REG_PREAMBLE_MSB          0x20
#define REG_PREAMBLE_LSB          0x21
#define REG_PAYLOAD_LENGTH        0x22
#define REG_MODEM_CONFIG_3        0x26
#define REG_DETECTION_OPTIMIZE    0x31
#define REG_DETECTION_THRESHOLD   0x37
#define REG_SYNC_WORD             0x39
#define REG_DIO_MAPPING_1         0x40
#define REG_VERSION               0x42
#define REG_PA_DAC                0x4d

// Chế độ hoạt động (REG_OP_MODE)
#define MODE_LONG_RANGE_MODE      0x80  // bit7 = 1 -> chế độ LoRa (không phải FSK)
#define MODE_SLEEP                0x00
#define MODE_STDBY                0x01
#define MODE_TX                   0x03
#define MODE_RX_CONTINUOUS        0x05

#define IRQ_TX_DONE_MASK          0x08
#define IRQ_RX_DONE_MASK          0x40
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20

// Giao tiếp SPI mức thấp 
static void cs_select(sx127x_t *dev)   { gpio_set_level(dev->pin_cs, 0); }
static void cs_deselect(sx127x_t *dev) { gpio_set_level(dev->pin_cs, 1); }

static void write_reg(sx127x_t *dev, uint8_t addr, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)(addr | 0x80), value };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    cs_select(dev);
    spi_device_transmit(dev->spi, &t);
    cs_deselect(dev);
}

static uint8_t read_reg(sx127x_t *dev, uint8_t addr) {
    uint8_t tx[2] = { (uint8_t)(addr & 0x7f), 0x00 };
    uint8_t rx[2] = { 0, 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    cs_select(dev);
    spi_device_transmit(dev->spi, &t);
    cs_deselect(dev);
    return rx[1];
}

static void write_fifo_burst(sx127x_t *dev, const uint8_t *data, uint8_t len) {
    uint8_t buf[257];
    buf[0] = REG_FIFO | 0x80;
    if (len > 0) {
        memcpy(&buf[1], data, len);
    }
    spi_transaction_t t = { .length = (size_t)(len + 1) * 8, .tx_buffer = buf };
    cs_select(dev);
    spi_device_transmit(dev->spi, &t);
    cs_deselect(dev);
}

static void read_fifo_burst(sx127x_t *dev, uint8_t *buf, uint8_t len) {
    uint8_t tx[257] = { 0 };
    uint8_t rx[257] = { 0 };
    tx[0] = REG_FIFO & 0x7f;

    spi_transaction_t t = { .length = (size_t)(len + 1) * 8, .tx_buffer = tx, .rx_buffer = rx };
    cs_select(dev);
    spi_device_transmit(dev->spi, &t);
    cs_deselect(dev);

    if (len > 0) {
        memcpy(buf, &rx[1], len);
    }
}

static void set_mode(sx127x_t *dev, uint8_t mode) {
    write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | mode);
}

// API

bool sx127x_init(sx127x_t *dev, spi_host_device_t host,
                  gpio_num_t sck, gpio_num_t miso, gpio_num_t mosi,
                  gpio_num_t cs, gpio_num_t reset, gpio_num_t dio0,
                  int32_t frequency_hz) {
    dev->pin_cs = cs;
    dev->pin_reset = reset;
    dev->pin_dio0 = dio0;
    dev->frequency_hz = frequency_hz;

    // Cấu hình chân CS/RESET làm output thủ công (không dùng CS phần cứng của SPI
    // để chủ động giữ CS thấp xuyên suốt các transaction địa chỉ+dữ liệu FIFO burst)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << cs) | (1ULL << reset),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(cs, 1);

    spi_bus_config_t buscfg = {
        .miso_io_num = miso,
        .mosi_io_num = mosi,
        .sclk_io_num = sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 264,
    };
    // Cho phép gọi lại init nhiều lần an toàn: bỏ qua lỗi "bus đã cài đặt"
    esp_err_t err = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize loi: %d", err);
        return false;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4 * 1000 * 1000, // 4MHz, an toan cho SX127x (toi da 10MHz)
        .mode = 0,
        .spics_io_num = -1,   // dieu khien CS thu cong
        .queue_size = 4,
    };
    err = spi_bus_add_device(host, &devcfg, &dev->spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device loi: %d", err);
        return false;
    }

    // Reset phần cứng module LoRa
    gpio_set_level(reset, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(reset, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); // tang thoi gian cho, mot so module clone can lau hon 10ms

    uint8_t version = read_reg(dev, REG_VERSION);
    if (version != 0x12) {
        ESP_LOGE(TAG, "Khong tim thay SX127x (doc REG_VERSION=0x%02x, ky vong 0x12). Kiem tra day noi SPI!", version);
        return false;
    }

    set_mode(dev, MODE_SLEEP);

    // Đặt tần số
    uint64_t frf = ((uint64_t)frequency_hz << 19) / 32000000ULL;
    write_reg(dev, REG_FRF_MSB, (uint8_t)(frf >> 16));
    write_reg(dev, REG_FRF_MID, (uint8_t)(frf >> 8));
    write_reg(dev, REG_FRF_LSB, (uint8_t)(frf >> 0));

    // FIFO: dùng toàn bộ 256 byte, base addr TX=0, RX=0
    write_reg(dev, REG_FIFO_TX_BASE_ADDR, 0x00);
    write_reg(dev, REG_FIFO_RX_BASE_ADDR, 0x00);

    // Bật LNA boost, AGC tự động
    write_reg(dev, REG_LNA, read_reg(dev, REG_LNA) | 0x03);
    write_reg(dev, REG_MODEM_CONFIG_3, 0x04); // AGC auto on

    // Công suất phát mặc định + bật PA_BOOST (RA-02 dùng chân PA_BOOST)
    sx127x_set_tx_power(dev, 17);

    set_mode(dev, MODE_STDBY);

    ESP_LOGI(TAG, "Khoi tao SX127x thanh cong, tan so %ld Hz", (long)frequency_hz);
    return true;
}

void sx127x_set_sync_word(sx127x_t *dev, uint8_t sync_word) {
    write_reg(dev, REG_SYNC_WORD, sync_word);
}

void sx127x_set_spreading_factor(sx127x_t *dev, uint8_t sf) {
    if (sf < 6) sf = 6;
    if (sf > 12) sf = 12;
    // Ngưỡng tối ưu hóa phát hiện theo datasheet khi SF6
    if (sf == 6) {
        write_reg(dev, REG_DETECTION_OPTIMIZE, 0xc5);
        write_reg(dev, REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        write_reg(dev, REG_DETECTION_OPTIMIZE, 0xc3);
        write_reg(dev, REG_DETECTION_THRESHOLD, 0x0a);
    }
    uint8_t reg2 = read_reg(dev, REG_MODEM_CONFIG_2);
    write_reg(dev, REG_MODEM_CONFIG_2, (reg2 & 0x0f) | ((sf << 4) & 0xf0));
}

void sx127x_set_bandwidth(sx127x_t *dev, long bandwidth_hz) {
    uint8_t bw_bits;
    if (bandwidth_hz <= 7800)       bw_bits = 0;
    else if (bandwidth_hz <= 10400) bw_bits = 1;
    else if (bandwidth_hz <= 15600) bw_bits = 2;
    else if (bandwidth_hz <= 20800) bw_bits = 3;
    else if (bandwidth_hz <= 31250) bw_bits = 4;
    else if (bandwidth_hz <= 41700) bw_bits = 5;
    else if (bandwidth_hz <= 62500) bw_bits = 6;
    else if (bandwidth_hz <= 125000) bw_bits = 7;
    else if (bandwidth_hz <= 250000) bw_bits = 8;
    else                             bw_bits = 9; // 500kHz

    uint8_t reg1 = read_reg(dev, REG_MODEM_CONFIG_1);
    write_reg(dev, REG_MODEM_CONFIG_1, (reg1 & 0x0f) | (bw_bits << 4));
}

void sx127x_set_coding_rate(sx127x_t *dev, uint8_t denominator) {
    if (denominator < 5) denominator = 5;
    if (denominator > 8) denominator = 8;
    uint8_t cr = denominator - 4; // 4/5..4/8 -> 1..4
    uint8_t reg1 = read_reg(dev, REG_MODEM_CONFIG_1);
    write_reg(dev, REG_MODEM_CONFIG_1, (reg1 & 0xf1) | (cr << 1));
}

void sx127x_set_tx_power(sx127x_t *dev, uint8_t level_dbm) {
    if (level_dbm > 20) level_dbm = 20;
    if (level_dbm < 2)  level_dbm = 2;
    if (level_dbm > 17) {
        // Dùng chế độ +20dBm qua PA_DAC (chi phí dòng tiêu thụ cao hơn)
        write_reg(dev, REG_PA_DAC, 0x87);
        write_reg(dev, REG_PA_CONFIG, 0x80 | 0x70 | (level_dbm - 5));
    } else {
        write_reg(dev, REG_PA_DAC, 0x84);
        write_reg(dev, REG_PA_CONFIG, 0x80 | (level_dbm - 2)); // PA_BOOST, max power
    }
}

bool sx127x_send_packet(sx127x_t *dev, const uint8_t *data, uint8_t len, uint32_t timeout_ms) {
    set_mode(dev, MODE_STDBY);
    write_reg(dev, REG_FIFO_ADDR_PTR, 0x00);
    write_fifo_burst(dev, data, len);
    write_reg(dev, REG_PAYLOAD_LENGTH, len);

    write_reg(dev, REG_IRQ_FLAGS, 0xff); // xoa co IRQ cu
    set_mode(dev, MODE_TX);

    TickType_t start = xTaskGetTickCount();
    while (true) {
        uint8_t irq = read_reg(dev, REG_IRQ_FLAGS);
        if (irq & IRQ_TX_DONE_MASK) {
            write_reg(dev, REG_IRQ_FLAGS, 0xff);
            set_mode(dev, MODE_STDBY);
            return true;
        }
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
            ESP_LOGW(TAG, "Gui goi LoRa timeout");
            set_mode(dev, MODE_STDBY);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void sx127x_start_receive(sx127x_t *dev) {
    write_reg(dev, REG_FIFO_ADDR_PTR, 0x00);
    set_mode(dev, MODE_RX_CONTINUOUS);
}

int sx127x_receive_packet(sx127x_t *dev, uint8_t *buf, uint8_t max_len, int *rssi_out, float *snr_out) {
    uint8_t irq = read_reg(dev, REG_IRQ_FLAGS);
    if (!(irq & IRQ_RX_DONE_MASK)) {
        return 0; // chua co goi moi
    }
    write_reg(dev, REG_IRQ_FLAGS, 0xff); // xoa toan bo co, kể cả RX_DONE

    if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK) {
        ESP_LOGW(TAG, "Goi LoRa nhan bi loi CRC, bo qua");
        return 0;
    }

    uint8_t len = read_reg(dev, REG_RX_NB_BYTES);
    if (len > max_len) len = max_len;

    uint8_t cur_addr = read_reg(dev, REG_FIFO_RX_CURRENT_ADDR);
    write_reg(dev, REG_FIFO_ADDR_PTR, cur_addr);
    read_fifo_burst(dev, buf, len);

    // Tính RSSI/SNR theo datasheet (tần số <525MHz dùng offset -164)
    int8_t snr_raw = (int8_t)read_reg(dev, REG_PKT_SNR_VALUE);
    float snr = snr_raw / 4.0f;
    int rssi_reg = read_reg(dev, REG_PKT_RSSI_VALUE);
    int rssi = (dev->frequency_hz < 525000000) ? (rssi_reg - 164) : (rssi_reg - 157);
    if (snr < 0) rssi += (int)snr; // hieu chinh khi tin hieu duoi muc nhieu, theo khuyen nghi Semtech

    if (rssi_out) *rssi_out = rssi;
    if (snr_out) *snr_out = snr;

    return len;
}
