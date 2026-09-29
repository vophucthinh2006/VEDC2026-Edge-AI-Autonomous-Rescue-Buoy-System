/*
 * ============================================================
 *  PARSER NMEA TỐI GIẢN CHO GPS NEO-6M
 * ============================================================
 * Chỉ parse câu $GPGGA/$GNGGA (đủ thông tin: fix, lat, lon, số
 * vệ tinh, HDOP) - không cần thư viện ngoài như TinyGPS++ (đó
 * là thư viện Arduino, không dùng được trong ESP-IDF thuần).
 * ============================================================
 */
#pragma once
#include <stdbool.h>

typedef struct {
    bool fix_valid;
    double lat;     // độ thập phân, dương = Bắc, âm = Nam
    double lon;     // độ thập phân, dương = Đông, âm = Tây
    int satellites;
    float hdop;
} gps_fix_t;

/*
 * Đưa 1 ký tự nhận được từ UART vào bộ đệm dòng nội bộ. Khi gặp
 * ký tự xuống dòng, hàm tự parse câu NMEA vừa nhận (nếu là GGA)
 * và cập nhật *fix. Trả về true nếu vừa parse được 1 câu GGA hợp lệ.
 */
bool nmea_feed_char(char c, gps_fix_t *fix);

// ---------- Cac bo dem chan doan (debug) ----------
// So tong byte da nhan tu UART (bao gom moi ky tu, khong chi cau GGA)
unsigned long nmea_debug_so_byte_da_nhan(void);
// So cau $GPGGA/$GNGGA hop le da parse duoc (ke ca khi fix_valid=false)
unsigned long nmea_debug_so_cau_gga_da_parse(void);
